/*
 * Bare-metal UART transport for EmbedMCP. See bare_uart_transport.h for
 * the design notes. All memory allocation goes through the platform
 * HAL so this file is portable C99 with no POSIX dependencies.
 */
#include "transport/bare_uart_transport.h"
#include "hal/platform_hal.h"
#include "hal/hal_common.h"
#include "utils/logging.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define DEFAULT_LINE_BUFFER_SIZE   4096u
#define BARE_UART_CONNECTION_ID    "bare-uart-0"

/* Per-transport private state. */
typedef struct {
    mcp_bare_uart_io_t  io;
    int                 io_valid;     /* 0 until mcp_bare_uart_transport_set_io */
    mcp_connection_t   *connection;   /* single connection, like stdio */

    char    *line_buffer;
    size_t   line_buffer_capacity;
    size_t   line_buffer_used;
    int      line_overflow;           /* drop bytes until next \n on overflow */
    /* A complete line was assembled by mcp_bare_uart_transport_pump() and is
     * waiting for the next poll() to dispatch it. pump() deliberately does not
     * dispatch (see its comment in the header), so the line has to be parked
     * somewhere; line_buffer[0..line_buffer_used) is it. */
    int      line_ready;
} mcp_bare_uart_transport_data_t;

/* Per-connection private state (kept for symmetry with stdio transport). */
typedef struct {
    int is_connected;
} mcp_bare_uart_connection_data_t;

/* Forward declarations of vtable impls. */
static int  bare_uart_init_impl(mcp_transport_t *transport,
                                const mcp_transport_config_t *config);
static int  bare_uart_start_impl(mcp_transport_t *transport);
static int  bare_uart_stop_impl(mcp_transport_t *transport);
static int  bare_uart_send_impl(mcp_connection_t *connection,
                                const char *message, size_t length);
static int  bare_uart_close_connection_impl(mcp_connection_t *connection);
static int  bare_uart_get_stats_impl(mcp_transport_t *transport, void *stats);
static void bare_uart_cleanup_impl(mcp_transport_t *transport);

const mcp_transport_interface_t mcp_bare_uart_transport_interface = {
    .init             = bare_uart_init_impl,
    .start            = bare_uart_start_impl,
    .stop             = bare_uart_stop_impl,
    .send             = bare_uart_send_impl,
    .close_connection = bare_uart_close_connection_impl,
    .get_stats        = bare_uart_get_stats_impl,
    .cleanup          = bare_uart_cleanup_impl,
};

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

static mcp_bare_uart_transport_data_t *
get_data(mcp_transport_t *transport)
{
    if (!transport) return NULL;
    return (mcp_bare_uart_transport_data_t *) transport->private_data;
}

static mcp_connection_t *
bare_uart_connection_create(mcp_transport_t *transport)
{
    const mcp_platform_hal_t *hal = mcp_platform_get_hal();
    if (!hal) return NULL;

    mcp_connection_t *connection = hal->memory.alloc(sizeof(mcp_connection_t));
    if (!connection) return NULL;
    memset(connection, 0, sizeof(*connection));

    mcp_bare_uart_connection_data_t *conn_data =
        hal->memory.alloc(sizeof(mcp_bare_uart_connection_data_t));
    if (!conn_data) {
        hal->memory.free(connection);
        return NULL;
    }
    memset(conn_data, 0, sizeof(*conn_data));
    conn_data->is_connected = 1;

    connection->transport      = transport;
    connection->connection_id  = hal_strdup(hal, BARE_UART_CONNECTION_ID);
    connection->session_id     = NULL;
    connection->is_active      = true;
    connection->created_time   = 0;   /* time() avoided for bare-metal portability */
    connection->last_activity  = 0;
    connection->private_data   = conn_data;

    return connection;
}

static void
bare_uart_connection_destroy(mcp_connection_t *connection)
{
    if (!connection) return;
    const mcp_platform_hal_t *hal = mcp_platform_get_hal();
    if (!hal) return;
    hal_free(hal, connection->connection_id);
    hal_free(hal, connection->session_id);
    hal->memory.free(connection->private_data);
    hal->memory.free(connection);
}

static void
dispatch_line(mcp_transport_t *transport, const char *line, size_t length)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data || !data->connection) return;

    data->connection->messages_received++;
    data->connection->bytes_received += length;

    if (transport->on_message) {
        transport->on_message(line, length, data->connection,
                              transport->user_data);
    }
    transport->messages_received++;
}

static void
handle_error(mcp_transport_t *transport, int error_code, const char *message)
{
    if (!transport) return;
    if (transport->on_error) {
        transport->on_error(transport, error_code, message,
                            transport->user_data);
    }
}

/* --------------------------------------------------------------------------
 * vtable: init / start / stop / cleanup
 * -------------------------------------------------------------------------- */

static int
bare_uart_init_impl(mcp_transport_t *transport,
                    const mcp_transport_config_t *config)
{
    if (!transport) return -1;
    const mcp_platform_hal_t *hal = mcp_platform_get_hal();
    if (!hal) return -1;

    mcp_bare_uart_transport_data_t *data =
        hal->memory.alloc(sizeof(mcp_bare_uart_transport_data_t));
    if (!data) return -1;
    memset(data, 0, sizeof(*data));

    /* Stash a copy of the config so embed_mcp.c can inspect it later. */
    if (config) {
        transport->config = hal->memory.alloc(sizeof(mcp_transport_config_t));
        if (transport->config) {
            *transport->config = *config;
        }
    }

    size_t buf_size = (config && config->max_message_size > 0)
                          ? config->max_message_size
                          : DEFAULT_LINE_BUFFER_SIZE;
    data->line_buffer = hal->memory.alloc(buf_size);
    if (!data->line_buffer) {
        hal->memory.free(data);
        return -1;
    }
    data->line_buffer_capacity = buf_size;
    data->line_buffer_used     = 0;
    data->line_overflow        = 0;

    transport->private_data = data;
    return 0;
}

static int
bare_uart_start_impl(mcp_transport_t *transport)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data) return -1;
    if (!data->io_valid) {
        mcp_log_error("bare_uart: I/O callbacks not set "
                      "(call mcp_bare_uart_transport_set_io before start)");
        return -1;
    }

    if (data->io.init) {
        int rc = data->io.init(data->io.user);
        if (rc != 0) {
            mcp_log_error("bare_uart: io.init returned %d", rc);
            return rc;
        }
    }

    data->connection = bare_uart_connection_create(transport);
    if (!data->connection) {
        if (data->io.deinit) data->io.deinit(data->io.user);
        return -1;
    }

    if (transport->on_connection_opened) {
        transport->on_connection_opened(data->connection,
                                        transport->user_data);
    }
    transport->connections_opened++;
    return 0;
}

static int
bare_uart_stop_impl(mcp_transport_t *transport)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data) return -1;

    if (data->connection) {
        if (transport->on_connection_closed) {
            transport->on_connection_closed(data->connection,
                                            transport->user_data);
        }
        transport->connections_closed++;
        bare_uart_connection_destroy(data->connection);
        data->connection = NULL;
    }

    if (data->io.deinit) {
        data->io.deinit(data->io.user);
    }
    return 0;
}

static void
bare_uart_cleanup_impl(mcp_transport_t *transport)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    const mcp_platform_hal_t *hal = mcp_platform_get_hal();
    if (!hal) return;

    if (data) {
        if (data->connection) bare_uart_connection_destroy(data->connection);
        hal_free(hal, data->line_buffer);
        hal->memory.free(data);
    }
    if (transport && transport->config) {
        hal->memory.free(transport->config);
        transport->config = NULL;
    }
    if (transport) transport->private_data = NULL;
}

/* --------------------------------------------------------------------------
 * vtable: send / close
 * -------------------------------------------------------------------------- */

static int
bare_uart_send_impl(mcp_connection_t *connection,
                    const char *message, size_t length)
{
    if (!connection || !connection->transport || !message) return -1;
    mcp_bare_uart_transport_data_t *data = get_data(connection->transport);
    if (!data || !data->io_valid || !data->io.tx_blocking) return -1;

    int sent = data->io.tx_blocking((const uint8_t *) message, length,
                                    data->io.user);
    if (sent < 0) {
        handle_error(connection->transport, sent, "bare_uart tx failed");
        return sent;
    }
    static const uint8_t newline = '\n';
    int rc = data->io.tx_blocking(&newline, 1, data->io.user);
    if (rc < 0) {
        handle_error(connection->transport, rc, "bare_uart tx (newline) failed");
        return rc;
    }

    connection->messages_sent++;
    connection->bytes_sent += length + 1;
    connection->transport->messages_sent++;
    return (int) length;
}

static int
bare_uart_close_connection_impl(mcp_connection_t *connection)
{
    if (!connection) return -1;
    connection->is_active = false;
    /* The bare-UART transport keeps the single connection alive until
     * stop(); a "close" here just flags it inactive. */
    return 0;
}

static int
bare_uart_get_stats_impl(mcp_transport_t *transport, void *stats)
{
    (void) transport;
    (void) stats;
    return 0;   /* Stats are kept on transport->messages_*, expose later. */
}

/* --------------------------------------------------------------------------
 * Public API: factory, IO setter, cooperative poll
 * -------------------------------------------------------------------------- */

mcp_transport_t *
mcp_transport_create_bare_uart(void)
{
    const mcp_platform_hal_t *hal = mcp_platform_get_hal();
    if (!hal) return NULL;

    mcp_transport_t *transport = hal->memory.alloc(sizeof(mcp_transport_t));
    if (!transport) return NULL;
    memset(transport, 0, sizeof(*transport));

    transport->type      = MCP_TRANSPORT_BARE_UART;
    transport->state     = MCP_TRANSPORT_STATE_STOPPED;
    transport->interface = &mcp_bare_uart_transport_interface;

    /* Initialize private_data and line buffer via the standard init path. */
    mcp_transport_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.type             = MCP_TRANSPORT_BARE_UART;
    cfg.enable_logging   = false;
    cfg.max_message_size = DEFAULT_LINE_BUFFER_SIZE;
    cfg.max_connections  = 1;
    cfg.connection_timeout = 0;

    if (bare_uart_init_impl(transport, &cfg) != 0) {
        hal->memory.free(transport);
        return NULL;
    }
    return transport;
}

int
mcp_bare_uart_transport_set_io(mcp_transport_t *transport,
                               const mcp_bare_uart_io_t *io)
{
    if (!transport || !io) return -1;
    if (!io->rx_one_nonblocking || !io->tx_blocking) return -1;

    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data) return -1;

    data->io       = *io;   /* copy by value; struct is small */
    data->io_valid = 1;
    return 0;
}

/* ------------------------------------------------------------------------
 * Shared byte drain for poll() and pump().
 *
 * `may_dispatch` is the only difference between the two:
 *   1 (poll) -- a completed line is dispatched immediately and draining
 *               continues, so several messages can be handled per call.
 *   0 (pump) -- a completed line is parked (line_ready) and draining STOPS.
 *               Nothing is dispatched and nothing is transmitted, which is
 *               what makes pump() safe to call from inside a handler.
 * ---------------------------------------------------------------------- */
static int
bare_uart_drain(mcp_transport_t *transport, int may_dispatch)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);

    int dispatched = 0;
    for (;;) {
        uint8_t byte = 0;
        int rc = data->io.rx_one_nonblocking(&byte, data->io.user);
        if (rc == 0) break;        /* nothing available */
        if (rc < 0) {
            handle_error(transport, rc, "bare_uart rx failed");
            return rc;
        }

        if (byte == '\r') {
            continue;              /* tolerate CRLF terminators */
        }
        if (byte == '\n') {
            if (!may_dispatch) {
                /* pump(): park the completed line (or the overflow marker) and
                 * stop. The next poll() reproduces the exact behaviour it
                 * would have had if it had seen this newline itself. */
                data->line_ready = 1;
                return 1;
            }
            if (data->line_overflow) {
                /* The line was too long for the buffer (already logged when
                 * overflow began below). Reply with a JSON-RPC error instead
                 * of just dropping it — an id-less request would otherwise
                 * vanish with no response at all, and the client only learns
                 * something went wrong when its own request eventually times
                 * out. `id` is null per JSON-RPC 2.0 (we can't recover the
                 * caller's id from data we discarded). */
                if (data->connection) {
                    static const char err_fmt[] =
                        "{\"jsonrpc\":\"2.0\",\"id\":null,\"error\":{\"code\":-32700,"
                        "\"message\":\"request too large (> %zu bytes), dropped\"}}";
                    char err_line[128];
                    int n = snprintf(err_line, sizeof(err_line), err_fmt,
                                     data->line_buffer_capacity);
                    if (n > 0) {
                        bare_uart_send_impl(data->connection, err_line,
                                            (size_t)((size_t)n < sizeof(err_line) ? n : sizeof(err_line) - 1));
                    }
                }
                data->line_buffer_used = 0;
                data->line_overflow    = 0;
                continue;
            }
            data->line_buffer[data->line_buffer_used] = '\0';
            size_t line_len = data->line_buffer_used;
            /* Reset BEFORE dispatching, not after. A handler may block, and
             * pump() runs during that block and appends into this same buffer
             * — so if the reset happened afterwards it would wipe whatever
             * pump() had absorbed, and the parked request would vanish with no
             * response and no error. (Observed exactly that.)
             *
             * Safe because the raw line is dead by the time a handler blocks:
             * mcp_protocol_handle_message() calls jsonrpc_parse_message() as
             * its first action and works from the parsed message thereafter,
             * so nothing reads `line` again once the handler is running. */
            data->line_buffer_used = 0;
            if (line_len > 0) {
                dispatch_line(transport, data->line_buffer, line_len);
                dispatched++;
            }
            continue;
        }

        if (data->line_buffer_used + 1 >= data->line_buffer_capacity) {
            /* Line longer than buffer: drop characters until the next '\n'
             * to recover the framing instead of truncating + dispatching
             * a garbage frame. */
            if (!data->line_overflow) {
                mcp_log_error("bare_uart: line longer than %zu, dropping until \\n",
                              data->line_buffer_capacity);
                data->line_overflow = 1;
            }
            continue;
        }
        data->line_buffer[data->line_buffer_used++] = (char) byte;
    }
    return dispatched;
}

/* Dispatch a line parked by pump(), if any. Returns 1 if one was consumed. */
static int
bare_uart_flush_ready(mcp_transport_t *transport)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data->line_ready) return 0;
    data->line_ready = 0;

    if (data->line_overflow) {
        if (data->connection) {
            static const char err_fmt[] =
                "{\"jsonrpc\":\"2.0\",\"id\":null,\"error\":{\"code\":-32700,"
                "\"message\":\"request too large (> %zu bytes), dropped\"}}";
            char err_line[128];
            int n = snprintf(err_line, sizeof(err_line), err_fmt,
                             data->line_buffer_capacity);
            if (n > 0) {
                bare_uart_send_impl(data->connection, err_line,
                                    (size_t)((size_t)n < sizeof(err_line) ? n : sizeof(err_line) - 1));
            }
        }
        data->line_buffer_used = 0;
        data->line_overflow    = 0;
        return 0;
    }

    data->line_buffer[data->line_buffer_used] = '\0';
    size_t line_len = data->line_buffer_used;
    data->line_buffer_used = 0;      /* before dispatch — see the note in
                                      * bare_uart_drain(); this handler can
                                      * block and pump() again too. */
    int dispatched = 0;
    if (line_len > 0) {
        dispatch_line(transport, data->line_buffer, line_len);
        dispatched = 1;
    }
    return dispatched;
}

int
mcp_bare_uart_transport_poll(mcp_transport_t *transport)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data || !data->io_valid) return 0;
    if (transport->state != MCP_TRANSPORT_STATE_RUNNING &&
        transport->state != MCP_TRANSPORT_STATE_STARTING) {
        /* Caller may invoke poll() unconditionally even before start; just
         * idle until the upper layer has flipped the state. */
        if (!data->connection) return 0;
    }

    /* A line parked by pump() must go out before we read anything new, or
     * messages would be reordered. */
    int dispatched = bare_uart_flush_ready(transport);

    int rc = bare_uart_drain(transport, 1);
    if (rc < 0) return rc;
    return dispatched + rc;
}

int
mcp_bare_uart_transport_pump(mcp_transport_t *transport)
{
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data || !data->io_valid) return 0;
    if (transport->state != MCP_TRANSPORT_STATE_RUNNING &&
        transport->state != MCP_TRANSPORT_STATE_STARTING) {
        if (!data->connection) return 0;
    }
    /* Already holding a completed line: do not read further, or a second
     * request would overwrite the first. The caller's blocking wait will end
     * and poll() will drain the rest. */
    if (data->line_ready) return 0;

    return bare_uart_drain(transport, 0);
}

int
mcp_bare_uart_send_output_line(mcp_transport_t *transport, const char *line)
{
    if (!transport || !line) return -1;
    mcp_bare_uart_transport_data_t *data = get_data(transport);
    if (!data || !data->connection) return -1;
    return bare_uart_send_impl(data->connection, line, strlen(line));
}
