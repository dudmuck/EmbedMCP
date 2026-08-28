/*
 * Bare-metal UART transport for EmbedMCP.
 *
 * Drop-in alternative to the STDIO transport for environments that cannot
 * use POSIX threads or stdio FILE* streams (bare-metal MCUs, RTOS targets
 * without retargeted newlib, etc.).
 *
 * Differences from the STDIO transport:
 *   * No reader thread, no mutex. The host drives the transport
 *     cooperatively by calling mcp_bare_uart_transport_poll() from its
 *     main loop.
 *   * Byte I/O is delegated to caller-supplied callbacks; the transport
 *     itself does not touch any specific UART driver. The PER app
 *     plugs in smtc_hal_uart calls, an STM32 HAL UART driver, an FTDI
 *     bridge on host, etc.
 *
 * Framing: line-delimited JSON-RPC. Bytes are accumulated until '\n'
 * is seen, at which point the line is dispatched via transport->on_message.
 * '\r' bytes are silently dropped so '\r\n' terminators work as well.
 */
#ifndef MCP_BARE_UART_TRANSPORT_H
#define MCP_BARE_UART_TRANSPORT_H

#include "transport_interface.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Caller-supplied UART I/O callbacks. All callbacks may be NULL except
 * tx_blocking and rx_one_nonblocking, which are required. */
typedef struct {
    /* Non-blocking single-byte read.
     * Returns:
     *    1 = one byte was placed at *out_byte
     *    0 = no byte available right now
     *   <0 = transport error (the transport's on_error callback fires) */
    int (*rx_one_nonblocking)(uint8_t *out_byte, void *user);

    /* Blocking write of len bytes.
     * Returns number of bytes written (>=0) or <0 on error. The transport
     * appends a single trailing '\n' separately, so callers do not need
     * to add framing here. */
    int (*tx_blocking)(const uint8_t *data, size_t len, void *user);

    /* Optional initialization hook, invoked from
     * mcp_bare_uart_transport_start. May be NULL when the UART is
     * already initialized by the host (e.g. via smtc_hal_uart2_init in
     * a separate call). Return 0 for success, <0 to abort start. */
    int (*init)(void *user);

    /* Optional teardown hook, invoked from mcp_bare_uart_transport_stop. */
    void (*deinit)(void *user);

    /* Opaque pointer passed back to all callbacks. */
    void *user;
} mcp_bare_uart_io_t;

/* Transport interface vtable (exported for transport.c registration). */
extern const mcp_transport_interface_t mcp_bare_uart_transport_interface;

/* Factory: create a bare-UART transport. The transport is in
 * INITIAL state and has no I/O wired up; call
 * mcp_bare_uart_transport_set_io() before mcp_transport_start(). */
mcp_transport_t *mcp_transport_create_bare_uart(void);

/* Wire I/O callbacks. Must be called after create and before start.
 * The transport copies the struct contents; the io pointer itself need
 * not stay alive after this call. Returns 0 on success, <0 on error. */
int mcp_bare_uart_transport_set_io(mcp_transport_t *transport,
                                   const mcp_bare_uart_io_t *io);

/* Cooperative poll. Call from the host main loop. Drains all bytes
 * currently available from rx_one_nonblocking(), assembles complete
 * lines, and invokes transport->on_message for each. Returns the
 * number of complete messages dispatched (>=0) or <0 on transport
 * error. Returns 0 immediately if the transport is not in RUNNING
 * state -- safe to call unconditionally. */
int mcp_bare_uart_transport_poll(mcp_transport_t *transport);

/* Drain-only pump — absorb bytes WITHOUT dispatching.
 *
 * Same byte loop as poll(), except that a completed line is parked rather than
 * handed to on_message, and draining stops there. Nothing is dispatched and
 * nothing is transmitted, so this is safe to call from *inside* a tool handler
 * — which is the entire point of it.
 *
 * WHY THIS EXISTS. poll() invokes on_message synchronously, so a handler that
 * blocks (waiting on a hardware interrupt, a long operation) stops the caller's
 * RX buffer from being drained for the whole duration. Anything the client
 * sends in that window is lost once the buffer fills: measured on a 115200
 * link with a 1024-byte ISR ring, a 2103-byte request sent while a handler was
 * blocked lost exactly 1079 bytes (= 2103 - 1024) and came back as a -32700
 * parse error. Client-side chunking does not help, because with the drain
 * stopped only the total volume during the stall matters, not the rate.
 *
 * Calling this from a blocking handler's wait loop raises the absorbable
 * volume from the caller's ISR buffer to this transport's line buffer. Since a
 * line longer than the line buffer is rejected by design, that is complete
 * coverage for a single pipelined request: any request that could be accepted
 * at all can now be absorbed, regardless of how long the handler blocks.
 *
 * Returns 1 if a complete line is now parked, 0 if it merely absorbed a partial
 * line or had nothing to read, <0 on transport error. Calling it again while a
 * line is already parked is a no-op returning 0 — a second concurrent request
 * still needs poll() to run first, and would overflow otherwise.
 *
 * Ordering is preserved: the next poll() dispatches the parked line before
 * reading any new bytes. */
int mcp_bare_uart_transport_pump(mcp_transport_t *transport);

/* Optional helper: emit a line via the configured tx callback. Same
 * convention as mcp_stdio_send_output_line: appends '\n'. Used
 * internally by the vtable's send() and exposed for callers that want
 * to push out-of-band notifications. */
int mcp_bare_uart_send_output_line(mcp_transport_t *transport,
                                   const char *line);

#ifdef __cplusplus
}
#endif

#endif /* MCP_BARE_UART_TRANSPORT_H */
