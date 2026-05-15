/*
 * Bare-metal HAL for EmbedMCP.
 *
 * Single-threaded cooperative target: STM32 / ARM Cortex-M / similar.
 * Memory is newlib malloc/free. The sync primitives are no-ops because
 * there is no preemption -- the upper layers only ever take a lock from
 * the cooperative main loop, which has no contender.
 *
 * Time and delay are wired through the host's smtc HAL, which both Linux
 * and the STM32 examples expose. The MCU HAL declarations are forward-
 * declared rather than #included to avoid pulling smtc_hal headers into
 * EmbedMCP's third-party tree.
 *
 * Networking and threading are absent. EmbedMCP's STM32 transport
 * (bare_uart_transport) does not exercise either: it is poll-driven and
 * single-connection. Setting the function pointers to NULL is the
 * idiomatic way to communicate "this platform does not support it" --
 * upper layers must check capabilities (.has_threading / .has_networking)
 * before invoking. tool_registry's HAL-mutex refactor calls sync.mutex_*
 * unconditionally, so those must be present (as no-ops).
 */

#include "hal/platform_hal.h"
#include "hal/hal_common.h"

#ifdef MCP_PLATFORM_BARE_METAL

#include <stdint.h>
#include <stdlib.h>     /* malloc / free / realloc */

/*
 * Forward-declare the smtc HAL primitives we wrap. They are implemented
 * in smtc_hal_mcu.c on STM32 and in smtc_hal_linux on Linux; either way
 * the link symbol is the same. Avoids pulling smtc_hal_mcu.h into a
 * third-party tree.
 */
extern uint32_t smtc_modem_hal_get_time_in_ms(void);
extern void     hal_mcu_delay_ms(uint32_t ms);

/* ---------------------------------------------------------------------------
 * Memory: newlib heap (configured by the linker script's _Min_Heap_Size).
 * ------------------------------------------------------------------------- */

static void* bm_alloc(size_t size)              { return malloc(size); }
static void  bm_free(void* ptr)                  { free(ptr); }
static void* bm_realloc(void* ptr, size_t size)  { return realloc(ptr, size); }
static size_t bm_get_free_size(void)             { return 0; /* unknown */ }

/* ---------------------------------------------------------------------------
 * Synchronization: no-ops. Cooperative single-threaded means no preemption,
 * so a lock acquired by the main loop cannot be re-entered. The HAL still
 * needs valid function pointers (tool_registry calls them unconditionally),
 * so the create/destroy hand back a sentinel handle and lock/unlock are
 * empty.
 * ------------------------------------------------------------------------- */

static int bm_mutex_create(void** mutex) {
    /* Hand out a non-NULL sentinel so callers' NULL-checks pass. The
     * value is never dereferenced. */
    static int sentinel;
    if (mutex) *mutex = (void*) &sentinel;
    return 0;
}
static int bm_mutex_lock(void* mutex)    { (void) mutex; return 0; }
static int bm_mutex_unlock(void* mutex)  { (void) mutex; return 0; }
static int bm_mutex_destroy(void* mutex) { (void) mutex; return 0; }

/* ---------------------------------------------------------------------------
 * Time: wrap the existing smtc HAL.
 * ------------------------------------------------------------------------- */

static uint32_t bm_get_tick_ms(void)    { return smtc_modem_hal_get_time_in_ms(); }
static uint64_t bm_get_time_us(void)    {
    /* 1 ms resolution from smtc_hal; promote to microseconds. */
    return ((uint64_t) smtc_modem_hal_get_time_in_ms()) * 1000u;
}
static void     bm_delay_ms(uint32_t ms){ hal_mcu_delay_ms(ms); }
static void     bm_delay_us(uint32_t us){
    /* Coarse: snap up to a millisecond. No microsecond busy-wait
     * primitive in smtc_hal_mcu. Acceptable for EmbedMCP's use
     * (currently delay is only called by HTTP transport's poll, which
     * we don't compile in). */
    if (us == 0) return;
    hal_mcu_delay_ms((us + 999u) / 1000u);
}

/* ---------------------------------------------------------------------------
 * Platform identity + capabilities.
 * ------------------------------------------------------------------------- */

static int  bm_platform_init(void)    { return 0; }
static void bm_platform_cleanup(void) { /* nothing to clean up */ }

static const mcp_platform_capabilities_t bm_capabilities = {
    .has_dynamic_memory = true,
    .has_threading      = false,
    .has_networking     = false,
    .max_memory_kb      = 96,    /* L476 main SRAM; informational */
    .max_connections    = 1,     /* single UART connection */
    .tick_frequency_hz  = 1000,  /* smtc_modem_hal_get_time_in_ms */
};

static const mcp_platform_hal_t bare_metal_hal = {
    .platform_name = "BareMetal",
    .version       = "1.0.0",
    .capabilities  = bm_capabilities,

    .memory = {
        .alloc         = bm_alloc,
        .free          = bm_free,
        .realloc       = bm_realloc,
        .get_free_size = bm_get_free_size,
    },

    /* No threading -- single-threaded cooperative. */
    .thread = {
        .create   = NULL,
        .join     = NULL,
        .yield    = NULL,
        .sleep_ms = bm_delay_ms,    /* harmless alias; nothing should call it */
        .get_id   = NULL,
    },

    .sync = {
        .mutex_create  = bm_mutex_create,
        .mutex_destroy = bm_mutex_destroy,
        .mutex_lock    = bm_mutex_lock,
        .mutex_unlock  = bm_mutex_unlock,
    },

    .time = {
        .get_tick_ms = bm_get_tick_ms,
        .get_time_us = bm_get_time_us,
        .delay_ms    = bm_delay_ms,
        .delay_us    = bm_delay_us,
    },

    /* No network stack. Upper layers must check has_networking. The
     * bare_uart transport does not invoke any of these. */
    .network = {
        .http_server_start   = NULL,
        .http_response_send  = NULL,
        .network_poll        = NULL,
        .http_server_stop    = NULL,
        .socket_create       = NULL,
        .socket_bind         = NULL,
        .socket_send         = NULL,
        .socket_recv         = NULL,
        .socket_close        = NULL,
    },

    .init    = bm_platform_init,
    .cleanup = bm_platform_cleanup,
};

HAL_IMPLEMENT_EXPORTS(bare_metal_hal, bm_capabilities,
                      bm_platform_init, bm_platform_cleanup)

#endif /* MCP_PLATFORM_BARE_METAL */
