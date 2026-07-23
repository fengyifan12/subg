/*
 * Copyright (c) 2022-2025 Rafael Microelectronics Inc. All rights reserved.
 * 
 * SPDX-License-Identifier: LicenseRef-RafaelMicro-Proprietary-1.0
 *
 */

#if !defined(CONFIG_BOOTLOADER_APP)
#include <FreeRTOS.h>
#include <task.h>
#endif
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hosal_uart.h"
#include "mcu.h"

#ifndef CONFIG_UART_STDIO_PORT
#define CONFIG_UART_STDIO_PORT 0
#endif // !CONFIG_UART_STDIO_POR

#ifndef CONFIG_UART_STDIO_BAUDRATE
#define CONFIG_UART_STDIO_BAUDRATE 2000000
#endif // !CONFIG_UART_STDIO_BAUDRATE

#ifndef CONFIG_UART_STDIO_TX_PIN
#define CONFIG_UART_STDIO_TX_PIN 17
#endif // !CONFIG_UART_STDIO_TX_PIN

#ifndef CONFIG_UART_STDIO_RX_PIN
#define CONFIG_UART_STDIO_RX_PIN 16
#endif // !CONFIG_UART_STDIO_RX_PIN

typedef struct execption_ctxt {
    uint32_t exp_vect;
    uint32_t exp_addr;
    uint32_t spsr;
    uint32_t cpsr;
    uint32_t r[16];
} exception_ctxt_t;

#if (CONFIG_UART_STDIO_BAUDRATE == 9600)
HOSAL_UART_DEV_DECL(uartstdio, CONFIG_UART_STDIO_PORT, CONFIG_UART_STDIO_TX_PIN,
                    CONFIG_UART_STDIO_RX_PIN, UART_BAUDRATE_9600)
#elif (CONFIG_UART_STDIO_BAUDRATE == 115200)
HOSAL_UART_DEV_DECL(uartstdio, CONFIG_UART_STDIO_PORT, CONFIG_UART_STDIO_TX_PIN,
                    CONFIG_UART_STDIO_RX_PIN, UART_BAUDRATE_115200)
#elif (CONFIG_UART_STDIO_BAUDRATE == 500000)
HOSAL_UART_DEV_DECL(uartstdio, CONFIG_UART_STDIO_PORT, CONFIG_UART_STDIO_TX_PIN,
                    CONFIG_UART_STDIO_RX_PIN, UART_BAUDRATE_500000)
#elif (CONFIG_UART_STDIO_BAUDRATE == 1000000)
HOSAL_UART_DEV_DECL(uartstdio, CONFIG_UART_STDIO_PORT, CONFIG_UART_STDIO_TX_PIN,
                    CONFIG_UART_STDIO_RX_PIN, UART_BAUDRATE_1000000)
#elif (CONFIG_UART_STDIO_BAUDRATE == 2000000)
HOSAL_UART_DEV_DECL(uartstdio, CONFIG_UART_STDIO_PORT, CONFIG_UART_STDIO_TX_PIN,
                    CONFIG_UART_STDIO_RX_PIN, UART_BAUDRATE_2000000)
#endif // 0

#define STDIO_UART_BUFF_SIZE 128

typedef struct uart_io {
    uint16_t start;
    uint16_t end;
    uint32_t recvLen;
    uint8_t uart_cache[STDIO_UART_BUFF_SIZE];
} uart_io_t;

static uart_io_t g_uart_rx_io = {
    .start = 0,
    .end = 0,
};
static char bsp_c_g_msg[64] __attribute__((aligned(4))) = {0};
exception_ctxt_t __exi_ctxt;

static int __uartstdio_rx_callback(void* p_arg) {
    uint32_t len = 0;
    if (g_uart_rx_io.start >= g_uart_rx_io.end) {
        g_uart_rx_io.start += hosal_uart_receive(
            p_arg, g_uart_rx_io.uart_cache + g_uart_rx_io.start,
            STDIO_UART_BUFF_SIZE - g_uart_rx_io.start - 1);
        if (g_uart_rx_io.start == (STDIO_UART_BUFF_SIZE - 1)) {
            g_uart_rx_io.start = hosal_uart_receive(
                p_arg, g_uart_rx_io.uart_cache,
                (STDIO_UART_BUFF_SIZE + g_uart_rx_io.end - 1)
                    % STDIO_UART_BUFF_SIZE);
        }
    } else if (((g_uart_rx_io.start + 1) % STDIO_UART_BUFF_SIZE)
               != g_uart_rx_io.end) {
        g_uart_rx_io.start += hosal_uart_receive(
            p_arg, g_uart_rx_io.uart_cache,
            g_uart_rx_io.end - g_uart_rx_io.start - 1);
    }

    if (g_uart_rx_io.start != g_uart_rx_io.end) {

        len = (g_uart_rx_io.start + STDIO_UART_BUFF_SIZE - g_uart_rx_io.end)
              % STDIO_UART_BUFF_SIZE;
        if (g_uart_rx_io.recvLen != len)
            g_uart_rx_io.recvLen = len;
    }
    return 0;
}

static int __uartstdio_line_status_callback(void* p_arg) {

    return hosal_uart_get_lsr((hosal_uart_dev_t*)p_arg);
}

int uart_stdio_write(uint8_t* p_data, uint32_t length) {

    return hosal_uart_send(&uartstdio, p_data, length);
}

int uart_stdio_read(uint8_t* p_data, uint32_t length) {
    uint32_t byte_cnt = 0;
    enter_critical_section();

    if (g_uart_rx_io.start != g_uart_rx_io.end) {
        if (g_uart_rx_io.start > g_uart_rx_io.end) {
            memcpy(p_data, g_uart_rx_io.uart_cache + g_uart_rx_io.end,
                   g_uart_rx_io.start - g_uart_rx_io.end);
            g_uart_rx_io.end = g_uart_rx_io.start;
        } else {
            memcpy(p_data, g_uart_rx_io.uart_cache + g_uart_rx_io.end,
                   STDIO_UART_BUFF_SIZE - g_uart_rx_io.end);
            g_uart_rx_io.end = STDIO_UART_BUFF_SIZE - 1;
            if (g_uart_rx_io.start) {
                memcpy(p_data, g_uart_rx_io.uart_cache, g_uart_rx_io.start);
                g_uart_rx_io.end = (STDIO_UART_BUFF_SIZE + g_uart_rx_io.start
                                    - 1)
                                   % STDIO_UART_BUFF_SIZE;
            }
        }
    }

    byte_cnt = g_uart_rx_io.recvLen;

    g_uart_rx_io.start = g_uart_rx_io.end = 0;
    g_uart_rx_io.recvLen = 0;
    leave_critical_section();
    return byte_cnt;
}

void uart_stdio_en_int_mode(void) {
    hosal_uart_ioctl(&uartstdio, HOSAL_UART_MODE_SET,
                     (void*)HOSAL_UART_MODE_INT_RX);
}

int uart_stdio_deinit(void) {
    hosal_uart_send_complete(&uartstdio);
    hosal_uart_finalize(&uartstdio);
    return 0;
}

int uart_stdio_init(void) {
    /*Init UART In the first place*/
    hosal_uart_init(&uartstdio);

    /* Configure UART Rx interrupt callback function */
    hosal_uart_callback_set(&uartstdio, HOSAL_UART_RX_CALLBACK,
                            __uartstdio_rx_callback, &uartstdio);

    /* Configure UART break interrupt callback function */
    hosal_uart_callback_set(&uartstdio, HOSAL_UART_RECEIVE_LINE_STATUS_CALLBACK,
                            __uartstdio_line_status_callback, &uartstdio);

    /* Configure UART to interrupt mode */
    hosal_uart_ioctl(&uartstdio, HOSAL_UART_MODE_SET,
                     (void*)HOSAL_UART_MODE_INT_RX);

    return 0;
}

static void _uint2strdec(char *pStr, unsigned int number)
{
    char tmp[11];
    int i = 0;

    if (number == 0) {
        pStr[strlen(pStr)] = '0';
        return;
    }

    while (number > 0 && i < (int)sizeof(tmp)) {
        tmp[i++] = (char)('0' + (number % 10));
        number /= 10;
    }

    while (i > 0) {
        pStr[strlen(pStr)] = tmp[--i];
    }
}

static void _uint2strhex(char* pStr, unsigned int number,
                         const char nibbles_to_print) {
#define MAX_NIBBLES (8)
    int nibble = 0;
    char nibbles = (nibbles_to_print > MAX_NIBBLES) ? nibbles_to_print
                                                    : MAX_NIBBLES;

    while (nibbles > 0) {
        nibbles--;
        nibble = (int)(number >> (nibbles * 4)) & 0x0F;
        pStr[strlen(pStr)] = (nibble <= 9) ? (char)('0' + nibble)
                                           : (char)('A' - 10 + nibble);
    }
    return;
}


#define FAULT_STACK_DUMP_WORDS 4   /* 16 bytes = 4 words */

#define FAULT_TYPE_NMI       0U
#define FAULT_TYPE_HARD      1U
#define FAULT_TYPE_MEMMANAGE 2U
#define FAULT_TYPE_BUS       3U
#define FAULT_TYPE_USAGE     4U
#define FAULT_TYPE_SECURE    5U
#define FAULT_TYPE_DEBUGMON  6U

#ifndef SCB_CFSR_MLSPERR_Msk
#define SCB_CFSR_MLSPERR_Msk (1UL << 5)
#endif

typedef struct fault_snapshot {
    uint32_t fault_type;
    uint32_t exc_return;
    uint32_t msp;
    uint32_t psp;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t shcsr;
    uint32_t icsr;
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    uint32_t sfsr;
    uint32_t sfar;
#endif
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_psr;
    uint32_t frame_valid;
    uint32_t msp_dump[FAULT_STACK_DUMP_WORDS];
    uint32_t psp_dump[FAULT_STACK_DUMP_WORDS];
    uint32_t msp_dump_valid;
    uint32_t psp_dump_valid;
} fault_snapshot_t;

static fault_snapshot_t g_fault_snapshot;

static int _exp_dump_out(char* pMsg, int len) {

#if defined(CONFIG_RT584H) || defined(CONFIG_RT584HA4)                         \
    || defined(CONFIG_RT584L) || defined(CONFIG_RF1301)                        \
    || defined(CONFIG_RT584H_NONE_OS) || defined(CONFIG_RT584L_NONE_OS)        \
    || defined(CONFIG_RF1301_NONE_OS) || defined(CONFIG_RT584HA4_NONE_OS)
    uart_t* table[] = {UART0, UART1, UART2};
    uart_t* pCSR = table[CONFIG_UART_STDIO_PORT];
    while (len) {

        while ((pCSR->lsr & 0x20) == 0) {}
        pCSR->thr = *pMsg++;
#else
    uart_t* table[] = {UART0, UART1, UART2};
    uart_t* pCSR = table[CONFIG_UART_STDIO_PORT];
    while (len) {

        while ((pCSR->LSR & 0x20) == 0) {}
        pCSR->THR = *pMsg++;
#endif
        len--;
    }

    return 0;
}

static void _exp_log_out(const char* format, ...) {
    char* pch = (char*)format;
    va_list va;
    va_start(va, format);

    if (!pch) {
        va_end(va);
        return;
    }

    while (*pch) {
        /* format identification character */
        if (*pch != '%') {
            _exp_dump_out(pch, 1);
            pch++;
            continue;
        }

        pch++;
        if (!pch) {
            pch++;
            continue;
        }

        switch (*pch) {
            case 'x': {
                const unsigned int number = va_arg(va, unsigned int);
                for (int j = 0; j < sizeof(bsp_c_g_msg); j += 4)
                    *(uint32_t*)&bsp_c_g_msg[j] = 0;
                strcpy(&bsp_c_g_msg[strlen(bsp_c_g_msg)], "0x");
                _uint2strhex(bsp_c_g_msg, number, 8);
                _exp_dump_out(bsp_c_g_msg, strlen(bsp_c_g_msg));
            } break;
            case 's': {
                char* string = va_arg(va, char*);
                _exp_dump_out(string, strlen(string));
            } break;
            case 'd': {
                const unsigned int number = va_arg(va, unsigned int);
                for (int j = 0; j < sizeof(bsp_c_g_msg); j += 4)
                    *(uint32_t*)&bsp_c_g_msg[j] = 0;

                _uint2strdec(bsp_c_g_msg, number);
                _exp_dump_out(bsp_c_g_msg, strlen(bsp_c_g_msg));
            } break;
            default: break;
        }
        pch++;
    }

    va_end(va);
    return;
}

static int _exp_dump_init(void) {
    uart_stdio_init();
    return 0;
}

static uint32_t _fault_has_bad_stack_frame(uint32_t cfsr)
{
    uint32_t bad = 0;

    bad |= SCB_CFSR_MSTKERR_Msk;
    bad |= SCB_CFSR_MUNSTKERR_Msk;
    bad |= SCB_CFSR_MLSPERR_Msk;
    bad |= SCB_CFSR_STKERR_Msk;
    bad |= SCB_CFSR_UNSTKERR_Msk;

#if defined(CONFIG_RT584H) || defined(CONFIG_RT584HA4) || \
    defined(CONFIG_RT584L) || defined(CONFIG_RF1301) || \
    defined(CONFIG_RT584H_NONE_OS) || defined(CONFIG_RT584L_NONE_OS) || \
    defined(CONFIG_RF1301_NONE_OS) || defined(CONFIG_RT584HA4_NONE_OS)
#ifdef SCB_CFSR_LSPERR_Msk
    bad |= SCB_CFSR_LSPERR_Msk;
#endif
#ifdef SCB_CFSR_STKOF_Msk
    bad |= SCB_CFSR_STKOF_Msk;
#endif
#endif

    return ((cfsr & bad) != 0U) ? 1U : 0U;
}

static const char* _fault_name(uint32_t fault_type)
{
    switch (fault_type) {
        case FAULT_TYPE_NMI:       return "NMI";
        case FAULT_TYPE_HARD:      return "HardFault";
        case FAULT_TYPE_MEMMANAGE: return "MemManage";
        case FAULT_TYPE_BUS:       return "BusFault";
        case FAULT_TYPE_USAGE:     return "UsageFault";
        case FAULT_TYPE_SECURE:    return "SecureFault";
        case FAULT_TYPE_DEBUGMON:  return "DebugMon";
        default:                   return "UnknownFault";
    }
}

static void _fault_capture_stack_words(uint32_t sp,
                                       uint32_t *out,
                                       uint32_t *valid) {
    *valid = 0U;

    if ((sp & 0x3U) != 0U) {
        return;
    }

    for (uint32_t i = 0; i < FAULT_STACK_DUMP_WORDS; i++) {
        uint32_t addr = sp - ((i) * 4U);

        out[i] = *(volatile uint32_t *)addr;
    }

    *valid = 1U;
}

static void _fault_dump_stack_bytes(const char *name,
                                      uint32_t sp,
                                      const uint32_t *data,
                                      uint32_t valid)
{
    _exp_log_out("--- %s %dB Dump ---\r\n", (char *)name, (4*FAULT_STACK_DUMP_WORDS));
    _exp_log_out("%s = %x\r\n", (char *)name, sp);

    if (valid == 0U) {
        _exp_log_out("invalid stack address\r\n");
        return;
    }

    _exp_log_out("[%x] = %x\r\n", sp - 0x00U, data[0]);
    _exp_log_out("[%x] = %x\r\n", sp - 0x04U, data[1]);
    _exp_log_out("[%x] = %x\r\n", sp - 0x08U, data[2]);
    _exp_log_out("[%x] = %x\r\n", sp - 0x0CU, data[3]);
}

static void _fault_dump_basic_info(const fault_snapshot_t* s)
{
    _exp_log_out("\r\n========================================\r\n");
    _exp_log_out("[FAULT] %s\r\n", (char*)_fault_name(s->fault_type));
    _exp_log_out("========================================\r\n");

    _exp_log_out("EXC_RETURN = %x\r\n", s->exc_return);
    _exp_log_out("MSP        = %x\r\n", s->msp);
    _exp_log_out("PSP        = %x\r\n", s->psp);
    _exp_log_out("ICSR       = %x\r\n", s->icsr);
    _exp_log_out("SHCSR      = %x\r\n", s->shcsr);
}

static void _fault_dump_frame(const fault_snapshot_t* s)
{
    _exp_log_out("--- Stacked Registers ---\r\n");

    if (s->frame_valid == 0U) {
        _exp_log_out("stack frame not trusted\r\n");
        return;
    }

    _exp_log_out("R0  = %x\r\n", s->stacked_r0);
    _exp_log_out("R1  = %x\r\n", s->stacked_r1);
    _exp_log_out("R2  = %x\r\n", s->stacked_r2);
    _exp_log_out("R3  = %x\r\n", s->stacked_r3);
    _exp_log_out("R12 = %x\r\n", s->stacked_r12);
    _exp_log_out("LR  = %x\r\n", s->stacked_lr);
    _exp_log_out("PC  = %x\r\n", s->stacked_pc);
    _exp_log_out("PSR = %x\r\n", s->stacked_psr);
}

static void _fault_dump_status(const fault_snapshot_t* s)
{
    _exp_log_out("--- Fault Status Registers ---\r\n");

    _exp_log_out("HFSR = %x\r\n", s->hfsr);
    if (s->hfsr & SCB_HFSR_FORCED_Msk) {
        _exp_log_out("  HFSR: FORCED\r\n");
    }
    if (s->hfsr & SCB_HFSR_VECTTBL_Msk) {
        _exp_log_out("  HFSR: VECTTBL\r\n");
    }
    if (s->hfsr & SCB_HFSR_DEBUGEVT_Msk) {
        _exp_log_out("  HFSR: DEBUGEVT\r\n");
    }

    _exp_log_out("CFSR = %x\r\n", s->cfsr);

    if (s->cfsr & SCB_CFSR_MMARVALID_Msk) {
        _exp_log_out("  MMFSR: MMARVALID MMFAR = %x\r\n", s->mmfar);
    }
    if (s->cfsr & SCB_CFSR_MLSPERR_Msk) {
        _exp_log_out("  MMFSR: MLSPERR\r\n");
    }
    if (s->cfsr & SCB_CFSR_MSTKERR_Msk) {
        _exp_log_out("  MMFSR: MSTKERR\r\n");
    }
    if (s->cfsr & SCB_CFSR_MUNSTKERR_Msk) {
        _exp_log_out("  MMFSR: MUNSTKERR\r\n");
    }
    if (s->cfsr & SCB_CFSR_DACCVIOL_Msk) {
        _exp_log_out("  MMFSR: DACCVIOL\r\n");
    }
    if (s->cfsr & SCB_CFSR_IACCVIOL_Msk) {
        _exp_log_out("  MMFSR: IACCVIOL\r\n");
    }

    if (s->cfsr & SCB_CFSR_BFARVALID_Msk) {
        _exp_log_out("  BFSR: BFARVALID BFAR = %x\r\n", s->bfar);
    }
#ifdef SCB_CFSR_LSPERR_Msk
    if (s->cfsr & SCB_CFSR_LSPERR_Msk) {
        _exp_log_out("  BFSR: LSPERR\r\n");
    }
#endif
    if (s->cfsr & SCB_CFSR_STKERR_Msk) {
        _exp_log_out("  BFSR: STKERR\r\n");
    }
    if (s->cfsr & SCB_CFSR_UNSTKERR_Msk) {
        _exp_log_out("  BFSR: UNSTKERR\r\n");
    }
    if (s->cfsr & SCB_CFSR_IMPRECISERR_Msk) {
        _exp_log_out("  BFSR: IMPRECISERR\r\n");
    }
    if (s->cfsr & SCB_CFSR_PRECISERR_Msk) {
        _exp_log_out("  BFSR: PRECISERR\r\n");
    }
    if (s->cfsr & SCB_CFSR_IBUSERR_Msk) {
        _exp_log_out("  BFSR: IBUSERR\r\n");
    }

    if (s->cfsr & SCB_CFSR_DIVBYZERO_Msk) {
        _exp_log_out("  UFSR: DIVBYZERO\r\n");
    }
    if (s->cfsr & SCB_CFSR_UNALIGNED_Msk) {
        _exp_log_out("  UFSR: UNALIGNED\r\n");
    }
#ifdef SCB_CFSR_STKOF_Msk
    if (s->cfsr & SCB_CFSR_STKOF_Msk) {
        _exp_log_out("  UFSR: STKOF\r\n");
    }
#endif
    if (s->cfsr & SCB_CFSR_NOCP_Msk) {
        _exp_log_out("  UFSR: NOCP\r\n");
    }
    if (s->cfsr & SCB_CFSR_INVPC_Msk) {
        _exp_log_out("  UFSR: INVPC\r\n");
    }
    if (s->cfsr & SCB_CFSR_INVSTATE_Msk) {
        _exp_log_out("  UFSR: INVSTATE\r\n");
    }
    if (s->cfsr & SCB_CFSR_UNDEFINSTR_Msk) {
        _exp_log_out("  UFSR: UNDEFINSTR\r\n");
    }

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    if (s->fault_type == FAULT_TYPE_SECURE) {
        _exp_log_out("SFSR = %x\r\n", s->sfsr);
        if (s->sfsr & SAU_SFSR_SFARVALID_Msk) {
            _exp_log_out("  SFSR: SFARVALID SFAR = %x\r\n", s->sfar);
        }
        if (s->sfsr & SAU_SFSR_LSPERR_Msk) {
            _exp_log_out("  SFSR: LSPERR\r\n");
        }
        if (s->sfsr & SAU_SFSR_INVTRAN_Msk) {
            _exp_log_out("  SFSR: INVTRAN\r\n");
        }
        if (s->sfsr & SAU_SFSR_AUVIOL_Msk) {
            _exp_log_out("  SFSR: AUVIOL\r\n");
        }
        if (s->sfsr & SAU_SFSR_INVER_Msk) {
            _exp_log_out("  SFSR: INVER\r\n");
        }
        if (s->sfsr & SAU_SFSR_INVIS_Msk) {
            _exp_log_out("  SFSR: INVIS\r\n");
        }
        if (s->sfsr & SAU_SFSR_INVEP_Msk) {
            _exp_log_out("  SFSR: INVEP\r\n");
        }
    }
#endif
}

__attribute__((used))
void my_fault_handler_c(uint32_t* sp, uint32_t lr_value, uint32_t fault_type)
{
    fault_snapshot_t* s = &g_fault_snapshot;

    __disable_irq();

    _exp_dump_init();

    s->fault_type = fault_type;
    s->exc_return = lr_value;
    s->msp = __get_MSP();
    s->psp = __get_PSP();
    _fault_capture_stack_words(s->msp, s->msp_dump, &s->msp_dump_valid);
    _fault_capture_stack_words(s->psp, s->psp_dump, &s->psp_dump_valid);
    s->cfsr = SCB->CFSR;
    s->hfsr = SCB->HFSR;
    s->mmfar = SCB->MMFAR;
    s->bfar = SCB->BFAR;
    s->shcsr = SCB->SHCSR;
    s->icsr = SCB->ICSR;
    s->frame_valid = 0U;

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    s->sfsr = SAU->SFSR;
    s->sfar = SAU->SFAR;
#endif

    if ((_fault_has_bad_stack_frame(s->cfsr) == 0U) && (sp != NULL)) {
        s->stacked_r0  = sp[0];
        s->stacked_r1  = sp[1];
        s->stacked_r2  = sp[2];
        s->stacked_r3  = sp[3];
        s->stacked_r12 = sp[4];
        s->stacked_lr  = sp[5];
        s->stacked_pc  = sp[6];
        s->stacked_psr = sp[7];
        s->frame_valid = 1U;
    }

    NVIC_DisableIRQ(Wdt_IRQn);

    _fault_dump_basic_info(s);
    _fault_dump_stack_bytes("MSP", s->msp, s->msp_dump, s->msp_dump_valid);
    _fault_dump_stack_bytes("PSP", s->psp, s->psp_dump, s->psp_dump_valid);   
    _fault_dump_frame(s);
    _fault_dump_status(s);
    _exp_log_out("========================================\r\n");

    while (1) {
        __NOP();
    }
}

/* =========================================================
 * Naked assembly stubs
 * r0 = stack pointer used by faulted context
 * r1 = EXC_RETURN (LR)
 * r2 = fault type
 * ========================================================= */
#define DEFINE_FAULT_HANDLER(name, type_num)                              \
    __attribute__((naked, noreturn)) void name(void) {                    \
        __asm volatile(                                                   \
            "mov   r2, lr         \n"                                     \
            "tst   r2, #4         \n"                                     \
            "ite   eq             \n"                                     \
            "mrseq r0, msp        \n"                                     \
            "mrsne r0, psp        \n"                                     \
            "mov   r1, r2         \n"                                     \
            "movs  r2, #" #type_num " \n"                                 \
            "b     my_fault_handler_c \n"                                 \
        );                                                                \
    }

/* For exception types that do not necessarily have valid EXC_RETURN stack frame,
 * still route through the same handler for unified dump.
 */
#define DEFINE_SIMPLE_FAULT_HANDLER(name, type_num)                       \
    __attribute__((naked, noreturn)) void name(void) {                    \
        __asm volatile(                                                   \
            "mov   r1, lr         \n"                                     \
            "mrs   r0, msp        \n"                                     \
            "movs  r2, #" #type_num " \n"                                 \
            "b     my_fault_handler_c \n"                                 \
        );                                                                \
    }

DEFINE_SIMPLE_FAULT_HANDLER(NMI_Handler,      0)
DEFINE_FAULT_HANDLER(HardFault_Handler,       1)
DEFINE_FAULT_HANDLER(MemManage_Handler,       2)
DEFINE_FAULT_HANDLER(BusFault_Handler,        3)
DEFINE_FAULT_HANDLER(UsageFault_Handler,      4)

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
DEFINE_FAULT_HANDLER(SecureFault_Handler,     5)
#endif

DEFINE_SIMPLE_FAULT_HANDLER(DebugMon_Handler, 6)