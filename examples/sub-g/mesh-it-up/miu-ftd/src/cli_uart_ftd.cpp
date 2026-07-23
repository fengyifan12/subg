/*
 * FTD 专属 CLI UART 实现。
 *
 * 基于 miu-common/src/cli_uart.cpp，唯一改动在 ProcessCommand()：
 *   - 若输入行以 '{' 开头 → 转发给 PC JSON 桥接（app_uart_pc_process_line）
 *   - 否则              → 正常走 OT CLI（otCliInputLine）
 *
 * 这样 CLI 与 PC JSON 命令可以在同一条 UART0 上共存，无需切换模式。
 *
 * 注意：miu-ftd/CMakeLists.txt 中须用本文件替换
 *       ${MIU_COMMON_DIR}/src/cli_uart.cpp。
 */

#include "openthread-core-config.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <openthread/cli.h>
#include <openthread/logging.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "uart.h"

extern "C" {
#include "app_uart_pc.h"
}

#ifndef OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE
#if OPENTHREAD_CONFIG_REFERENCE_DEVICE_ENABLE
#define OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE 640
#else
#define OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE 512
#endif
#endif

#ifndef OPENTHREAD_CONFIG_CLI_UART_TX_BUFFER_SIZE
#define OPENTHREAD_CONFIG_CLI_UART_TX_BUFFER_SIZE 1024
#endif

#if OPENTHREAD_CONFIG_DIAG_ENABLE
#if OPENTHREAD_CONFIG_DIAG_OUTPUT_BUFFER_SIZE > OPENTHREAD_CONFIG_CLI_UART_TX_BUFFER_SIZE
#error "diag output buffer should be smaller than CLI UART tx buffer"
#endif
#if OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE > OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE
#error "diag command line should be smaller than CLI UART rx buffer"
#endif
#endif

#if OPENTHREAD_CONFIG_CLI_MAX_LINE_LENGTH > OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE
#error "command line should be should be smaller than CLI rx buffer"
#endif

enum {
    kRxBufferSize = OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE,
    kTxBufferSize = OPENTHREAD_CONFIG_CLI_UART_TX_BUFFER_SIZE,
};

char sRxBuffer[kRxBufferSize];
uint16_t sRxLength;
char sLefcount;

char sTxBuffer[kTxBufferSize];
uint16_t sTxHead;
uint16_t sTxLength;

uint16_t sSendLength;

#ifndef OT_CLI_UART_OUTPUT_LOCK
#define OT_CLI_UART_OUTPUT_LOCK() do { } while (0)
#endif

#ifndef OT_CLI_UART_OUTPUT_UNLOCK
#define OT_CLI_UART_OUTPUT_UNLOCK() do { } while (0)
#endif

static int Output(const char* aBuf, uint16_t aBufLength);
static otError ProcessCommand(void);

static const uint16_t History_Deep = 16;
static int History_Index = 0;
static char History_Buf[History_Deep][OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE];

static void Push_To_History(char* buffer, uint16_t buffer_size) {
    uint16_t i = 0;
    char c;

    do {
        if (buffer_size == 0)
            break;
        for (i = 0; i < buffer_size; i++) {
            c = buffer[i];
            if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z')
                || (c >= 'A' && c <= 'Z') || c == '_' || c == '?' || c == '-'
                || c == '=' || c == ':' || c == 0x20) {
            } else {
                break;
            }
        }
        char* last_history_buffer = History_Buf[History_Deep - 1];
        (void)last_history_buffer;

        for (i = (History_Deep - 1); i > 0; i--) {
            memset(History_Buf[i], 0x0, OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE);
            memcpy(History_Buf[i], History_Buf[i - 1], OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE);
        }

        memset(History_Buf[0], 0x0, OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE);
        memcpy(History_Buf[0], buffer, buffer_size);

        History_Index = 0;

    } while (0);
}

static char* Pop_History(int* index) {
    uint16_t i = 0;
    char* buffer = NULL;
    do {
        for (i = 0; i < History_Deep; i++) {
            if (strlen(History_Buf[i]) == 0) {
                break;
            }
        }

        if (*index > i) {
            *index = 0;
        } else if (*index < 0) {
            *index = i;
        }
        buffer = History_Buf[*index];
    } while (0);

    return buffer;
}

static void ReceiveTask(const uint8_t* aBuf, uint16_t aBufLength) {
    static const char sEraseString[] = {'\b', ' ', '\b'};
    static const char sEscRighString[] = "\x1b[C";
    static const char sEscLeftString[] = "\x1b[D";
    static const char CRNL[] = {'\r', '\n'};
    static uint8_t sLastChar = '\0';
    static uint8_t move = 0;

    const uint8_t* end = aBuf + aBufLength;
    char* pop_buffer = NULL;
    uint16_t i = 0;

    for (; aBuf < end; aBuf++) {
        uint8_t ch = *aBuf;

        switch (ch) {
            case '\n':
                if (sLastChar == '\r')
                    break;
                OT_FALL_THROUGH;

            case '\r':
                Output(CRNL, sizeof(CRNL));
                sRxBuffer[sRxLength] = '\0';
                sLefcount = 0;
                move = 0;
                Push_To_History(sRxBuffer, sRxLength);
                IgnoreError(ProcessCommand());
                break;

            case '\b':
            case 127:
                move = 0;
                if (sRxLength > 0) {
                    if (sLefcount > 0 && sLefcount < sRxLength) {
                        Output(sEraseString, sizeof(sEraseString));
                        memmove(&sRxBuffer[sRxLength - sLefcount - 1],
                                &sRxBuffer[sRxLength - sLefcount], sLefcount);
                        sRxBuffer[--sRxLength] = '\0';
                        Output(&sRxBuffer[sRxLength - sLefcount], sLefcount);
                        Output(sEscRighString, strlen(sEscRighString));
                        Output(sEraseString, sizeof(sEraseString));
                        for (i = 0; i < sLefcount; i++) {
                            Output(sEscLeftString, strlen(sEscLeftString));
                        }
                    } else if (sLefcount == 0) {
                        Output(sEraseString, sizeof(sEraseString));
                        sRxBuffer[--sRxLength] = '\0';
                    }
                }
                break;

            case 0x1b:
                move = 1;
                break;

            case 0x5b: // '['
                move = (move == 1) ? 2 : 0;
                break;

            case 65: // Up
            case 66: // Down
                if (move == 2) {
                    while (sLefcount > 0) {
                        Output(sEscRighString, strlen(sEscRighString));
                        sLefcount--;
                    }
                    while (sRxLength > 0) {
                        Output(sEraseString, sizeof(sEraseString));
                        sRxBuffer[--sRxLength] = '\0';
                    }
                    sLefcount = 0;
                    pop_buffer = Pop_History(&History_Index);
                    if (pop_buffer != NULL) {
                        size_t len = strlen(pop_buffer);
                        if (len < kRxBufferSize) {
                            sRxLength = (uint16_t)len;
                            memcpy(sRxBuffer, pop_buffer, sRxLength);
                            sRxBuffer[sRxLength] = '\0';
                            Output(sRxBuffer, sRxLength);
                        }
                    }
                    if (ch == 65) {
                        History_Index++;
                    } else {
                        History_Index--;
                    }
                }
                OT_FALL_THROUGH;

            case 67: // Right
            case 68: // Left
                if (move == 2) {
                    if (ch == 67 && sLefcount > 0) {
                        Output(sEscRighString, strlen(sEscRighString));
                        sLefcount--;
                    } else if (ch == 68 && sLefcount < sRxLength) {
                        Output(sEscLeftString, strlen(sEscLeftString));
                        sLefcount++;
                    }
                    move = 0;
                    break;
                }
                OT_FALL_THROUGH;

            default:
                if (move != 0)
                    move = 0;

                if (sRxLength < kRxBufferSize - 1) {
                    if (sLefcount > 0) {
                        memmove(&sRxBuffer[sRxLength - sLefcount + 1],
                                &sRxBuffer[sRxLength - sLefcount], sLefcount);
                        sRxBuffer[sRxLength - sLefcount] = (char)ch;
                        Output(&sRxBuffer[sRxLength - sLefcount], (sLefcount + 1));
                        sRxLength++;
                        for (i = 0; i < sLefcount; i++) {
                            Output(sEscLeftString, strlen(sEscLeftString));
                        }
                    } else {
                        char current_ch = (char)ch;
                        Output(&current_ch, 1);
                        sRxBuffer[sRxLength++] = current_ch;
                    }
                }
                break;
        }
        sLastChar = ch;
    }
}

static otError ProcessCommand(void) {
    otError error = OT_ERROR_NONE;

    while (sRxLength > 0
           && (sRxBuffer[sRxLength - 1] == '\n'
               || sRxBuffer[sRxLength - 1] == '\r')) {
        sRxBuffer[--sRxLength] = '\0';
    }

    if (sRxLength > 0 && sRxBuffer[0] == '{') {
        /* JSON 命令 → PC 桥接处理，不走 OT CLI */
        app_uart_pc_process_line(sRxBuffer);
    } else {
        otCliInputLine(sRxBuffer);
    }
    sRxLength = 0;

    return error;
}

static void Send(void) {
    VerifyOrExit(sSendLength == 0);

    if (sTxLength > kTxBufferSize - sTxHead) {
        sSendLength = kTxBufferSize - sTxHead;
    } else {
        sSendLength = sTxLength;
    }

    if (sSendLength > 0) {
        IgnoreError(otPlatUartSend(
            reinterpret_cast<uint8_t*>(sTxBuffer + sTxHead), sSendLength));
    }

exit:
    return;
}

static void SendDoneTask(void) {
    sTxHead = (sTxHead + sSendLength) % kTxBufferSize;
    sTxLength -= sSendLength;
    sSendLength = 0;

    Send();
}

static int Output(const char* aBuf, uint16_t aBufLength) {
    OT_CLI_UART_OUTPUT_LOCK();
    uint16_t sent = 0;

    while (aBufLength > 0) {
        uint16_t remaining = kTxBufferSize - sTxLength;
        uint16_t tail;
        uint16_t sendLength = aBufLength;

        if (sendLength > remaining) {
            sendLength = remaining;
        }

        for (uint16_t i = 0; i < sendLength; i++) {
            tail = (sTxHead + sTxLength) % kTxBufferSize;
            sTxBuffer[tail] = *aBuf++;
            aBufLength--;
            sTxLength++;
        }

        Send();

        sent += sendLength;

        if (aBufLength > 0) {
            otError err = otPlatUartFlush();
            if (err == OT_ERROR_NONE) {
                SendDoneTask();
            } else {
                break;
            }
        }
    }

    OT_CLI_UART_OUTPUT_UNLOCK();

    return sent;
}

static int CliUartOutput(void* aContext, const char* aFormat, va_list aArguments) {
    OT_UNUSED_VARIABLE(aContext);

    int rval;

    if (sTxLength == 0) {
        rval = vsnprintf(sTxBuffer, kTxBufferSize, aFormat, aArguments);
        VerifyOrExit(rval >= 0 && rval < kTxBufferSize,
                     otLogWarnPlat("Failed to format CLI output `%s`", aFormat));
        sTxHead = 0;
        sTxLength = static_cast<uint16_t>(rval);
        sSendLength = 0;
    } else {
        va_list retryArguments;
        uint16_t tail = (sTxHead + sTxLength) % kTxBufferSize;
        uint16_t remaining = (sTxHead > tail ? (sTxHead - tail)
                                             : (kTxBufferSize - tail));

        va_copy(retryArguments, aArguments);

        rval = vsnprintf(&sTxBuffer[tail], remaining, aFormat, aArguments);

        if (rval < 0) {
            otLogWarnPlat("Failed to format CLI output `%s`", aFormat);
        } else if (rval < remaining) {
            sTxLength += rval;
        } else if (rval < kTxBufferSize) {
            while (sTxLength != 0) {
                otError error;
                Send();
                error = otPlatUartFlush();
                if (error == OT_ERROR_NONE) {
                    SendDoneTask();
                } else {
                    otLogWarnPlat("Failed to output CLI: %s",
                                  otThreadErrorToString(error));
                    break;
                }
            }
            rval = vsnprintf(sTxBuffer, kTxBufferSize, aFormat, retryArguments);
            OT_ASSERT(rval > 0);
            sTxLength = static_cast<uint16_t>(rval);
            sTxHead = 0;
            sSendLength = 0;
        } else {
            otLogWarnPlat("CLI output `%s` truncated", aFormat);
        }

        va_end(retryArguments);
    }

    Send();

exit:
    return rval;
}

void otPlatUartReceived(const uint8_t* aBuf, uint16_t aBufLength) {
    ReceiveTask(aBuf, aBufLength);
}

void otPlatUartSendDone(void) { SendDoneTask(); }

extern "C" void otAppCliInit(otInstance* aInstance) {
    sRxLength = 0;
    sTxHead = 0;
    sTxLength = 0;
    sSendLength = 0;
    sLefcount = 0;

    IgnoreError(otPlatUartEnable());

    otCliInit(aInstance, CliUartOutput, aInstance);
}
