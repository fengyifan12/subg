#include "app_uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "cli.h"
#include "hosal_uart.h"
#include "log.h"
#include "main.h"
#if CONFIG_MESH_IT_UP_FTD
#  if CONFIG_APP_UART_PC_ENABLE
#    include "app_uart_pc.h"
#  else
#    include "ota_download_cmd_handler.h"
#  endif
#endif
#include "task.h"
#include "util_queue.h"

#include <timers.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifndef CONFIG_APP_CLI_UART1_ENABLE
#define CONFIG_APP_CLI_UART1_ENABLE 0
#endif

#ifndef CONFIG_APP_LOG_UART1_ENABLE
#define CONFIG_APP_LOG_UART1_ENABLE 0
#endif

#ifndef CONFIG_APP_UART0_PORT
#define CONFIG_APP_UART0_PORT 0
#endif

#ifndef CONFIG_APP_UART0_TX_PIN
#define CONFIG_APP_UART0_TX_PIN 17
#endif

#ifndef CONFIG_APP_UART0_RX_PIN
#define CONFIG_APP_UART0_RX_PIN 16
#endif

#ifndef CONFIG_APP_UART1_PORT
#define CONFIG_APP_UART1_PORT 1
#endif

#ifndef CONFIG_APP_UART1_TX_PIN
#define CONFIG_APP_UART1_TX_PIN 28
#endif

#ifndef CONFIG_APP_UART1_RX_PIN
#define CONFIG_APP_UART1_RX_PIN 29
#endif

#if CONFIG_APP_CLI_UART1_ENABLE
#include "uart.h"
#endif

#define UART_HANDLER_RX_CACHE_SIZE 128
#define RX_BUFF_SIZE               1024
#define MAX_UART_BUFFER_SIZE       384

static xQueueHandle app_uart_msg_queue;

typedef enum {
    APP_UART0_RECEIVED_EVENT = 0x01,
    APP_UART1_RECEIVED_EVENT = 0x02,
} uart_event_id_t;

typedef struct uart_io {
    uint16_t start;
    uint16_t end;

    uint32_t recvLen;
    uint8_t uart_cache[RX_BUFF_SIZE];
} uart_io_t;

static uart_io_t g_uart0_rx_io = {.start = 0, .end = 0, .recvLen = 0};
static uart_io_t g_uart1_rx_io = {.start = 0, .end = 0, .recvLen = 0};
HOSAL_UART_DEV_DECL(app_uart0_dev, CONFIG_APP_UART0_PORT, CONFIG_APP_UART0_TX_PIN,
                    CONFIG_APP_UART0_RX_PIN, UART_BAUDRATE_Baud115200)
HOSAL_UART_DEV_DECL(uart1_dev, CONFIG_APP_UART1_PORT, CONFIG_APP_UART1_TX_PIN,
                    CONFIG_APP_UART1_RX_PIN, UART_BAUDRATE_Baud115200)

static uint8_t g_uart0_buf[MAX_UART_BUFFER_SIZE] = {0};

static TimerHandle_t app_uart1_rx_cb_time = NULL;
static uint8_t s_uart0_hw_ready = 0;
static uint8_t s_uart1_hw_ready = 0;

static void app_uart_task();

#if CONFIG_APP_LOG_UART1_ENABLE
static uint8_t s_uart1_log_ready = 0;

int app_log_output(uint8_t* p_data, uint32_t data_len) {
    if (p_data == NULL || data_len == 0) {
        return 0;
    }

    if (!s_uart1_hw_ready) {
        hosal_uart_init(&uart1_dev);
        s_uart1_hw_ready = 1;
    }
    s_uart1_log_ready = 1;

    return hosal_uart_send(&uart1_dev, p_data, data_len);
}
#endif

int app_uart0_rx_read(uint8_t* p_data, uint32_t p_data_len) {
    if (p_data == NULL || p_data_len == 0) {
        return -1; // Prevent invalid reads
    }
    uint32_t available_data = 0;

    taskENTER_CRITICAL();
    if (g_uart0_rx_io.start >= g_uart0_rx_io.end) {
        available_data = g_uart0_rx_io.start - g_uart0_rx_io.end;
    } else {
        available_data = RX_BUFF_SIZE - g_uart0_rx_io.end + g_uart0_rx_io.start;
    }
    taskEXIT_CRITICAL();

    if (available_data == 0) {
        return 0; // No readable data
    }

    uint32_t read_len = (p_data_len > available_data) ? available_data
                                                      : p_data_len;

    // Ring buffer reads data
    if (g_uart0_rx_io.end + read_len < RX_BUFF_SIZE) {
        // Read directly from end
        memcpy(p_data, g_uart0_rx_io.uart_cache + g_uart0_rx_io.end, read_len);
        taskENTER_CRITICAL();
        g_uart0_rx_io.end += read_len;
        taskEXIT_CRITICAL();
    } else {
        // Read the tail part first
        uint32_t tail_len = RX_BUFF_SIZE - g_uart0_rx_io.end;
        memcpy(p_data, g_uart0_rx_io.uart_cache + g_uart0_rx_io.end, tail_len);

        // Then read the rest from the beginning
        uint32_t head_len = read_len - tail_len;
        memcpy(p_data + tail_len, g_uart0_rx_io.uart_cache, head_len);
        taskENTER_CRITICAL();
        g_uart0_rx_io.end = head_len; // Update `end` position
        taskEXIT_CRITICAL();
    }

    // Update `recvLen`
    taskENTER_CRITICAL();
    g_uart0_rx_io.recvLen -= read_len;
    taskEXIT_CRITICAL();

    return read_len;
}

static int uart0_rx_callback(void* p_arg) {
    uint32_t new_data_len = 0;

    // Ring buffer writes (make sure not to exceed RX_BUFF_SIZE)
    if (g_uart0_rx_io.start >= g_uart0_rx_io.end) {
        new_data_len = hosal_uart_receive(
            p_arg, g_uart0_rx_io.uart_cache + g_uart0_rx_io.start,
            RX_BUFF_SIZE - g_uart0_rx_io.start);
        g_uart0_rx_io.start = (g_uart0_rx_io.start + new_data_len)
                              % RX_BUFF_SIZE;
    } else if (((g_uart0_rx_io.start + 1) % RX_BUFF_SIZE)
               != g_uart0_rx_io.end) {
        new_data_len = hosal_uart_receive(
            p_arg, g_uart0_rx_io.uart_cache + g_uart0_rx_io.start,
            g_uart0_rx_io.end - g_uart0_rx_io.start - 1);
        g_uart0_rx_io.start = (g_uart0_rx_io.start + new_data_len)
                              % RX_BUFF_SIZE;
    }
    if (new_data_len > 0) {
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        uint8_t app_uart_msg = APP_UART0_RECEIVED_EVENT;
        if (xQueueSendFromISR(app_uart_msg_queue, &app_uart_msg,
                              &pxHigherPriorityTaskWoken)
            == pdPASS) {
            ot_app_task_post(app_uart_task, &app_uart_msg);
        }
    }

    return 0;
}

/*uart 1 use*/
static int __uart1_read(uint8_t* p_data, uint32_t p_data_len) {
    if (p_data == NULL || p_data_len == 0) {
        return -1; // Prevent invalid reads
    }
    uint32_t available_data = 0;

    taskENTER_CRITICAL();
    if (g_uart1_rx_io.start >= g_uart1_rx_io.end) {
        available_data = g_uart1_rx_io.start - g_uart1_rx_io.end;
    } else {
        available_data = RX_BUFF_SIZE - g_uart1_rx_io.end + g_uart1_rx_io.start;
    }
    taskEXIT_CRITICAL();

    if (available_data == 0) {
        return 0; // No readable data
    }

    uint32_t read_len = (p_data_len > available_data) ? available_data
                                                      : p_data_len;

    // Ring buffer reads data
    if (g_uart1_rx_io.end + read_len < RX_BUFF_SIZE) {
        // Read directly from end
        memcpy(p_data, g_uart1_rx_io.uart_cache + g_uart1_rx_io.end, read_len);
        taskENTER_CRITICAL();
        g_uart1_rx_io.end += read_len;
        taskEXIT_CRITICAL();
    } else {
        // Read the tail part first
        uint32_t tail_len = RX_BUFF_SIZE - g_uart1_rx_io.end;
        memcpy(p_data, g_uart1_rx_io.uart_cache + g_uart1_rx_io.end, tail_len);

        // Then read the rest from the beginning
        uint32_t head_len = read_len - tail_len;
        memcpy(p_data + tail_len, g_uart1_rx_io.uart_cache, head_len);
        taskENTER_CRITICAL();
        g_uart1_rx_io.end = head_len; // Update `end` position
        taskEXIT_CRITICAL();
    }

    // Update `recvLen`
    taskENTER_CRITICAL();
    g_uart1_rx_io.recvLen -= read_len;
    taskEXIT_CRITICAL();

    return read_len;
}

static int __uart1_rx_callback(void* p_arg) {
    uint32_t new_data_len = 0;

    // Ring buffer writes (make sure not to exceed RX_BUFF_SIZE)
    if (g_uart1_rx_io.start >= g_uart1_rx_io.end) {
        new_data_len = hosal_uart_receive(
            p_arg, g_uart1_rx_io.uart_cache + g_uart1_rx_io.start,
            RX_BUFF_SIZE - g_uart1_rx_io.start);
        g_uart1_rx_io.start = (g_uart1_rx_io.start + new_data_len)
                              % RX_BUFF_SIZE;
    } else if (((g_uart1_rx_io.start + 1) % RX_BUFF_SIZE)
               != g_uart1_rx_io.end) {
        new_data_len = hosal_uart_receive(
            p_arg, g_uart1_rx_io.uart_cache + g_uart1_rx_io.start,
            g_uart1_rx_io.end - g_uart1_rx_io.start - 1);
        g_uart1_rx_io.start = (g_uart1_rx_io.start + new_data_len)
                              % RX_BUFF_SIZE;
    }
    if (new_data_len > 0) {
        if (app_uart1_rx_cb_time) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTimerResetFromISR(app_uart1_rx_cb_time, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    return 0;
}

static int __uart1_break_callback(void* p_arg) {
    /*can't printf*/
    return 0;
}

void app_uart1_recv() {
    static uint8_t tmp_buf[UART_HANDLER_RX_CACHE_SIZE] = {0};
    int len = 0;

    do {
        len = __uart1_read(tmp_buf, UART_HANDLER_RX_CACHE_SIZE);
        if (len <= 0) {
            break;
        }
#if CONFIG_APP_CLI_UART1_ENABLE
        otPlatUartReceived(tmp_buf, (uint16_t)len);
#else
        log_info_hexdump("uar1 rx", tmp_buf, len);
#endif
    } while (len > 0);
}

void app_uart1_data_recv() { return; }

void app_uart0_hex_recv() {
    static uint16_t total_len = 0;
    static uint16_t offset = 0;

    uint16_t msgbufflen = 0;
    uint32_t parser_status = 0;
    int lens = 0;
    static uint8_t uart0_packet[UART_HANDLER_RX_CACHE_SIZE];

    do {
        if (total_len >= MAX_UART_BUFFER_SIZE) {
            total_len = 0;
        }

        memset(uart0_packet, 0, UART_HANDLER_RX_CACHE_SIZE);
        lens = app_uart0_rx_read(uart0_packet, UART_HANDLER_RX_CACHE_SIZE);
        if (lens > 0) {
            uint32_t space_left = MAX_UART_BUFFER_SIZE - total_len;
            uint32_t data_to_copy = (lens > space_left) ? space_left : lens;

            memcpy(g_uart0_buf + total_len, uart0_packet, data_to_copy);
            total_len += data_to_copy;
#if CONFIG_APP_TASK_OTA_ENABLE && CONFIG_MESH_IT_UP_FTD
            parser_status = ota_download_cmd_parser(g_uart0_buf, total_len,
                                                    &msgbufflen, &offset);
            if (parser_status == UART_DATA_VALID) {
                ota_download_cmd_proc((g_uart0_buf + offset), msgbufflen);
            } else if (parser_status == UART_DATA_CS_ERROR) {
                total_len = 0;
            }
#endif
            if (msgbufflen > 0) {
                if (total_len > msgbufflen) {
                    total_len -= msgbufflen;
                    if (total_len > offset) {
                        memcpy((g_uart0_buf + offset),
                               (g_uart0_buf + msgbufflen + offset),
                               total_len - offset);
                    }
                } else {
                    total_len = 0;
                }
            }

            offset = 0;
            msgbufflen = 0;
        }

    } while (lens > 0);
}

static void app_uart_task() {
    uint8_t app_uart_event = 0xFF;
    if (xQueueReceive(app_uart_msg_queue, &app_uart_event, 0) == pdPASS) {
        if (app_uart_event == APP_UART1_RECEIVED_EVENT) {
            app_uart1_recv();
        }
#if CONFIG_MESH_IT_UP_FTD
        else if (app_uart_event == APP_UART0_RECEIVED_EVENT) {
#if CONFIG_APP_UART_PC_ENABLE
            app_uart_pc_json_recv();   /* PC JSON bridge */
#else
            app_uart0_hex_recv();      /* OTA download */
#endif
        }
#endif
    }
}

int app_uart_data_send(uint8_t u_port, uint8_t* p_data, uint16_t data_len) {
    hosal_uart_send(&uart1_dev, p_data, data_len);
    return 0;
}

int app_uart0_data_send(const uint8_t *p_data, uint16_t data_len) {
    if (p_data == NULL || data_len == 0) {
        return 0;
    }

    if (!s_uart0_hw_ready) {
        hosal_uart_init(&app_uart0_dev);
        s_uart0_hw_ready = 1;
    }

    return hosal_uart_send(&app_uart0_dev, p_data, data_len);
}

void app_uart1_rx_cb_timeout_callback(TimerHandle_t xTimer) {
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    uint8_t app_uart_msg = APP_UART1_RECEIVED_EVENT;
    if (xQueueSendFromISR(app_uart_msg_queue, &app_uart_msg,
                          &pxHigherPriorityTaskWoken)
        == pdPASS) {
        ot_app_task_post(app_uart_task, &app_uart_msg);
    }
}

void app_uart0_enable(void) {
    if (!s_uart0_hw_ready) {
        hosal_uart_init(&app_uart0_dev);
        s_uart0_hw_ready = 1;
    }

    memset(&g_uart0_rx_io, 0, sizeof(g_uart0_rx_io));
    hosal_uart_callback_set(&app_uart0_dev, HOSAL_UART_RX_CALLBACK,
                            uart0_rx_callback, &app_uart0_dev);
    hosal_uart_ioctl(&app_uart0_dev, HOSAL_UART_MODE_SET,
                     (void*)HOSAL_UART_MODE_INT_RX);
    /* Keep UART0 RX responsive for PC JSON bridge / OTA binary mode. */
    __NVIC_SetPriority(Uart0_IRQn, 3);
}

void app_uart0_disable(void) {
    memset(&g_uart0_rx_io, 0, sizeof(g_uart0_rx_io));
    __NVIC_SetPriority(Uart0_IRQn, 4); /* restore default */
}

void app_uart_log_init(void) {
#if CONFIG_APP_LOG_UART1_ENABLE
    if (!s_uart1_hw_ready) {
        hosal_uart_init(&uart1_dev);
        s_uart1_hw_ready = 1;
    }
    s_uart1_log_ready = 1;
#endif
}

void app_uart_init() {
    if (app_uart_msg_queue == NULL) {
        app_uart_msg_queue = xQueueCreate(5, sizeof(uint8_t));
    }

    if (app_uart1_rx_cb_time == NULL) {
        app_uart1_rx_cb_time = xTimerCreate(
            "app_uart1_rx_cb_time", 5, pdFALSE, NULL,
            (TimerCallbackFunction_t)app_uart1_rx_cb_timeout_callback);
    }

    if (!s_uart1_hw_ready) {
        hosal_uart_init(&uart1_dev);
        s_uart1_hw_ready = 1;
    }

    /* Configure UART Rx interrupt callback function */
    hosal_uart_callback_set(&uart1_dev, HOSAL_UART_RX_CALLBACK,
                            __uart1_rx_callback, &uart1_dev);

    /* Configure UART to interrupt mode */
    hosal_uart_ioctl(&uart1_dev, HOSAL_UART_MODE_SET,
                     (void*)HOSAL_UART_MODE_INT_RX);

    __NVIC_SetPriority(Uart1_IRQn, 2);
#if CONFIG_APP_LOG_UART1_ENABLE
    s_uart1_log_ready = 1;
#endif
}

#if CONFIG_APP_CLI_UART1_ENABLE
otError __wrap_otPlatUartEnable(void) { return OT_ERROR_NONE; }

otError __wrap_otPlatUartDisable(void) { return OT_ERROR_NONE; }

otError __wrap_otPlatUartSend(const uint8_t* aBuf, uint16_t aBufLength) {
    if (aBuf == NULL || aBufLength == 0) {
        otPlatUartSendDone();
        return OT_ERROR_NONE;
    }

    hosal_uart_send(&uart1_dev, (uint8_t*)aBuf, aBufLength);
    otPlatUartSendDone();

    return OT_ERROR_NONE;
}

otError __wrap_otPlatUartFlush(void) { return OT_ERROR_NONE; }
#endif
