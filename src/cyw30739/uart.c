/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include <stdio.h>

#include <openthread/instance.h>
#if OPENTHREAD_API_VERSION >= 85
#include <utils/uart.h>
#else
#include <openthread/platform/uart.h>
#endif
#include <wiced_hal_puart.h>

#include <wiced_platform.h>
#include <wiced_rtos.h>

#define UART_RX_BUFFER_SIZE (512)

#ifdef CLI_COMMAND_SEPARATOR

#define UART_RX_CLI_STATE_CHAR_CARRIAGE_RETURN '\r'

typedef enum uart_rx_cli_start_state
{
    UART_RX_CLI_START_STATE_IDLE    = 0,
    UART_RX_CLI_START_STATE_STARTED = 1, // receive '\r'
} uart_rx_cli_start_state_t;

#endif // CLI_COMMAND_SEPARATOR

typedef struct uart_cb
{
    bool     initialized;
    uint32_t event_code;

    struct
    {
        wiced_mutex_t *p_mutex;
        uint16_t       index_start;
        uint16_t       index_end;
        uint16_t       len;
        uint8_t        data[UART_RX_BUFFER_SIZE];

#ifdef CLI_COMMAND_SEPARATOR

        struct
        {
            uart_rx_cli_start_state_t start;
        } cli_state;

#endif // CLI_COMMAND_SEPARATOR
    } rx;
} uart_cb_t;

/* Declaration of static functions. */
static void    uart_rx_data_handler(void);
static uint8_t uart_rx_data_get(void);
static void    uart_rx_data_put(uint8_t data);
static void    uart_rx_interrupt(void *arg);
static void    uart_write(const uint8_t *aBuf, uint16_t aBufLength);

/* Declaration of static variables. */
static uart_cb_t uart_cb = {0};

void otPlatUartInit(void)
{
    /* Get/Register the application thread event code. */
    if (!wiced_platform_application_thread_event_register(&uart_cb.event_code, uart_rx_data_handler))
    {
        printf("%s: Fail to get event code.\n", __FUNCTION__);
        return;
    }

    /* Create rx data mutex. */
    uart_cb.rx.p_mutex = wiced_rtos_create_mutex();

    if (uart_cb.rx.p_mutex == NULL)
    {
        printf("%s: Fail to create mutex.\n", __FUNCTION__);
        return;
    }

    /* Initialize the rx data mutex. */
    if (wiced_rtos_init_mutex(uart_cb.rx.p_mutex) != WICED_SUCCESS)
    {
        printf("%s: Fail to init. mutex.\n", __FUNCTION__);
        return;
    }

    uart_cb.initialized = true;
}

otError otPlatUartEnable(void)
{
    wiced_platform_application_thread_check();

    if (!uart_cb.initialized)
    {
        return OT_ERROR_INVALID_STATE;
    }

    wiced_platform_puart_init(uart_rx_interrupt);

    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    wiced_platform_application_thread_check();
    if (!uart_cb.initialized)
    {
        return OT_ERROR_INVALID_STATE;
    }
    uart_write(aBuf, aBufLength);
    otPlatUartSendDone();

    return OT_ERROR_NONE;
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

//------------------------------ Static Function --------------------------------------------------

static void uart_rx_interrupt(void *arg)
{
    uint8_t data;

    OT_UNUSED_VARIABLE(arg);

    wiced_rtos_lock_mutex(uart_cb.rx.p_mutex);

    while (wiced_hal_puart_read(&data))
    {
        uart_rx_data_put(data);
    }

    wiced_platform_application_thread_event_set(uart_cb.event_code);

    wiced_rtos_unlock_mutex(uart_cb.rx.p_mutex);

    wiced_hal_puart_reset_puart_interrupt();
}

static void uart_rx_data_handler(void)
{
    uint8_t data;

    while (uart_cb.rx.len)
    {
        wiced_rtos_lock_mutex(uart_cb.rx.p_mutex);

        data = uart_rx_data_get();

        wiced_rtos_unlock_mutex(uart_cb.rx.p_mutex);

#ifdef CLI_COMMAND_SEPARATOR
        /* Check cli start state. */
        switch (uart_cb.rx.cli_state.start)
        {
        case UART_RX_CLI_START_STATE_IDLE:
            /* Check the data. */
            if (data == UART_RX_CLI_STATE_CHAR_CARRIAGE_RETURN)
            {
                /* Set cli start state. */
                uart_cb.rx.cli_state.start = UART_RX_CLI_START_STATE_STARTED;

                /* Deliver data to upper layer. */
                otPlatUartReceived(&data, sizeof(data));
            }
            break;

        case UART_RX_CLI_START_STATE_STARTED:
            /* Check the data. */
            if (data == UART_RX_CLI_STATE_CHAR_CARRIAGE_RETURN)
            {
                /* Set cli start state. */
                uart_cb.rx.cli_state.start = UART_RX_CLI_START_STATE_IDLE;
            }

            /* Deliver data to upper layer. */
            otPlatUartReceived(&data, sizeof(data));
            break;

        default:
            printf("Err: CLI in invalid start state\n");
            break;
        }
#else  // CLI_COMMAND_SEPARATOR
        otPlatUartReceived(&data, sizeof(data));
#endif // !CLI_COMMAND_SEPARATOR
    }
}

static void uart_rx_data_put(uint8_t data)
{
    if (uart_cb.rx.len == UART_RX_BUFFER_SIZE)
    {
        printf("%s: Error: Rx data full.\n", __FUNCTION__);
        return;
    }

    uart_cb.rx.data[uart_cb.rx.index_end++] = data;

    uart_cb.rx.len++;
    if (uart_cb.rx.index_end == UART_RX_BUFFER_SIZE)
    {
        uart_cb.rx.index_end = 0;
    }
}

static uint8_t uart_rx_data_get(void)
{
    uint8_t data;

    if (uart_cb.rx.len == 0)
    {
        printf("%s: Error: Rx data empty.\n", __FUNCTION__);
        return 0;
    }

    data = uart_cb.rx.data[uart_cb.rx.index_start++];

    uart_cb.rx.len--;
    if (uart_cb.rx.index_start == UART_RX_BUFFER_SIZE)
    {
        uart_cb.rx.index_start = 0;
    }

    return data;
}

static void uart_write(const uint8_t *aBuf, uint16_t aBufLength)
{
    while (aBufLength > 0)
    {
        wiced_hal_puart_write((UINT8)*aBuf);
        aBuf++;
        aBufLength--;
    }
}
