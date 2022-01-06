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
 * @brief
 *   This file includes the platform-specific initializers.
 */

#include <stdio.h>

#include <openthread-system.h>

#include <wiced_memory.h>
#include <wiced_platform.h>

#if !OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
#include "ifx_ot_cli_cmd.h"
#endif

extern void otPlatAlramInit(void);
extern void otPlatRadioInit(void);
extern void otPlatUartInit(void);

typedef struct
{
    struct
    {
        uint32_t post_init;
    } event_code;
} system_cb_t;

static system_cb_t system_cb = {0};

#ifndef CHIP_HAVE_CONFIG_H
static void system_post_init(void)
{
    printf("system_post_init\n");

#if !OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE
    ifx_ot_cli_cmd_install(otInstanceInitSingle());
#endif // !OPENTHREAD_CONFIG_MULTIPLE_INSTANCE_ENABLE

    printf("Free RAM sizes: %lu\n", wiced_memory_get_free_bytes());
}
#endif // !CHIP_HAVE_CONFIG_H

void otSysInit(int argc, char *argv[])
{
    OT_UNUSED_VARIABLE(argc);
    OT_UNUSED_VARIABLE(argv);

    /* Initialize the Alarm Abstraction Layer. */
    otPlatAlramInit();

    /* Initialize the Radio Abstraction Layer. */
    otPlatRadioInit();

#ifndef CHIP_HAVE_CONFIG_H
    /* Initialize the UART Abstraction Layer. */
    otPlatUartInit();

    /* Get event code for system post-initialization. */
    if (!wiced_platform_application_thread_event_register(&system_cb.event_code.post_init, system_post_init))
    {
        printf("%s: Fail to get event code.\n", __FUNCTION__);
        return;
    }

    /* Issue an event for system post-initialization. */
    wiced_platform_application_thread_event_set(system_cb.event_code.post_init);
#endif /* CHIP_HAVE_CONFIG_H */
}

void otSysDeinit(void)
{
}

bool otSysPseudoResetWasRequested(void)
{
    return false;
}

void otSysProcessDrivers(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    wiced_platform_application_thread_event_dispatch();
}
