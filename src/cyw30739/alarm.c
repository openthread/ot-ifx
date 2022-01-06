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
 *   This file implements the OpenThread platform abstraction for the alarm.
 */

#include <openthread/platform/alarm-milli.h>

#include <slist.h>
#include <stdio.h>
#include <string.h>

#include <clock_timer.h>
#include <wiced_platform.h>
#include <wiced_platform_memory.h>
#include <wiced_rtos.h>
#include <wiced_timer.h>

#ifndef ALARM_DEBUG
#define ALARM_DEBUG 0
#endif // ALARM_DEBUG

#if (ALARM_DEBUG != 0)
#define ALARM_TRACE(format, ...) printf(format, ##__VA_ARGS__)
#else
#define ALARM_TRACE(...)
#endif

#ifndef ALARM_TRIGGER_REPLACE
#define ALARM_TRIGGER_REPLACE 1
#endif // ALARM_TRIGGER_REPLACE

/* Enabled timer information. */
typedef struct alarm_timer_info
{
    slist_node_t  node;
    otInstance *  aInstance;
    wiced_timer_t timer;
} alarm_timer_info_t;

/* Fired alarm. */
typedef struct alarm_fired
{
    slist_node_t node;
    otInstance * aInstance;
} alarm_fired_t;

/* Alarm module control block. */
typedef struct alarm_cb
{
    wiced_bool_t   initialized;
    uint32_t       event_code;
    slist_node_t   enabled_timer_list; /* Refer to alarm_timer_info_t. */
    wiced_mutex_t *p_mutex;
    slist_node_t   fired_alarm_list; /* Refer to alarm_fired_t. */
} alarm_cb_t;

static alarm_cb_t alarm_cb = {0};

static wiced_bool_t alarmEnabledTimerListCheck(alarm_timer_info_t *p_target)
{
    slist_node_t *p_node;
    int           i;

    for (i = 0; i < slist_count(&alarm_cb.enabled_timer_list); i++)
    {
        p_node = slist_get(&alarm_cb.enabled_timer_list);
        slist_add_tail(p_node, &alarm_cb.enabled_timer_list);

        if (p_node == (slist_node_t *)p_target)
        {
            return WICED_TRUE;
        }
    }

    return WICED_FALSE;
}

static alarm_timer_info_t *alarmEnabledTimerListAdd(otInstance *aInstance)
{
    slist_node_t *p_node;

    /* Allocate memory */
    p_node = (slist_node_t *)wiced_platform_memory_allocate(sizeof(alarm_timer_info_t));

    if (!p_node)
    {
        return NULL;
    }

    memset((void *)p_node, 0, sizeof(alarm_timer_info_t));

    /* Add to the tail of enabled timer list. */
    slist_add_tail(p_node, &alarm_cb.enabled_timer_list);

    /* Store information. */
    ((alarm_timer_info_t *)p_node)->aInstance = aInstance;

    return (alarm_timer_info_t *)p_node;
}

static void alarmEnabledTimerListRemove(alarm_timer_info_t *p_target)
{
    slist_node_t *p_node;
    int           i;

    for (i = 0; i < slist_count(&alarm_cb.enabled_timer_list); i++)
    {
        p_node = slist_get(&alarm_cb.enabled_timer_list);

        if (p_node == (slist_node_t *)p_target)
        {
            /* Free memory. */
            wiced_platform_memory_free((void *)p_node);

            break;
        }
        else
        {
            slist_add_tail(p_node, &alarm_cb.enabled_timer_list);
        }
    }
}

static void alarmTimerCallback(WICED_TIMER_PARAM_TYPE cb_params)
{
    otInstance *  aInstance;
    slist_node_t *p_node;

    wiced_rtos_lock_mutex(alarm_cb.p_mutex);

    /* Check if the target entry exists in the enabled timer list. */
    if (!alarmEnabledTimerListCheck((alarm_timer_info_t *)cb_params))
    {
        wiced_rtos_unlock_mutex(alarm_cb.p_mutex);
        return;
    }

    aInstance = ((alarm_timer_info_t *)cb_params)->aInstance;

    /* Remove the entry from the enable timer list. */
    alarmEnabledTimerListRemove((alarm_timer_info_t *)cb_params);

    /* Add a new entry to the fired alarm list. */
    p_node = (slist_node_t *)wiced_platform_memory_allocate(sizeof(alarm_fired_t));

    if (!p_node)
    {
        ALARM_TRACE("%s: Err: Cannot allocate fired alarm entry\n", __FUNCTION__);

        wiced_rtos_unlock_mutex(alarm_cb.p_mutex);
        return;
    }

    slist_add_tail(p_node, &alarm_cb.fired_alarm_list);

    /* Store information. */
    ((alarm_fired_t *)p_node)->aInstance = aInstance;

    /* Set an application thread event for this fired alarm. */
    wiced_platform_application_thread_event_set(alarm_cb.event_code);

    wiced_rtos_unlock_mutex(alarm_cb.p_mutex);
}

static void alarmTimeoutHandler(void)
{
    slist_node_t *p_node;
    otInstance *  aInstance;

    ALARM_TRACE("%s\n", __FUNCTION__);

    wiced_rtos_lock_mutex(alarm_cb.p_mutex);

    /* Check the fired alarm list.*/
    if (slist_count(&alarm_cb.fired_alarm_list) <= 0)
    {
        wiced_rtos_unlock_mutex(alarm_cb.p_mutex);

        return;
    }

    /* Get the first entry from the fired alarm list. */
    p_node = slist_get(&alarm_cb.fired_alarm_list);

    aInstance = ((alarm_fired_t *)p_node)->aInstance;

    wiced_platform_memory_free((void *)p_node);

    wiced_rtos_unlock_mutex(alarm_cb.p_mutex);

    /* Signal the fired alarm. */
#if OPENTHREAD_CONFIG_DIAG_ENABLE
    if (otPlatDiagModeGet())
    {
        otPlatDiagAlarmFired(aInstance);
    }
    else
#endif /* OPENTHREAD_CONFIG_DIAG_ENABLE */
    {
        otPlatAlarmMilliFired(aInstance);
    }

    /* Set an application thread event if the fired alarm list is not empty. */
    if (slist_count(&alarm_cb.fired_alarm_list) > 0)
    {
        wiced_platform_application_thread_event_set(alarm_cb.event_code);
    }
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
#if ALARM_TRIGGER_REPLACE
    int i;
#endif // ALARM_TRIGGER_REPLACE
    slist_node_t *p_node;
    uint32_t      target_timeout;
    uint32_t      time_now;

    ALARM_TRACE("%s (0x%p, %lu, %lu)\n", __FUNCTION__, aInstance, aT0, aDt);

    wiced_platform_application_thread_check();

    /* Check state. */
    if (!alarm_cb.initialized)
    {
        return;
    }

    /* Check parameter. */
    if (!aInstance)
    {
        return;
    }

    wiced_rtos_lock_mutex(alarm_cb.p_mutex);

#if ALARM_TRIGGER_REPLACE
    /* Check if the entry already exists in the enabled timer list. */
    for (i = 0; i < slist_count(&alarm_cb.enabled_timer_list); i++)
    {
        p_node = slist_get(&alarm_cb.enabled_timer_list);

        if (((alarm_timer_info_t *)p_node)->aInstance == aInstance)
        {
            /* Stop timer. */
            wiced_stop_timer(&(((alarm_timer_info_t *)p_node)->timer));

            /* Free memory. */
            wiced_platform_memory_free((void *)p_node);

            break;
        }
        else
        {
            slist_add_tail(p_node, &alarm_cb.enabled_timer_list);
        }
    }
#endif

    /* Add an entry to the enabled timer list and store information. */
    p_node = (slist_node_t *)alarmEnabledTimerListAdd(aInstance);

    if (!p_node)
    {
        ALARM_TRACE("Err: cannot add an enabled timer entry.\n");
        wiced_rtos_unlock_mutex(alarm_cb.p_mutex);

        return;
    }

    /* Calculate target timeout. */
    time_now = otPlatAlarmMilliGetNow();

    if (time_now >= aT0)
    { // Current time exceeds the target reference (start) time.
        if (aDt <= (time_now - aT0))
        { // Current time exceeds the target fired time.
            /* Trigger this timer immediately. */
            wiced_rtos_unlock_mutex(alarm_cb.p_mutex);

            alarmTimerCallback((WICED_TIMER_PARAM_TYPE)p_node);

            return;
        }
        else
        {
            target_timeout = aDt - (time_now - aT0);
        }
    }
    else
    {
        target_timeout = aDt + (aT0 - time_now);
    }

    /* Initialize the timer module. */
    wiced_init_timer(&(((alarm_timer_info_t *)p_node)->timer), alarmTimerCallback, (WICED_TIMER_PARAM_TYPE)p_node,
                     WICED_MILLI_SECONDS_TIMER);

    /* Start timer. */
    wiced_start_timer(&(((alarm_timer_info_t *)p_node)->timer), target_timeout);

    wiced_rtos_unlock_mutex(alarm_cb.p_mutex);
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    int           i;
    slist_node_t *p_node;

    ALARM_TRACE("%s (%p)\n", __FUNCTION__, aInstance);

    wiced_platform_application_thread_check();

    wiced_rtos_lock_mutex(alarm_cb.p_mutex);

    /* Find the target alarm. */
    for (i = 0; i < slist_count(&alarm_cb.enabled_timer_list); i++)
    {
        p_node = slist_get(&alarm_cb.enabled_timer_list);

        if (((alarm_timer_info_t *)p_node)->aInstance == aInstance)
        {
            /* Stop timer. */
            wiced_stop_timer(&(((alarm_timer_info_t *)p_node)->timer));

            /* Free memory. */
            wiced_platform_memory_free((void *)p_node);
        }
        else
        {
            slist_add_tail(p_node, &alarm_cb.enabled_timer_list);
        }
    }

    wiced_rtos_unlock_mutex(alarm_cb.p_mutex);
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    return (uint32_t)(clock_SystemTimeMicroseconds64() / 1000);
}

uint32_t otPlatAlarmMicroGetNow(void)
{
    return clock_SystemTimeMicroseconds32();
}

void otPlatAlramInit(void)
{
    if (alarm_cb.initialized)
    {
        return;
    }

    /* Get/Register the application thread event code. */
    if (!wiced_platform_application_thread_event_register(&alarm_cb.event_code, alarmTimeoutHandler))
    {
        ALARM_TRACE("%s: Fail to get event code.\n", __FUNCTION__);
        return;
    }

    /* Create mutex. */
    alarm_cb.p_mutex = wiced_rtos_create_mutex();

    if (alarm_cb.p_mutex == NULL)
    {
        ALARM_TRACE("%s: Fail to create mutex.\n", __FUNCTION__);
        return;
    }

    /* Initialize the mutex. */
    if (wiced_rtos_init_mutex(alarm_cb.p_mutex) != WICED_SUCCESS)
    {
        ALARM_TRACE("%s: Fail to init. mutex.\n", __FUNCTION__);
        return;
    }

    /* Initialize the enabled timer list. */
    INIT_SLIST_NODE(&alarm_cb.enabled_timer_list);

    /* Initialize the fired alarm list. */
    INIT_SLIST_NODE(&alarm_cb.fired_alarm_list);

    alarm_cb.initialized = WICED_TRUE;
}
