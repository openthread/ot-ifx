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
 *   This file implements the OpenThread platform abstraction for radio communication.
 */

#include <openthread-core-config.h>
#include <openthread/platform/entropy.h>
#include <openthread/platform/radio.h>

#include <inttypes.h>
#include <slist.h>
#include <stdio.h>
#include <string.h>

#include "i15dot4.h"

#include <hcidefs.h>
#if PACKET_STATISTICS
#include "packet_statistics.h"
#endif // PACKET_STATISTICS
#include <platform_nvram.h>
#include <wiced_bt_dev.h>
#include <wiced_misc_rtos_utils.h>
#include <wiced_platform.h>
#include <wiced_platform_memory.h>
#include <wiced_rtos.h>
#include <wiced_timer.h>

#ifndef RADIO_DEBUG
#define RADIO_DEBUG 0
#endif // RADIO_DEBUG

#if (RADIO_DEBUG != 0)
#define RADIO_TRACE(format, ...) printf(format, ##__VA_ARGS__)
#else
#define RADIO_TRACE(...)
#endif

#ifndef RADIO_TRANSMIT_POWER_IN_DBM
#define RADIO_TRANSMIT_POWER_IN_DBM 0
#endif // RADIO_TRANSMIT_POWER_IN_DBM

#ifndef RADIO_RECEIVE_SENSITIVITY
#define RADIO_RECEIVE_SENSITIVITY (-103) // dBm
#endif                                   // RADIO_RECEIVE_SENSITIVITY

#define RADIO_DATA_WAIT_CONF_TIMEOUT (8000)          // ms
#define RADIO_DATA_WAIT_CONF_PROCESS_STEP_TIME (500) // ms

#if (OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)
#define RADIO_CAPABILITY                                                                      \
    (OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_ENERGY_SCAN | OT_RADIO_CAPS_TRANSMIT_RETRIES | \
     OT_RADIO_CAPS_CSMA_BACKOFF | OT_RADIO_CAPS_SLEEP_TO_TX)
#else // (OPENTHREAD_CONFIG_THREAD_VERSION != OT_THREAD_VERSION_1_1)
#define RADIO_CAPABILITY                                                                      \
    (OT_RADIO_CAPS_ACK_TIMEOUT | OT_RADIO_CAPS_ENERGY_SCAN | OT_RADIO_CAPS_TRANSMIT_RETRIES | \
     OT_RADIO_CAPS_CSMA_BACKOFF | OT_RADIO_CAPS_SLEEP_TO_TX | OT_RADIO_CAPS_TRANSMIT_TIMING)
#endif // (OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)

#define HCI_VSC_I15DOT4_META_OPCODE (HCI_GRP_VENDOR_SPECIFIC | 0x0177)
#define HCI_VSE_I15DOT4_META_EVENT (0x5D)

/* Definition used for IEEE EUI64 Storing Method. */
#define RADIO_IEEE_EUI64_STORING_METHOD_NVRAM 1

#ifndef RADIO_IEEE_EUI64_STORING_METHOD
#define RADIO_IEEE_EUI64_STORING_METHOD RADIO_IEEE_EUI64_STORING_METHOD_NVRAM
#define RADIO_IEEE_EUI64_NVRAM_ID PLATFORM_NVRAM_SSID_OPENTHREAD_BASE

#endif // RADIO_IEEE_EUI64_STORING_METHOD

#define RADIO_I15DOT4_SEQ_NUM_LENGTH (1)

#if defined(WICED_PLATFORM_I15DOT4_PHY_ED_THRESHOLD)
#define I15DOT4_PHY_ED_THRESHOLD_VALUE WICED_PLATFORM_I15DOT4_PHY_ED_THRESHOLD
#else // !defined(WICED_PLATFORM_I15DOT4_PHY_ED_THRESHOLD)
#define I15DOT4_PHY_ED_THRESHOLD_VALUE (-70)
#endif // defined(WICED_PLATFORM_I15DOT4_PHY_ED_THRESHOLD)

typedef struct radio_i15dot4_mapping
{
    uint8_t                                          req_id;
    uint8_t                                          rsp_id;
    uint32_t                                         event_code;
    wiced_platform_application_thread_event_handler *p_event_handler;
} radio_i15dot4_mapping_t;

typedef struct radio_data_wait_conf_entry
{
    slist_node_t  node;
    otInstance *  aInstance;
    otRadioFrame *aTxFrame;
    uint8_t       ackFrame[I15DOT4_ACK_LENGTH];
    uint8_t       msdu_handle;
    uint32_t      process_time;
    uint8_t       status; // refer to I15DOT4_STATUS_t
} radio_data_wait_conf_entry_t;

typedef struct radio_data_received_frame
{
    slist_node_t node;
    otRadioFrame rxFrame;
} radio_data_received_frame_t;

typedef struct radio_cb
{
    bool         initialized;
    otExtAddress eui64;

    struct
    {
        uint8_t  attrib_len;
        uint8_t *p_attrib_data;
    } i15dot4_get_conf;

    struct
    {
        uint8_t status;
    } i15dot4_set_conf;

    struct
    {
        bool                scanning;
        wiced_timer_t       timer;
        I15DOT4_SCAN_TYPE_t type;

        struct
        {
            otInstance *aInstance;
            uint8_t     scan_channel;
            int8_t      rssi;
        } ed_scan;
    } scan;

    struct
    {
        struct
        {
            otRadioFrame  radio_frame;
            uint8_t       psdu[OT_RADIO_FRAME_MAX_SIZE];
            uint8_t       msdu;
            uint8_t       msdu_handle;
            slist_node_t  wait_conf_list; // refer to radio_data_wait_conf_entry_t
            slist_node_t  confirmed_list; // refer to radio_data_wait_conf_entry_t
            wiced_timer_t timer;
        } tx;

        struct
        {
            otInstance * aInstance;
            slist_node_t frame_list; // refer to radio_data_received_frame_t
        } rx;
    } data;

    struct
    {
        otRadioState current;
        otRadioState next;
        uint8_t      channel;
        uint16_t     pan_id;

        struct
        {
            uint8_t rx_on_when_idle;
            uint8_t rx_enable;
        } rx;
    } radio_state;

    struct
    {
        uint8_t status;
    } thread_addr_match_conf;

    struct
    {
        uint16_t opcode;
        uint8_t  param_len;
        uint8_t *p_param;
    } vsc_cmd;

#if PACKET_STATISTICS
    PACKET_STATISTICS_t statistics;
#endif // PACKET_STATISTICS
} radio_cb_t;

/* Declaration of static functions. */
static void          radio_hci_vsc_command_queue_reset(void);
static void          radio_hci_vsc_command_send(uint16_t opcode, uint8_t param_len, uint8_t *p_param);
static void          radio_hci_vsc_command_send_handler(void *p_data);
static void          radio_hci_vse_callback(uint8_t len, uint8_t *p);
static void          radio_hci_vse_callback_mlme_set_conf(uint8_t len, uint8_t *p);
static void          radio_hci_vse_callback_mlme_get_conf(uint8_t len, uint8_t *p);
static void          radio_hci_vse_callback_mlme_rx_enable_conf(uint8_t len, uint8_t *p);
static void          radio_hci_vse_callback_mlme_scan_conf(uint8_t len, uint8_t *p);
static void          radio_hci_vse_callback_thread_addr_match_conf(uint8_t len, uint8_t *p);
static void          radio_hci_vse_callback_thread_data_conf(uint8_t len, uint8_t *p);
static void          radio_hci_vse_callback_thread_data_ind(uint8_t len, uint8_t *p);
static uint32_t      radio_i15dot4_application_thread_event_code_get(uint8_t i15dot4_cmd_event_id);
static wiced_bool_t  radio_i15dot4_mcps_data_req_msdu_handle_get(uint8_t *p_msdu_handle);
static void          radio_i15dot4_mcps_data_req_timeout_handler(WICED_TIMER_PARAM_TYPE cb_params);
static slist_node_t *radio_i15dot4_mcps_data_req_wait_conf_add(uint8_t       msdu_handle,
                                                               otInstance *  aInstance,
                                                               otRadioFrame *aFrame);
static void          radio_i15dot4_mcps_data_req_wait_conf_remove(slist_node_t *p_target, wiced_bool_t free);
static void          radio_i15dot4_mlme_get_conf_data_reset(void);
static wiced_bool_t  radio_i15dot4_mlme_get_req(uint8_t attribute_id);
static wiced_bool_t  radio_i15dot4_mlme_rx_enable_req(wiced_bool_t defer_permit,
                                                      uint32_t     rx_on_time,
                                                      uint32_t     rx_on_duration);
static void          radio_i15dot4_mlme_scan_conf_handler(void);
static void          radio_i15dot4_mlme_scan_conf_timeout_handler(WICED_TIMER_PARAM_TYPE cb_params);
static wiced_bool_t  radio_i15dot4_mlme_scan_req(uint8_t  scan_type,
                                                 uint32_t scan_channel,
                                                 uint8_t  scan_duration,
                                                 uint8_t  channel_page);
static wiced_bool_t  radio_i15dot4_mlme_set_req(uint8_t attribute_id, uint32_t attribute_len, uint8_t *p_attribute);
static wiced_bool_t  radio_i15dot4_thread_addr_match_req(ADDR_MATCH_ACTION_t action, I15DOT4_ADDR_t *p_addr);
static void          radio_i15dot4_thread_data_conf_handler(void);
static void          radio_i15dot4_thread_data_ind_handler(void);
static wiced_bool_t  radio_i15dot4_thread_data_req(otRadioFrame *aFrame, uint8_t msdu_handle);
static void          radio_ieee_eui64_load(void);
static void          radio_utils_data_display(uint8_t data_len, uint8_t *p_data);
static void          radio_utils_i15dot4_thread_data_ind_display(I15DOT4_THREAD_DATA_IND_t *p_thread_data_ind);
static void          radio_utils_radio_frame_display(otRadioFrame *p_frame, wiced_bool_t tx);
static uint8_t       radio_utils_scan_time_to_scan_duration_calculate(uint32_t scan_time, uint8_t channel_num);

/* Declaration of static variables. */
static radio_i15dot4_mapping_t radio_i15dot4_mapping_table[] = {
    {I15DOT4_CMDID_MLME_GET_REQ, I15DOT4_CMDID_MLME_GET_CONF, 0, NULL},
    {I15DOT4_CMDID_MLME_RX_ENABLE_REQ, I15DOT4_CMDID_MLME_RX_ENABLE_CONF, 0, NULL},
    {I15DOT4_CMDID_MLME_SCAN_REQ, I15DOT4_CMDID_MLME_SCAN_CONF, 0, radio_i15dot4_mlme_scan_conf_handler},
    {I15DOT4_CMDID_MLME_SET_REQ, I15DOT4_CMDID_MLME_SET_CONF, 0, NULL},
    {I15DOT4_CMDID_THREAD_DATA_REQ, I15DOT4_CMDID_THREAD_DATA_CONF, 0, radio_i15dot4_thread_data_conf_handler},
    {I15DOT4_CMDID_THREAD_DATA_IND, I15DOT4_CMDID_THREAD_DATA_IND, 0, radio_i15dot4_thread_data_ind_handler},
    {I15DOT4_CMDID_THREAD_ADDR_MATCH_REQ, I15DOT4_CMDID_THREAD_ADDR_MATCH_CONF, 0, NULL},
};

static radio_cb_t radio_cb = {0};

#if PACKET_STATISTICS

/**************************************************************************************************
 * Function:     ifx_ot_cli_cmd_install
 *
 * Abstract:     Install the Infineon specific OpenThread CLI commands to stack.
 *
 * Input/Output:
 *   otInstance *ot_instance (I) - the instance
 *
 * Return:       None
 *
 * Notices:
 **************************************************************************************************/
void otPlatRadioPacketStatisticsGet(PACKET_STATISTICS_t *p_statistics)
{
    /* Check parameter. */
    if (!p_statistics)
        return;

    /* Copy data. */
    memcpy((void *)p_statistics, (void *)&radio_cb.statistics, sizeof(radio_cb.statistics));
}

#endif // PACKET_STATISTICS

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    return RADIO_CAPABILITY;
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    return RADIO_RECEIVE_SENSITIVITY;
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    memcpy((void *)aIeeeEui64, (void *)&radio_cb.eui64, sizeof(radio_cb.eui64));

    radio_utils_data_display(sizeof(radio_cb.eui64), (uint8_t *)aIeeeEui64);
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t aPanid)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s (0x%04X)\n", __FUNCTION__, aPanid);

    if (radio_i15dot4_mlme_set_req(I15DOT4_MAC_PAN_ID, sizeof(aPanid), (uint8_t *)&aPanid) == WICED_FALSE)
    {
        return;
    }

    /* Check status. */
    if (radio_cb.i15dot4_set_conf.status != I15DOT4_STATUS_SUCCESS)
    {
        return;
    }

    /* Update information. */
    radio_cb.radio_state.pan_id = aPanid;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    radio_i15dot4_mlme_set_req(I15DOT4_MAC_EXTENDED_ADDRESS, sizeof(otExtAddress), (uint8_t *)aAddress);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    radio_i15dot4_mlme_set_req(I15DOT4_MAC_MAC_SHORT_ADDRESS, sizeof(aAddress), (uint8_t *)&aAddress);
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aPower);

    RADIO_TRACE("%s\n", __FUNCTION__);

    /* Check parameter. */
    if (!aPower)
    {
        return OT_ERROR_INVALID_ARGS;
    }

#if (!RADIO_TRANSMIT_POWER_IN_DBM)
    return OT_ERROR_NOT_IMPLEMENTED;
#else  // RADIO_TRANSMIT_POWER_IN_DBM
    if (!radio_i15dot4_mlme_get_req(I15DOT4_PHY_TX_POWER))
    {
        return OT_ERROR_FAILED;
    }

    /* Check if the MLME-GET.req is successful. */
    if (radio_cb.i15dot4_get_conf.p_attrib_data)
    {
        memcpy((void *)aPower, (void *)radio_cb.i15dot4_get_conf.p_attrib_data, radio_cb.i15dot4_get_conf.attrib_len);

        radio_i15dot4_mlme_get_conf_data_reset();

        return OT_ERROR_NONE;
    }

    return OT_ERROR_FAILED;
#endif // !RADIO_TRANSMIT_POWER_IN_DBM
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    OT_UNUSED_VARIABLE(aInstance);

#if (!RADIO_DEBUG)
    OT_UNUSED_VARIABLE(aPower);
#endif

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, aPower);

#if (!RADIO_TRANSMIT_POWER_IN_DBM)
    return OT_ERROR_NOT_IMPLEMENTED;
#else  // RADIO_TRANSMIT_POWER_IN_DBM
    if (radio_i15dot4_mlme_set_req(I15DOT4_PHY_TX_POWER, sizeof(aPower), (uint8_t *)&aPower) == WICED_FALSE)
    {
        return OT_ERROR_FAILED;
    }

    /* Check status. */
    if (radio_cb.i15dot4_set_conf.status != I15DOT4_STATUS_SUCCESS)
    {
        return OT_ERROR_FAILED;
    }

    return OT_ERROR_NONE;
#endif // !RADIO_TRANSMIT_POWER_IN_DBM
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);

    RADIO_TRACE("%s\n", __FUNCTION__);

    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aThreshold);

    RADIO_TRACE("%s\n", __FUNCTION__);

    return OT_ERROR_NOT_IMPLEMENTED;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    bool mode = false;

    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    if (!radio_i15dot4_mlme_get_req(I15DOT4_MAC_PROMISCUOUS_MODE))
    {
        return false;
    }

    if (radio_cb.i15dot4_get_conf.p_attrib_data)
    {
        memcpy((void *)&mode, (void *)radio_cb.i15dot4_get_conf.p_attrib_data, sizeof(mode));

        radio_i15dot4_mlme_get_conf_data_reset();
    }

    return mode;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    radio_i15dot4_mlme_set_req(I15DOT4_MAC_PROMISCUOUS_MODE, sizeof(aEnable), (uint8_t *)&aEnable);
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    return radio_cb.radio_state.current;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, radio_cb.radio_state.current);

    /* Check radio module state. */
    if (!radio_cb.initialized)
    {
        return OT_ERROR_FAILED;
    }

    switch (radio_cb.radio_state.current)
    {
    case OT_RADIO_STATE_SLEEP:
    case OT_RADIO_STATE_RECEIVE:
    case OT_RADIO_STATE_TRANSMIT:
        break;

    case OT_RADIO_STATE_DISABLED:
    case OT_RADIO_STATE_INVALID:
    default:
        radio_cb.radio_state.current = OT_RADIO_STATE_SLEEP;
        break;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, radio_cb.radio_state.current);

    if (radio_cb.radio_state.current != OT_RADIO_STATE_SLEEP)
    {
        return OT_ERROR_INVALID_STATE;
    }

    radio_cb.radio_state.current = OT_RADIO_STATE_DISABLED;

    return OT_ERROR_NONE;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, radio_cb.radio_state.current);

    switch (radio_cb.radio_state.current)
    {
    case OT_RADIO_STATE_SLEEP:
    case OT_RADIO_STATE_RECEIVE:
    case OT_RADIO_STATE_TRANSMIT:
        return true;

    case OT_RADIO_STATE_DISABLED:
    case OT_RADIO_STATE_INVALID:
    default:
        break;
    }

    return false;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, radio_cb.radio_state.current);

    /* Check current radio state. */
    switch (radio_cb.radio_state.current)
    {
    case OT_RADIO_STATE_SLEEP:
        return OT_ERROR_NONE;

    case OT_RADIO_STATE_RECEIVE:
    case OT_RADIO_STATE_TRANSMIT:
        break;

    case OT_RADIO_STATE_DISABLED:
    case OT_RADIO_STATE_INVALID:
    default:
        return OT_ERROR_INVALID_STATE;
    }

    /* Set PIB rx_on_when_idle. */
    if (radio_cb.radio_state.rx.rx_on_when_idle)
    {
        radio_cb.radio_state.rx.rx_on_when_idle = 0;

        if (radio_i15dot4_mlme_set_req(I15DOT4_MAC_RX_ON_WHEN_IDLE, sizeof(radio_cb.radio_state.rx.rx_on_when_idle),
                                       &radio_cb.radio_state.rx.rx_on_when_idle) == WICED_FALSE)
        {
            radio_cb.radio_state.rx.rx_on_when_idle = 1;
            return OT_ERROR_FAILED;
        }

        /* Check status. */
        if (radio_cb.i15dot4_set_conf.status != I15DOT4_STATUS_SUCCESS)
        {
            radio_cb.radio_state.rx.rx_on_when_idle = 1;
            return OT_ERROR_FAILED;
        }
    }

    /* Set expected state. */
    radio_cb.radio_state.next = OT_RADIO_STATE_SLEEP;

    /* Turn off the radio. */
    if (radio_cb.radio_state.rx.rx_enable)
    {
        if (!radio_i15dot4_mlme_rx_enable_req(WICED_FALSE, 0x00ffffff, 0))
        {
            return OT_ERROR_FAILED;
        }

        /* Check current state. */
        if (radio_cb.radio_state.current != OT_RADIO_STATE_SLEEP)
        {
            return OT_ERROR_FAILED;
        }
    }
    else
    {
        radio_cb.radio_state.current = OT_RADIO_STATE_SLEEP;
    }

    radio_cb.radio_state.rx.rx_enable = 0;

    return OT_ERROR_NONE;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    RADIO_TRACE("%s (%p, %d, 0x%02X, 0x%02X) (%d, %d)\n", __FUNCTION__, aInstance, radio_cb.radio_state.current,
                radio_cb.radio_state.channel, aChannel, radio_cb.radio_state.rx.rx_on_when_idle,
                radio_cb.radio_state.rx.rx_enable);

    /* Check current state. */
    switch (radio_cb.radio_state.current)
    {
    case OT_RADIO_STATE_SLEEP:
    case OT_RADIO_STATE_TRANSMIT:
        /* Check current channel. */
        if (radio_cb.radio_state.channel != aChannel)
        { // Current channel is different from target channel.
            /* Set channel to target channel. */
            if (!radio_i15dot4_mlme_set_req(I15DOT4_PHY_CURRENT_CHANNEL, sizeof(aChannel), (uint8_t *)&aChannel))
            {
                return OT_ERROR_FAILED;
            }

            /* Check status. */
            if (radio_cb.i15dot4_set_conf.status != I15DOT4_STATUS_SUCCESS)
            {
                return OT_ERROR_FAILED;
            }

            /* Save information. */
            radio_cb.radio_state.channel = aChannel;
        }

        /* Set expected state. */
        radio_cb.radio_state.next = OT_RADIO_STATE_RECEIVE;

        if (!radio_cb.radio_state.rx.rx_on_when_idle)
        {
            radio_cb.radio_state.rx.rx_on_when_idle = 1;

            /* Set PIB rx_on_when_idle. */
            if (radio_i15dot4_mlme_set_req(I15DOT4_MAC_RX_ON_WHEN_IDLE, sizeof(radio_cb.radio_state.rx.rx_on_when_idle),
                                           &radio_cb.radio_state.rx.rx_on_when_idle) == WICED_FALSE)
            {
                radio_cb.radio_state.rx.rx_on_when_idle = 0;
                return OT_ERROR_FAILED;
            }

            /* Check status. */
            if (radio_cb.i15dot4_set_conf.status != I15DOT4_STATUS_SUCCESS)
            {
                radio_cb.radio_state.rx.rx_on_when_idle = 0;
                return OT_ERROR_FAILED;
            }
        }

        if (!radio_cb.radio_state.rx.rx_enable)
        {
            /* Turn on the radio. */
            if (!radio_i15dot4_mlme_rx_enable_req(WICED_FALSE, 0x00ffffff, 0x00ffffff))
            {
                return OT_ERROR_FAILED;
            }

            /* Check current state. */
            if (radio_cb.radio_state.current != OT_RADIO_STATE_RECEIVE)
            {
                return OT_ERROR_FAILED;
            }

            radio_cb.radio_state.rx.rx_enable = 1;
        }
        else
        {
            radio_cb.radio_state.current = OT_RADIO_STATE_RECEIVE;
        }

        radio_cb.data.rx.aInstance = aInstance;

        return OT_ERROR_NONE;

    case OT_RADIO_STATE_RECEIVE:
        /* Check current channel. */
        if (radio_cb.radio_state.channel == aChannel)
        {
            radio_cb.data.rx.aInstance = aInstance;

            return OT_ERROR_NONE;
        }

        /* Set channel to target channel.
         * Note here we do NOT need to turn of the radio in prior to set
         * the channel. The i15dot4 module will do this automatically. */
        if (!radio_i15dot4_mlme_set_req(I15DOT4_PHY_CURRENT_CHANNEL, sizeof(aChannel), (uint8_t *)&aChannel))
        {
            return OT_ERROR_FAILED;
        }

        /* Check status. */
        if (radio_cb.i15dot4_set_conf.status != I15DOT4_STATUS_SUCCESS)
        {
            return OT_ERROR_FAILED;
        }

        /* Save information. */
        radio_cb.radio_state.channel = aChannel;
        radio_cb.data.rx.aInstance   = aInstance;

        return OT_ERROR_NONE;

    case OT_RADIO_STATE_DISABLED:
    case OT_RADIO_STATE_INVALID:
    default:
        return OT_ERROR_INVALID_STATE;
    }
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    return &radio_cb.data.tx.radio_frame;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    uint8_t       msdu_handle;
    slist_node_t *p_new_entry;

    RADIO_TRACE("%s (%d, %d => %d)\n", __FUNCTION__, radio_cb.radio_state.current, radio_cb.radio_state.channel,
                aFrame->mChannel);

    /* Check radio state. */
    if (radio_cb.scan.scanning)
    {
        return OT_ERROR_INVALID_STATE;
    }

    switch (radio_cb.radio_state.current)
    {
    case OT_RADIO_STATE_SLEEP:
        if (!(RADIO_CAPABILITY & OT_RADIO_CAPS_SLEEP_TO_TX))
        {
            return OT_ERROR_INVALID_STATE;
        }
        break;

    case OT_RADIO_STATE_RECEIVE:
    case OT_RADIO_STATE_TRANSMIT:
        break;

    case OT_RADIO_STATE_DISABLED:
    case OT_RADIO_STATE_INVALID:
    default:
        return OT_ERROR_INVALID_STATE;
    }

    /* Check current channel. */
    if (radio_cb.radio_state.channel != aFrame->mChannel)
    {
        /* Set channel to target channel. */
        if (!radio_i15dot4_mlme_set_req(I15DOT4_PHY_CURRENT_CHANNEL, sizeof(aFrame->mChannel),
                                        (uint8_t *)&aFrame->mChannel))
        {
            return OT_ERROR_FAILED;
        }

        /* Check status. */
        if (radio_cb.i15dot4_set_conf.status != I15DOT4_STATUS_SUCCESS)
        {
            return OT_ERROR_FAILED;
        }

        /* Save information. */
        radio_cb.radio_state.channel = aFrame->mChannel;
    }

    /* Get the msdu handle. */
    if (!radio_i15dot4_mcps_data_req_msdu_handle_get(&msdu_handle))
    {
        return OT_ERROR_FAILED;
    }

    /* Add a MCPS-DATA.req wait confirm entry. */
    p_new_entry = radio_i15dot4_mcps_data_req_wait_conf_add(msdu_handle, aInstance, aFrame);

    /* Transmit mpdu to i15dot4 module. */
    if (!radio_i15dot4_thread_data_req(aFrame, msdu_handle))
    {
        radio_i15dot4_mcps_data_req_wait_conf_remove(p_new_entry, WICED_TRUE);

        return OT_ERROR_FAILED;
    }

    /* Set state. */
    radio_cb.radio_state.current = OT_RADIO_STATE_TRANSMIT;

    /* Notify OpenThread stack. */
    otPlatRadioTxStarted(aInstance, aFrame);

#if PACKET_STATISTICS

    radio_cb.statistics.tx.num++;

#endif // PACKET_STATISTICS

    return OT_ERROR_NONE;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    return 0;
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    uint8_t scan_duration;

    RADIO_TRACE("%s (0x%02X, %d)\n", __FUNCTION__, aScanChannel, aScanDuration);

    /* Check capability. */
    if (!(RADIO_CAPABILITY & OT_RADIO_CAPS_ENERGY_SCAN))
    {
        OT_UNUSED_VARIABLE(aScanChannel);
        OT_UNUSED_VARIABLE(aScanDuration);

        return OT_ERROR_NOT_IMPLEMENTED;
    }

    /* Check state. */
    if (radio_cb.scan.scanning)
    {
        return OT_ERROR_INVALID_STATE;
    }

    /* Check parameter. */
    if ((aScanChannel < OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MIN) || (aScanChannel > OT_RADIO_2P4GHZ_OQPSK_CHANNEL_MAX))
    {
        return OT_ERROR_INVALID_ARGS;
    }

    /* Convert scan duration from millisecond to the ScanDuration attribute value defined in the
     * IEEE 802.15.4 standard. */
    scan_duration = radio_utils_scan_time_to_scan_duration_calculate((uint32_t)aScanDuration * 1000, 1);

    /* Create a timer to avoid missing the corresponding MLME-SCAN.conf primitive from
     * i15dot4 module. */
    wiced_start_timer(&radio_cb.scan.timer, aScanDuration + 500);

    /* Ask i15dot4 module to do the energy detection. */
    if (!radio_i15dot4_mlme_scan_req(I15DOT4_SCAN_TYPE_ED, (uint32_t)1 << aScanChannel, scan_duration,
                                     OT_RADIO_CHANNEL_PAGE_0))
    {
        wiced_stop_timer(&radio_cb.scan.timer);

        return OT_ERROR_FAILED;
    }

    /* Set state. */
    radio_cb.scan.scanning = true;

    /* Save information. */
    radio_cb.scan.type                 = I15DOT4_SCAN_TYPE_ED;
    radio_cb.scan.ed_scan.aInstance    = aInstance;
    radio_cb.scan.ed_scan.scan_channel = aScanChannel;
    radio_cb.scan.ed_scan.rssi         = 0;

    return OT_ERROR_NONE;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, aEnable);

    radio_i15dot4_thread_addr_match_req(aEnable ? ADDR_MATCH_ENABLE : ADDR_MATCH_DISABLE, NULL);
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    I15DOT4_ADDR_t addr = {0};

    RADIO_TRACE("%s (0x%04X)\n", __FUNCTION__, aShortAddress);

    addr.short_addr = aShortAddress;

    if (!radio_i15dot4_thread_addr_match_req(ADDR_MATCH_SHORT_ADD, &addr))
    {
        return OT_ERROR_FAILED;
    }

    /* Check status. */
    switch (radio_cb.thread_addr_match_conf.status)
    {
    case I15DOT4_STATUS_SUCCESS:
        return OT_ERROR_NONE;

    case I15DOT4_STATUS_LIMIT_REACHED:
    case I15DOT4_STATUS_UNKNOWN_ERROR:
        return OT_ERROR_NO_BUFS;

    default:
        break;
    }

    return OT_ERROR_FAILED;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    I15DOT4_ADDR_t addr = {0};

    RADIO_TRACE("%s\n", __FUNCTION__);

#if RADIO_DEBUG
    {
        uint8_t i;

        RADIO_TRACE("aExtAddress: 0x");
        for (i = 0; i < OT_EXT_ADDRESS_SIZE; i++)
        {
            printf("%02X", aExtAddress->m8[i]);
        }
        RADIO_TRACE("\n");
    }
#endif

    memcpy((void *)&addr.ext_addr, (void *)aExtAddress, sizeof(otExtAddress));

    if (!radio_i15dot4_thread_addr_match_req(ADDR_MATCH_EXT_ADD, &addr))
    {
        return OT_ERROR_FAILED;
    }

    /* Check status. */
    switch (radio_cb.thread_addr_match_conf.status)
    {
    case I15DOT4_STATUS_SUCCESS:
        return OT_ERROR_NONE;

    case I15DOT4_STATUS_LIMIT_REACHED:
    case I15DOT4_STATUS_UNKNOWN_ERROR:
        return OT_ERROR_NO_BUFS;

    default:
        break;
    }

    return OT_ERROR_FAILED;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    I15DOT4_ADDR_t addr = {0};

    RADIO_TRACE("%s (0x%04X)\n", __FUNCTION__, aShortAddress);

    addr.short_addr = aShortAddress;

    if (!radio_i15dot4_thread_addr_match_req(ADDR_MATCH_SHORT_CLEAR, &addr))
    {
        return OT_ERROR_FAILED;
    }

    /* Check status. */
    switch (radio_cb.thread_addr_match_conf.status)
    {
    case I15DOT4_STATUS_LIMIT_REACHED:
    case I15DOT4_STATUS_NO_DATA:
        return OT_ERROR_NO_ADDRESS;

    case I15DOT4_STATUS_SUCCESS:
        return OT_ERROR_NONE;

    default:
        break;
    }

    return OT_ERROR_FAILED;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    OT_UNUSED_VARIABLE(aInstance);

    I15DOT4_ADDR_t addr = {0};

    memcpy((void *)&addr.ext_addr, (void *)aExtAddress, sizeof(otExtAddress));

    RADIO_TRACE("%s (0x%" PRIx64 ")\n", __FUNCTION__, addr.ext_addr);

    if (!radio_i15dot4_thread_addr_match_req(ADDR_MATCH_EXT_CLEAR, &addr))
    {
        return OT_ERROR_FAILED;
    }

    /* Check status. */
    switch (radio_cb.thread_addr_match_conf.status)
    {
    case I15DOT4_STATUS_LIMIT_REACHED:
    case I15DOT4_STATUS_NO_DATA:
        return OT_ERROR_NO_ADDRESS;

    case I15DOT4_STATUS_SUCCESS:
        return OT_ERROR_NONE;

    default:
        break;
    }

    return OT_ERROR_FAILED;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    radio_i15dot4_thread_addr_match_req(ADDR_MATCH_SHORT_CLEAR_ALL, NULL);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    RADIO_TRACE("%s\n", __FUNCTION__);

    radio_i15dot4_thread_addr_match_req(ADDR_MATCH_EXT_CLEAR_ALL, NULL);
}

void otPlatRadioInit(void)
{
    uint8_t i;
    int8_t  cca_mode;
    int8_t  ed_threshold;

    if (radio_cb.initialized)
    {
        return;
    }

    /* Register the HCI VSE callback. */
    if (!wiced_platform_register_hci_vse_callback(HCI_VSE_I15DOT4_META_EVENT, radio_hci_vse_callback))
    {
        RADIO_TRACE("%s: Fail to register HCI VSE callback.\n", __FUNCTION__);
        return;
    }

    /* Initialize the timer used for scanning. */
    wiced_init_timer(&radio_cb.scan.timer, radio_i15dot4_mlme_scan_conf_timeout_handler, 0, WICED_MILLI_SECONDS_TIMER);

    /* Get/Register the application thread event code. */
    for (i = 0; i < sizeof(radio_i15dot4_mapping_table) / sizeof(radio_i15dot4_mapping_t); i++)
    {
        if (!wiced_platform_application_thread_event_register(&radio_i15dot4_mapping_table[i].event_code,
                                                              radio_i15dot4_mapping_table[i].p_event_handler))
        {
            RADIO_TRACE("%s: Fail to get event code for %d\n", __FUNCTION__, radio_i15dot4_mapping_table[i].req_id);
            return;
        }
    }

    /* Initialize the MCPS-DATA.req wait confirm list. */
    INIT_SLIST_NODE(&radio_cb.data.tx.wait_conf_list);

    /* Initialize the MCPS-DATA.req confirmed list. */
    INIT_SLIST_NODE(&radio_cb.data.tx.confirmed_list);

    /* Initialize the MCPS-DATA.req wait confirm timer. */
    wiced_init_timer(&radio_cb.data.tx.timer, radio_i15dot4_mcps_data_req_timeout_handler, 0,
                     WICED_MILLI_SECONDS_PERIODIC_TIMER);

    radio_cb.data.tx.radio_frame.mPsdu = &radio_cb.data.tx.psdu[0];

    /* Initialize the received frame list. */
    INIT_SLIST_NODE(&radio_cb.data.rx.frame_list);

    /* Load the IEEE EUI64 address. */
    radio_ieee_eui64_load();

    /* Set the ED Threshold value. */
    ed_threshold = I15DOT4_PHY_ED_THRESHOLD_VALUE;
    radio_i15dot4_mlme_set_req(I15DOT4_CY_ED_THRESHOLD, sizeof(ed_threshold), (uint8_t *)&ed_threshold);

    /* Set the CCA Mode to Carrier Sense only (Mode 2). */
    cca_mode = I15DOT4_PHY_CCA_MODE_3;
    radio_i15dot4_mlme_set_req(I15DOT4_PHY_CCA_MODE, sizeof(cca_mode), (uint8_t *)&cca_mode);

    radio_cb.initialized = true;
}

//---------------------------- Static Function ----------------------------------------------------

/*
 * Helper function to get the assigned application thread event code for target
 * i15dot4 command/event id.
 */
static uint32_t radio_i15dot4_application_thread_event_code_get(uint8_t i15dot4_cmd_event_id)
{
    uint8_t i;

    for (i = 0; i < sizeof(radio_i15dot4_mapping_table) / sizeof(radio_i15dot4_mapping_t); i++)
    {
        if ((radio_i15dot4_mapping_table[i].req_id == i15dot4_cmd_event_id) ||
            (radio_i15dot4_mapping_table[i].rsp_id == i15dot4_cmd_event_id))
        {
            return radio_i15dot4_mapping_table[i].event_code;
        }
    }

    return 0;
}

/*
 * Reset data in MLME GET CONF queue
 */
static void radio_i15dot4_mlme_get_conf_data_reset(void)
{
    if (radio_cb.i15dot4_get_conf.p_attrib_data)
    {
        wiced_platform_memory_free((void *)radio_cb.i15dot4_get_conf.p_attrib_data);
    }

    radio_cb.i15dot4_get_conf.attrib_len    = 0;
    radio_cb.i15dot4_get_conf.p_attrib_data = NULL;
}

/*
 * Issue a MLME-SET.req primitive to i15dot4 module.
 */
static wiced_bool_t radio_i15dot4_mlme_set_req(uint8_t attribute_id, uint32_t attribute_len, uint8_t *p_attribute)
{
    I15DOT4_MLME_SET_REQ_t mlme_set_req = {0};

    RADIO_TRACE("%s (0x%02X)\n", __FUNCTION__, attribute_id);

    wiced_platform_application_thread_check();

    mlme_set_req.context_id = CONTEXT_ID_ZBPRO;
    mlme_set_req.cmd_id     = I15DOT4_CMDID_MLME_SET_REQ;
    mlme_set_req.attrib_id  = attribute_id;
    mlme_set_req.attrib_len = attribute_len;
    memcpy((void *)&mlme_set_req.attrib[0], (void *)p_attribute, attribute_len);

    radio_hci_vsc_command_send(HCI_VSC_I15DOT4_META_OPCODE,
                               sizeof(mlme_set_req) - sizeof(mlme_set_req.attrib) + mlme_set_req.attrib_len,
                               (uint8_t *)&mlme_set_req);

    /* Wait for corresponding confirm message. */
    wiced_platform_application_thread_event_wait(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_SET_REQ));

    return WICED_TRUE;
}

/*
 * Issue a MLME-GET.req primitive to i15dot4 module.
 */
static wiced_bool_t radio_i15dot4_mlme_get_req(uint8_t attribute_id)
{
    I15DOT4_MLME_GET_REQ_t mlme_get_req = {0};

    RADIO_TRACE("%s (0x%02X)\n", __FUNCTION__, attribute_id);

    wiced_platform_application_thread_check();

    mlme_get_req.context_id = CONTEXT_ID_ZBPRO;
    mlme_get_req.cmd_id     = I15DOT4_CMDID_MLME_GET_REQ;
    mlme_get_req.attrib_id  = attribute_id;

    radio_hci_vsc_command_send(HCI_VSC_I15DOT4_META_OPCODE, sizeof(mlme_get_req), (uint8_t *)&mlme_get_req);

    /* Wait for corresponding confirm message. */
    wiced_platform_application_thread_event_wait(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_GET_REQ));

    return WICED_TRUE;
}

/*
 * Issue a MLME-RX-ENABLE.req primitive to i15dot4 module.
 */
static wiced_bool_t radio_i15dot4_mlme_rx_enable_req(wiced_bool_t defer_permit,
                                                     uint32_t     rx_on_time,
                                                     uint32_t     rx_on_duration)
{
    I15DOT4_MLME_RX_ENABLE_REQ_t mlme_rx_enable_req = {0};

    RADIO_TRACE("%s (%d, %08lx, %08lx)\n", __FUNCTION__, defer_permit, rx_on_time, rx_on_duration);

    mlme_rx_enable_req.context_id     = CONTEXT_ID_ZBPRO;
    mlme_rx_enable_req.cmd_id         = I15DOT4_CMDID_MLME_RX_ENABLE_REQ;
    mlme_rx_enable_req.defer_permit   = (UINT8)defer_permit;
    mlme_rx_enable_req.rx_on_time     = rx_on_time;
    mlme_rx_enable_req.rx_on_duration = rx_on_duration;

    radio_hci_vsc_command_send(HCI_VSC_I15DOT4_META_OPCODE, sizeof(mlme_rx_enable_req), (uint8_t *)&mlme_rx_enable_req);

    /* Wait for corresponding confirm message. */
    wiced_platform_application_thread_event_wait(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_RX_ENABLE_REQ));

    return WICED_TRUE;
}

/*
 * Issue a MLME-SCAN.req primitive to i15dot4 module.
 */
static wiced_bool_t radio_i15dot4_mlme_scan_req(uint8_t  scan_type,
                                                uint32_t scan_channel,
                                                uint8_t  scan_duration,
                                                uint8_t  channel_page)
{
    I15DOT4_MLME_SCAN_REQ_t mlme_scan_req = {0};

    RADIO_TRACE("%s (%d, 0x%08lx, %d, %d)\n", __FUNCTION__, scan_type, scan_channel, scan_duration, channel_page);

    mlme_scan_req.context_id    = CONTEXT_ID_ZBPRO;
    mlme_scan_req.cmd_id        = I15DOT4_CMDID_MLME_SCAN_REQ;
    mlme_scan_req.scan_type     = scan_type;
    mlme_scan_req.scan_channel  = scan_channel;
    mlme_scan_req.scan_duration = scan_duration;
    mlme_scan_req.channel_page  = channel_page;

    /*
     * The security related parameters are omitted.
     */

    radio_hci_vsc_command_send(HCI_VSC_I15DOT4_META_OPCODE, sizeof(mlme_scan_req), (uint8_t *)&mlme_scan_req);

    return WICED_TRUE;
}

/*
 * MCPS-DATA.req primitive timeout handler.
 *
 * This handler will be executed every RADIO_DATA_WAIT_CONF_PROCESS_STEP_TIME if started.
 */
static void radio_i15dot4_mcps_data_req_timeout_handler(WICED_TIMER_PARAM_TYPE cb_params)
{
    int           i;
    slist_node_t *p_node;

    OT_UNUSED_VARIABLE(cb_params);

    RADIO_TRACE("%s\n", __FUNCTION__);

    /* Increment the process time. */
    for (i = 0; i < slist_count(&radio_cb.data.tx.wait_conf_list); i++)
    {
        p_node = slist_get(&radio_cb.data.tx.wait_conf_list);

        ((radio_data_wait_conf_entry_t *)p_node)->process_time += RADIO_DATA_WAIT_CONF_PROCESS_STEP_TIME;

        if (((radio_data_wait_conf_entry_t *)p_node)->process_time >= RADIO_DATA_WAIT_CONF_TIMEOUT)
        {
            ((radio_data_wait_conf_entry_t *)p_node)->status = I15DOT4_STATUS_RAW_TIMEOUT;

            /* Move this entry to the data confirmed list. */
            slist_add_tail(p_node, &radio_cb.data.tx.confirmed_list);
        }
        else
        {
            /* Add this entry back to the data wait confirm list. */
            slist_add_tail(p_node, &radio_cb.data.tx.wait_conf_list);
        }
    }

    /* Stop MCPS-DATA.req wait confirm timer if not needed. */
    if (!slist_count(&radio_cb.data.tx.wait_conf_list))
    {
        wiced_stop_timer(&radio_cb.data.tx.timer);
    }

    /* Set an event to application thread to process the timeout entry. */
    if (slist_count(&radio_cb.data.tx.confirmed_list))
    {
        wiced_platform_application_thread_event_set(
            radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MCPS_DATA_CONF));
    }
}

/*
 * Remove an entry from the MCPS-DATA.req primitive wait confirm list.
 */
static void radio_i15dot4_mcps_data_req_wait_conf_remove(slist_node_t *p_target, wiced_bool_t free)
{
    slist_del(p_target, &radio_cb.data.tx.wait_conf_list);

    if (free)
    {
        wiced_platform_memory_free((void *)p_target);
    }
}

/*
 * Add a new entry to the MCPS-DATA.req primitive wait confirm list.
 */
static slist_node_t *radio_i15dot4_mcps_data_req_wait_conf_add(uint8_t       msdu_handle,
                                                               otInstance *  aInstance,
                                                               otRadioFrame *aFrame)
{
    slist_node_t *p_new;

    /* Allocate memory. */
    p_new = (slist_node_t *)wiced_platform_memory_allocate(sizeof(radio_data_wait_conf_entry_t));

    if (!p_new)
    {
        return NULL;
    }

    memset((void *)p_new, 0, sizeof(radio_data_wait_conf_entry_t));

    /* Fill in information. */
    ((radio_data_wait_conf_entry_t *)p_new)->aInstance    = aInstance;
    ((radio_data_wait_conf_entry_t *)p_new)->aTxFrame     = aFrame;
    ((radio_data_wait_conf_entry_t *)p_new)->msdu_handle  = msdu_handle;
    ((radio_data_wait_conf_entry_t *)p_new)->process_time = 0;

    /* Start the MCPS-DATA.req primitive timer. */
    if (slist_count(&radio_cb.data.tx.wait_conf_list) == 1)
    {
        /* Start timer. */
        wiced_start_timer(&radio_cb.data.tx.timer, RADIO_DATA_WAIT_CONF_PROCESS_STEP_TIME);
    }

    /* Add to the list. */
    slist_add_tail(p_new, &radio_cb.data.tx.wait_conf_list);

    return p_new;
}

/*
 * Utility to get a msdu handle used for MCPS-DATA.req primitive.
 */
static wiced_bool_t radio_i15dot4_mcps_data_req_msdu_handle_get(uint8_t *p_msdu_handle)
{
    uint8_t       target_handle;
    uint8_t       start_handle;
    wiced_bool_t  used = WICED_FALSE;
    int           i;
    slist_node_t *p_node;

    start_handle  = radio_cb.data.tx.msdu_handle++;
    target_handle = start_handle;

    do
    {
        if (used)
        {
            target_handle = radio_cb.data.tx.msdu_handle++;

            if (start_handle == target_handle)
            {
                return WICED_FALSE;
            }

            used = WICED_FALSE;
        }

        /* Check if the target msdu handle is already assigned to a MCPS-DATA.req primitive. */
        for (i = 0; i < slist_count(&radio_cb.data.tx.wait_conf_list); i++)
        {
            p_node = slist_get(&radio_cb.data.tx.wait_conf_list);

            if (((radio_data_wait_conf_entry_t *)p_node)->msdu_handle == target_handle)
            {
                used = WICED_TRUE;
            }

            slist_add_tail(p_node, &radio_cb.data.tx.wait_conf_list);
        }
    } while (used);

    *p_msdu_handle = target_handle;

    return WICED_TRUE;
}

static wiced_bool_t radio_i15dot4_thread_data_req(otRadioFrame *aFrame, uint8_t msdu_handle)
{
    I15DOT4_THREAD_DATA_REQ_t thread_data_req = {0};

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, msdu_handle);
    radio_utils_radio_frame_display(aFrame, WICED_TRUE);

    /* Check frame length. */
    if (aFrame->mLength > I15DOT4_MAX_PHY_PACKET_SIZE)
    {
        return WICED_FALSE;
    }

    thread_data_req.context_id = CONTEXT_ID_ZBPRO;
    thread_data_req.cmd_id     = I15DOT4_CMDID_THREAD_DATA_REQ;
    thread_data_req.handle     = msdu_handle;
    memcpy((void *)&thread_data_req.key[0], (void *)aFrame->mInfo.mTxInfo.mAesKey, OT_MAC_KEY_SIZE);
    thread_data_req.max_csma_backoffs     = aFrame->mInfo.mTxInfo.mMaxCsmaBackoffs;
    thread_data_req.max_frame_retries     = aFrame->mInfo.mTxInfo.mMaxFrameRetries;
    thread_data_req.csma_ca_enabled       = aFrame->mInfo.mTxInfo.mCsmaCaEnabled;
    thread_data_req.is_security_processed = aFrame->mInfo.mTxInfo.mIsSecurityProcessed;
    thread_data_req.psdu_length           = aFrame->mLength - I15DOT4_FCS_LENGTH;
    memcpy((void *)&thread_data_req.psdu[0], (void *)aFrame->mPsdu, thread_data_req.psdu_length);

    radio_hci_vsc_command_send(HCI_VSC_I15DOT4_META_OPCODE,
                               sizeof(thread_data_req) - sizeof(thread_data_req.psdu) + thread_data_req.psdu_length,
                               (uint8_t *)&thread_data_req);

    return WICED_TRUE;
}

/*
 * Issue a THREAD-ADDR_MATCH.req primitive to i15dot4 module.
 */
static wiced_bool_t radio_i15dot4_thread_addr_match_req(ADDR_MATCH_ACTION_t action, I15DOT4_ADDR_t *p_addr)
{
    I15DOT4_THREAD_ADDR_MATCH_REQ_t thread_addr_match_req = {0};

    wiced_platform_application_thread_check();

    thread_addr_match_req.context_id = CONTEXT_ID_ZBPRO;
    thread_addr_match_req.cmd_id     = I15DOT4_CMDID_THREAD_ADDR_MATCH_REQ;
    thread_addr_match_req.action     = (uint8_t)action;
    if (p_addr)
    {
        memcpy((void *)&thread_addr_match_req.addr, (void *)p_addr, sizeof(I15DOT4_ADDR_t));
    }

    radio_hci_vsc_command_send(HCI_VSC_I15DOT4_META_OPCODE, sizeof(thread_addr_match_req),
                               (uint8_t *)&thread_addr_match_req);

    /* Wait for corresponding confirm message. */
    wiced_platform_application_thread_event_wait(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_THREAD_ADDR_MATCH_REQ));

    return WICED_TRUE;
}

/*
 * MLME-SCAN.req primitive timeout handler.
 */
static void radio_i15dot4_mlme_scan_conf_timeout_handler(WICED_TIMER_PARAM_TYPE cb_params)
{
    OT_UNUSED_VARIABLE(cb_params);

    RADIO_TRACE("%s\n", __FUNCTION__);

    /* Check state. */
    if (!radio_cb.scan.scanning)
    {
        return;
    }

    switch (radio_cb.scan.type)
    {
    case I15DOT4_SCAN_TYPE_ED:
        /* Set an event to application thread. */
        wiced_platform_application_thread_event_set(
            radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_SCAN_CONF));
        break;

    default:
        break;
    }
}

/*
 * Utility to send HCI VSC command in MPAF thread.
 */
static void radio_hci_vsc_command_send(uint16_t opcode, uint8_t param_len, uint8_t *p_param)
{
    wiced_result_t result;

    /* Allocate memory. */
    radio_cb.vsc_cmd.p_param = (uint8_t *)wiced_platform_memory_allocate(param_len);

    if (!radio_cb.vsc_cmd.p_param)
    {
        RADIO_TRACE("Err: cannot allocate memory for vsc command.\n");
    }

    /* Store information. */
    radio_cb.vsc_cmd.opcode    = opcode;
    radio_cb.vsc_cmd.param_len = param_len;
    memcpy((void *)radio_cb.vsc_cmd.p_param, (void *)p_param, param_len);

    /* Call MPAF thread. */
    result = wiced_rtos_defer_execution(WICED_RTOS_DEFER_TO_MPAF_THREAD, &radio_hci_vsc_command_send_handler, NULL);

    if (result != WICED_SUCCESS)
    {
        RADIO_TRACE("Err: wiced_rtos_defer_execution (%d)\n", result);

        radio_hci_vsc_command_queue_reset();

        return;
    }
}

/*
 * Note: Runs in MPAF thread
 */
static void radio_hci_vsc_command_send_handler(void *p_data)
{
    wiced_result_t result;

    OT_UNUSED_VARIABLE(p_data);

    /* Check the HCI VSC command queue. */
    if (!radio_cb.vsc_cmd.p_param)
    {
        RADIO_TRACE("Err: No data in VSC cmd queue.\n");

        return;
    }

    /* Send the queued HCI VSC command. */
    result = wiced_bt_dev_vendor_specific_command(radio_cb.vsc_cmd.opcode, radio_cb.vsc_cmd.param_len,
                                                  radio_cb.vsc_cmd.p_param, NULL);

    if (result != WICED_SUCCESS)
    {
        RADIO_TRACE("%s: Fail to send VSC (%d)\n", __FUNCTION__, result);

        radio_hci_vsc_command_queue_reset();

        return;
    }

    radio_hci_vsc_command_queue_reset();
}

static void radio_hci_vsc_command_queue_reset(void)
{
    if (radio_cb.vsc_cmd.p_param)
    {
        wiced_platform_memory_free((void *)radio_cb.vsc_cmd.p_param);
    }

    memset((void *)&radio_cb.vsc_cmd, 0, sizeof(radio_cb.vsc_cmd));
}

/*
 * HCI VSE handler
 */
static void radio_hci_vse_callback(uint8_t len, uint8_t *p)
{
    I15DOT4_PARAM_HDR_t *p_header;

    /* Check parameter. */
    if (p == NULL)
    {
        return;
    }

    /* Check data length. */
    if (len < sizeof(I15DOT4_PARAM_HDR_t))
    {
        return;
    }

    /* Check context id. */
    p_header = (I15DOT4_PARAM_HDR_t *)p;

    if (p_header->context_id != CONTEXT_ID_ZBPRO)
    {
        return;
    }

    /* Process the event according to the command id.*/
    switch (p_header->cmd_id)
    {
    case I15DOT4_CMDID_MLME_GET_CONF:
        radio_hci_vse_callback_mlme_get_conf(len, p);
        break;

    case I15DOT4_CMDID_MLME_RX_ENABLE_CONF:
        radio_hci_vse_callback_mlme_rx_enable_conf(len, p);
        break;

    case I15DOT4_CMDID_MLME_SCAN_CONF:
        radio_hci_vse_callback_mlme_scan_conf(len, p);
        break;

    case I15DOT4_CMDID_MLME_SET_CONF:
        radio_hci_vse_callback_mlme_set_conf(len, p);
        break;

    case I15DOT4_CMDID_THREAD_DATA_CONF:
        radio_hci_vse_callback_thread_data_conf(len, p);
        break;

    case I15DOT4_CMDID_THREAD_DATA_IND:
        radio_hci_vse_callback_thread_data_ind(len, p);
        break;

    case I15DOT4_CMDID_THREAD_ADDR_MATCH_CONF:
        radio_hci_vse_callback_thread_addr_match_conf(len, p);
        break;

    default:
        break;
    }
}

/*
 * HCI VSE handler for THREAD-DATA.ind primitive
 */
static void radio_hci_vse_callback_thread_data_ind(uint8_t len, uint8_t *p)
{
    I15DOT4_THREAD_DATA_IND_t *  p_thread_data_ind;
    radio_data_received_frame_t *p_rx_frame = NULL;
    uint8_t *                    p_psdu     = NULL;

    /* Check data length. */
    if (len != sizeof(I15DOT4_THREAD_DATA_IND_t))
    {
        return;
    }

    p_thread_data_ind = (I15DOT4_THREAD_DATA_IND_t *)p;

    /* Check parameter. */
    if ((p_thread_data_ind->psdu_length + I15DOT4_FCS_LENGTH) > I15DOT4_MAX_PHY_PACKET_SIZE)
    {
        return;
    }

    radio_utils_i15dot4_thread_data_ind_display(p_thread_data_ind);

    /* Allocate memory for this incoming frame. */
    p_psdu = (uint8_t *)wiced_platform_memory_allocate(p_thread_data_ind->psdu_length + I15DOT4_FCS_LENGTH);
    if (p_psdu == NULL)
    {
        RADIO_TRACE("%s: Err: Fail to allocate memory for incoming psdu (%d)\n", __FUNCTION__,
                    p_thread_data_ind->psdu_length + I15DOT4_FCS_LENGTH);

        return;
    }

    memset((void *)p_psdu, 0, p_thread_data_ind->psdu_length + I15DOT4_FCS_LENGTH);

    p_rx_frame = (radio_data_received_frame_t *)wiced_platform_memory_allocate(sizeof(radio_data_received_frame_t));
    if (p_rx_frame == NULL)
    {
        RADIO_TRACE("%s: Err: Fail to allocate memory for incoming frame.\n", __FUNCTION__);

        wiced_platform_memory_free((void *)p_psdu);

        return;
    }

    memset((void *)p_rx_frame, 0, sizeof(radio_data_received_frame_t));

    /* Fill in information. */
    memcpy((void *)p_psdu, (void *)p_thread_data_ind->psdu, p_thread_data_ind->psdu_length);
    p_rx_frame->rxFrame.mPsdu                                = p_psdu;
    p_rx_frame->rxFrame.mLength                              = p_thread_data_ind->psdu_length + I15DOT4_FCS_LENGTH;
    p_rx_frame->rxFrame.mChannel                             = radio_cb.radio_state.channel;
    p_rx_frame->rxFrame.mInfo.mRxInfo.mTimestamp             = (uint64_t)p_thread_data_ind->symbol_counter;
    p_rx_frame->rxFrame.mInfo.mRxInfo.mRssi                  = (int8_t)p_thread_data_ind->rssi;
    p_rx_frame->rxFrame.mInfo.mRxInfo.mAckedWithFramePending = p_thread_data_ind->ack_frame_pending;

    /* Add this entry to the receive frame list. */
    slist_add_tail((slist_node_t *)p_rx_frame, &radio_cb.data.rx.frame_list);

    /* Set an event to application thread to process the entry. */
    wiced_platform_application_thread_event_set(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_THREAD_DATA_IND));
}

/*
 * HCI VSE handler for THREAD-DATA.conf primitive
 */
static void radio_hci_vse_callback_thread_data_conf(uint8_t len, uint8_t *p)
{
    I15DOT4_THREAD_DATA_CONF_t *p_thread_data_conf;
    int                         i;
    slist_node_t *              p_node;
    I15DOT4_FRAME_CTRL_t *      p_frame_ctrl;

    /* Check data length. */
    if (len != sizeof(I15DOT4_THREAD_DATA_CONF_t))
    {
        return;
    }

    p_thread_data_conf = (I15DOT4_THREAD_DATA_CONF_t *)p;

    RADIO_TRACE("%s (%d, 0x%02X)\n", __FUNCTION__, p_thread_data_conf->handle, p_thread_data_conf->status);

#if (RADIO_DEBUG)
    RADIO_TRACE("ACK Frame: ");
    for (i = 0; i < I15DOT4_ACK_LENGTH; i++) RADIO_TRACE("0x%02X ", p_thread_data_conf->ack_frame[i]);
    RADIO_TRACE("\n");
#endif

    /* Check if the corresponding THREAD-DATA.req exists. */
    for (i = 0; i < slist_count(&radio_cb.data.tx.wait_conf_list); i++)
    {
        p_node = slist_get(&radio_cb.data.tx.wait_conf_list);

        if (((radio_data_wait_conf_entry_t *)p_node)->msdu_handle == p_thread_data_conf->handle)
        {
            ((radio_data_wait_conf_entry_t *)p_node)->status = p_thread_data_conf->status;

            /* Keep Ack frame information if exists. */
            p_frame_ctrl = (I15DOT4_FRAME_CTRL_t *)p_thread_data_conf->ack_frame;
            if (p_frame_ctrl->field.frame_type == I15DOT4_FRAME_TYPE_ACK)
            {
                memcpy((void *)((radio_data_wait_conf_entry_t *)p_node)->ackFrame,
                       (void *)p_thread_data_conf->ack_frame, I15DOT4_ACK_LENGTH);
            }

            /* Move this entry to the data confirmed list. */
            slist_add_tail(p_node, &radio_cb.data.tx.confirmed_list);

            break;
        }
        else
        {
            /* Add this entry back to the data wait confirm list. */
            slist_add_tail(p_node, &radio_cb.data.tx.wait_conf_list);
        }
    }

    /* Stop data wait confirm timer if not needed. */
    if (!slist_count(&radio_cb.data.tx.wait_conf_list))
    {
        wiced_stop_timer(&radio_cb.data.tx.timer);
    }

    /* Set an event to application thread to process the entry. */
    if (slist_count(&radio_cb.data.tx.confirmed_list))
    {
        wiced_platform_application_thread_event_set(
            radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_THREAD_DATA_CONF));
    }
}

/*
 * HCI VSE handler for THREAD-ADDR_MATCH.conf primitive
 */
static void radio_hci_vse_callback_thread_addr_match_conf(uint8_t len, uint8_t *p)
{
    I15DOT4_THREAD_ADDR_MATCH_CONF_t *p_thread_addr_match_conf;

    /* Check data length. */
    if (len != sizeof(I15DOT4_THREAD_ADDR_MATCH_CONF_t))
    {
        return;
    }

    p_thread_addr_match_conf               = (I15DOT4_THREAD_ADDR_MATCH_CONF_t *)p;
    radio_cb.thread_addr_match_conf.status = p_thread_addr_match_conf->status;

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, p_thread_addr_match_conf->status);

    wiced_platform_application_thread_event_set(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_THREAD_ADDR_MATCH_CONF));
}

/*
 * HCI VSE handler for MLME-SCAN.conf primitive
 */
static void radio_hci_vse_callback_mlme_scan_conf(uint8_t len, uint8_t *p)
{
    I15DOT4_MLME_SCAN_CONF_t *p_mlme_scan_conf;

    /* Check data length. */
    if (len != sizeof(I15DOT4_MLME_SCAN_CONF_t))
    {
        return;
    }

    p_mlme_scan_conf = (I15DOT4_MLME_SCAN_CONF_t *)p;

    RADIO_TRACE("%s (%d, 0x%02X, %d, %d, 0x%08x, %d)\n", __FUNCTION__, radio_cb.scan.scanning, p_mlme_scan_conf->status,
                p_mlme_scan_conf->scan_type, p_mlme_scan_conf->channel_page, p_mlme_scan_conf->unscanned_channels,
                p_mlme_scan_conf->result_list_size);

    /* Check state. */
    if (!radio_cb.scan.scanning)
    {
        return;
    }

    /* Check parameter. */
    if (p_mlme_scan_conf->channel_page != OT_RADIO_CHANNEL_PAGE_0)
    {
        return;
    }

    /* Stop the scan timer. */
    wiced_stop_timer(&radio_cb.scan.timer);

    switch (p_mlme_scan_conf->scan_type)
    {
    case I15DOT4_SCAN_TYPE_ED:
        if ((p_mlme_scan_conf->status != I15DOT4_STATUS_SUCCESS) || (p_mlme_scan_conf->unscanned_channels != 0) ||
            (p_mlme_scan_conf->result_list_size != 1))
        {
            goto exit;
        }

        radio_cb.scan.ed_scan.rssi = (int8_t)p_mlme_scan_conf->result.ed[0];
        break;

    default:
        return;
    }

exit:

    /* Set an event to application thread. */
    wiced_platform_application_thread_event_set(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_SCAN_CONF));
}

/*
 * HCI VSE handler for MLME-RX-ENABLE.conf primitive
 */
static void radio_hci_vse_callback_mlme_rx_enable_conf(uint8_t len, uint8_t *p)
{
    I15DOT4_MLME_RX_ENABLE_CONF_t *p_mlme_radio_enable_conf;

    /* Check data length. */
    if (len != sizeof(I15DOT4_MLME_RX_ENABLE_CONF_t))
    {
        return;
    }

    p_mlme_radio_enable_conf = (I15DOT4_MLME_RX_ENABLE_CONF_t *)p;

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, p_mlme_radio_enable_conf->status);

    if (p_mlme_radio_enable_conf->status == I15DOT4_STATUS_SUCCESS)
    {
        radio_cb.radio_state.current = radio_cb.radio_state.next;
    }

    wiced_platform_application_thread_event_set(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_RX_ENABLE_CONF));
}

/*
 * HCI VSE handler for MLME-SET.conf primitive
 */
static void radio_hci_vse_callback_mlme_set_conf(uint8_t len, uint8_t *p)
{
    I15DOT4_MLME_SET_CONF_t *p_mlme_set_conf;

    /* Check data length. */
    if (len != sizeof(I15DOT4_MLME_SET_CONF_t))
    {
        return;
    }

    p_mlme_set_conf                  = (I15DOT4_MLME_SET_CONF_t *)p;
    radio_cb.i15dot4_set_conf.status = p_mlme_set_conf->status;

    RADIO_TRACE("%s (0x%02X, %d)\n", __FUNCTION__, p_mlme_set_conf->attrib_id, p_mlme_set_conf->status);

    wiced_platform_application_thread_event_set(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_SET_CONF));
}

/*
 * HCI VSE handler for MLME-GET.conf primitive
 */
static void radio_hci_vse_callback_mlme_get_conf(uint8_t len, uint8_t *p)
{
    I15DOT4_MLME_GET_CONF_t *p_mlme_get_conf;

    /* Check data length. */
    if (len != sizeof(I15DOT4_MLME_GET_CONF_t))
    {
        return;
    }

    p_mlme_get_conf = (I15DOT4_MLME_GET_CONF_t *)p;

    RADIO_TRACE("%s (0x%02X, %d)\n", __FUNCTION__, p_mlme_get_conf->attrib_id, p_mlme_get_conf->status);

    if (p_mlme_get_conf->status == I15DOT4_STATUS_SUCCESS)
    {
        radio_utils_data_display(p_mlme_get_conf->attrib_len, &p_mlme_get_conf->attrib[0]);

        /* Allocate memory. */
        radio_cb.i15dot4_get_conf.p_attrib_data =
            (uint8_t *)wiced_platform_memory_allocate(p_mlme_get_conf->attrib_len);

        if (!radio_cb.i15dot4_get_conf.p_attrib_data)
        {
            RADIO_TRACE("%s: Fail to allocate memory\n", __FUNCTION__);
        }
        else
        {
            radio_cb.i15dot4_get_conf.attrib_len = p_mlme_get_conf->attrib_len;

            memcpy((void *)radio_cb.i15dot4_get_conf.p_attrib_data, (void *)&p_mlme_get_conf->attrib[0],
                   p_mlme_get_conf->attrib_len);
        }
    }

    /* Set application thread event to unblock the application thread. */
    wiced_platform_application_thread_event_set(
        radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_MLME_GET_CONF));
}

/*
 * MLME-SCAN.conf primitive handler
 */
static void radio_i15dot4_mlme_scan_conf_handler(void)
{
    RADIO_TRACE("%s (%d, %i)\n", __FUNCTION__, radio_cb.scan.scanning, radio_cb.scan.ed_scan.rssi);

    /* Set state. */
    radio_cb.scan.scanning = false;

    /* Inform OpenThread stack. */
    otPlatRadioEnergyScanDone(radio_cb.scan.ed_scan.aInstance, radio_cb.scan.ed_scan.rssi);
}

/*
 * THREAD-DATA.conf primitive handler
 */
static void radio_i15dot4_thread_data_conf_handler(void)
{
    slist_node_t *        p_node;
    otError               aError = OT_ERROR_CHANNEL_ACCESS_FAILURE;
    I15DOT4_FRAME_CTRL_t *p_frame_ctrl;
    wiced_bool_t          ack_frame_included = WICED_FALSE;
    otRadioFrame          ack_frame          = {0};

    RADIO_TRACE("%s\n", __FUNCTION__);

    /* Check the content of data confirmed list. */
    if (!slist_count(&radio_cb.data.tx.confirmed_list))
    {
        return;
    }

    /* Get the first entry. */
    p_node = slist_get(&radio_cb.data.tx.confirmed_list);

    if (((radio_data_wait_conf_entry_t *)p_node)->status == I15DOT4_STATUS_SUCCESS)
    {
        aError = OT_ERROR_NONE;
    }
    else if (((radio_data_wait_conf_entry_t *)p_node)->status == I15DOT4_STATUS_NO_ACK)
    {
        aError = OT_ERROR_NO_ACK;
    }

#if PACKET_STATISTICS

    if (aError == OT_ERROR_NONE)
        radio_cb.statistics.tx.status.success++;
    else if (aError == OT_ERROR_NO_ACK)
        radio_cb.statistics.tx.status.no_ack++;
    else
        radio_cb.statistics.tx.status.channel_access_failure++;

#endif // PACKET_STATISTICS

    /* Check if the ACK frame exist. */
    p_frame_ctrl = (I15DOT4_FRAME_CTRL_t *)((radio_data_wait_conf_entry_t *)p_node)->ackFrame;
    if (p_frame_ctrl->field.frame_type == I15DOT4_FRAME_TYPE_ACK)
    {
        ack_frame_included = WICED_TRUE;
        ack_frame.mLength  = I15DOT4_ACK_LENGTH;
        ack_frame.mPsdu    = ((radio_data_wait_conf_entry_t *)p_node)->ackFrame;
        ack_frame.mChannel = radio_cb.radio_state.channel;
    }

#if OPENTHREAD_CONFIG_DIAG_ENABLE

    if (otPlatDiagModeGet())
    {
        otPlatDiagRadioTransmitDone(((radio_data_wait_conf_entry_t *)p_node)->aInstance,
                                    ((radio_data_wait_conf_entry_t *)p_node)->aTxFrame, aError);
    }
    else
#endif /* OPENTHREAD_CONFIG_DIAG_ENABLE */
    {
        otPlatRadioTxDone(((radio_data_wait_conf_entry_t *)p_node)->aInstance,
                          ((radio_data_wait_conf_entry_t *)p_node)->aTxFrame, ack_frame_included ? &ack_frame : NULL,
                          aError);
    }

    /* Free memory. */
    wiced_platform_memory_free((void *)p_node);

    /* Set an event to application thread to process the queued confirmed entry. */
    if (slist_count(&radio_cb.data.tx.confirmed_list))
    {
        wiced_platform_application_thread_event_set(
            radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_THREAD_DATA_CONF));
    }
}

/*
 * THREAD-DATA.ind primitive handler
 */
static void radio_i15dot4_thread_data_ind_handler(void)
{
    slist_node_t *p_node;

    RADIO_TRACE("%s (%d)\n", __FUNCTION__, slist_count(&radio_cb.data.rx.frame_list));

    /* Check if there is received frame. */
    if (slist_count(&radio_cb.data.rx.frame_list) == 0)
    {
        return;
    }

#if PACKET_STATISTICS

    radio_cb.statistics.rx.num++;

#endif // PACKET_STATISTICS

    /* Get the first entry. */
    p_node = slist_get(&radio_cb.data.rx.frame_list);

    /* Inform thread stack for the received frame. */
    otPlatRadioReceiveDone(radio_cb.data.rx.aInstance, &((radio_data_received_frame_t *)p_node)->rxFrame,
                           OT_ERROR_NONE);

    /* Free memory. */
    wiced_platform_memory_free((void *)((radio_data_received_frame_t *)p_node)->rxFrame.mPsdu);
    wiced_platform_memory_free((void *)p_node);

    /* Set an application thread event to process the queued received frames. */
    if (slist_count(&radio_cb.data.rx.frame_list))
    {
        wiced_platform_application_thread_event_set(
            radio_i15dot4_application_thread_event_code_get(I15DOT4_CMDID_THREAD_DATA_IND));
    }
}

/*
 * Utility to display data.
 */
#if (RADIO_DEBUG)
static void radio_utils_data_display(uint8_t data_len, uint8_t *p_data)
{
    uint8_t i;

    printf("data (%d): ", data_len);
    for (i = 0; i < data_len; i++)
    {
        printf("0x%02X, ", p_data[i]);
    }
    printf("\n");
}

static void radio_utils_radio_frame_display(otRadioFrame *p_frame, wiced_bool_t tx)
{
    uint16_t i;

    if (tx)
    {
        printf("=> ");
    }
    else
    {
        printf("<= ");
    }

#if (OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)
    printf("PSDU (%d, %d): ", p_frame->mChannel, p_frame->mLength - I15DOT4_FCS_LENGTH);
#else  // (OPENTHREAD_CONFIG_THREAD_VERSION != OT_THREAD_VERSION_1_1)
    printf("PSDU (%d, %d, %d): ", p_frame->mChannel, p_frame->mRadioType, p_frame->mLength);
#endif // (OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)
    for (i = 0; i < (I15DOT4_FCS_LENGTH + RADIO_I15DOT4_SEQ_NUM_LENGTH); i++)
    {
        printf("0x%02x ", p_frame->mPsdu[i]);
    }
    printf("--- ");
    for (i = (p_frame->mLength - I15DOT4_FCS_LENGTH - 2); i < (p_frame->mLength - I15DOT4_FCS_LENGTH); i++)
    {
        printf("0x%02x ", p_frame->mPsdu[i]);
    }
    printf("\n");

    if (tx)
    {
        if (p_frame->mInfo.mTxInfo.mAesKey)
        {
            printf("AES Key: ");
            for (i = 0; i < OT_MAC_KEY_SIZE; i++)
            {
                printf("0x%02x ", p_frame->mInfo.mTxInfo.mAesKey->m8[i]);
            }
            printf("\n");
        }

        if (p_frame->mInfo.mTxInfo.mIeInfo)
        {
            printf("Header IE - networkTimeOffset: %" PRId64 " , TimeIeOffset: %d, TimeSyncSeq: %d\n",
                   p_frame->mInfo.mTxInfo.mIeInfo->mNetworkTimeOffset, p_frame->mInfo.mTxInfo.mIeInfo->mTimeIeOffset,
                   p_frame->mInfo.mTxInfo.mIeInfo->mTimeSyncSeq);
        }

#if 0  // OPENTHREAD_VERSION=master
        printf("TxDelay: %ld\n", p_frame->mInfo.mTxInfo.mTxDelay);
        printf("TxDelayBaseTime: %ld\n", p_frame->mInfo.mTxInfo.mTxDelayBaseTime);
#endif // OPENTHREAD_VERSION
        printf("MaxCsmaBackoffs: %d\n", p_frame->mInfo.mTxInfo.mMaxCsmaBackoffs);
        printf("MaxFrameRetries: %d\n", p_frame->mInfo.mTxInfo.mMaxFrameRetries);
        printf("IsARetx: %d\n", p_frame->mInfo.mTxInfo.mIsARetx);
        printf("CsmaCaEnabled: %d\n", p_frame->mInfo.mTxInfo.mCsmaCaEnabled);
#if 0  // OPENTHREAD_VERSION=master
        printf("CslPresent: %d\n", p_frame->mInfo.mTxInfo.mCslPresent);
#endif // OPENTHREAD_VERSION
        printf("IsSecurityProcessed: %d\n", p_frame->mInfo.mTxInfo.mIsSecurityProcessed);
    }
    else
    {
        printf("Timestamp: %" PRIu64 "\n", p_frame->mInfo.mRxInfo.mTimestamp);
        printf("AckFrameCounter: %ld\n", p_frame->mInfo.mRxInfo.mAckFrameCounter);
        printf("AckKeyId: %d\n", p_frame->mInfo.mRxInfo.mAckKeyId);
        printf("Rssi: %d\n", p_frame->mInfo.mRxInfo.mRssi);
        printf("Lqi: %d\n", p_frame->mInfo.mRxInfo.mLqi);
        printf("AckedWithFramePending: %d\n", p_frame->mInfo.mRxInfo.mAckedWithFramePending);
        printf("AckedWithSecEnhAck: %d\n", p_frame->mInfo.mRxInfo.mAckedWithSecEnhAck);
    }
}

static void radio_utils_i15dot4_thread_data_ind_display(I15DOT4_THREAD_DATA_IND_t *p_thread_data_ind)
{
    uint8_t i;

    printf("THREAD-DATA.ind (%d, %ld)\n", p_thread_data_ind->rssi, p_thread_data_ind->symbol_counter);
    printf("PSDU (%d): ", p_thread_data_ind->psdu_length);
    for (i = 0; i < (I15DOT4_FCS_LENGTH + RADIO_I15DOT4_SEQ_NUM_LENGTH); i++)
    {
        printf("0x%02X ", p_thread_data_ind->psdu[i]);
    }
    printf("--- ");
    for (i = (p_thread_data_ind->psdu_length - 2); i < p_thread_data_ind->psdu_length; i++)
    {
        printf("0x%02X ", p_thread_data_ind->psdu[i]);
    }
    printf("\n");
}

#else // RADIO_DEBUG
static void radio_utils_data_display(uint8_t data_len, uint8_t *p_data)
{
    OT_UNUSED_VARIABLE(data_len);
    OT_UNUSED_VARIABLE(p_data);
}

static void radio_utils_radio_frame_display(otRadioFrame *p_frame, wiced_bool_t tx)
{
    OT_UNUSED_VARIABLE(p_frame);
    OT_UNUSED_VARIABLE(tx);
}

static void radio_utils_i15dot4_thread_data_ind_display(I15DOT4_THREAD_DATA_IND_t *p_thread_data_ind)
{
    OT_UNUSED_VARIABLE(p_thread_data_ind);
}

#endif // RADIO_DEBUG

/*
 * Utility to calculate the ScanDuration attribute value used for MLME-SCAN.req
 * primitive which is defined in the IEEE 802.15.4 standard.
 */
static uint8_t radio_utils_scan_time_to_scan_duration_calculate(uint32_t scan_time, uint8_t channel_num)
{
    uint8_t  scan_duration = 0;
    uint32_t scan_time_for_each_channel;
    uint32_t symbol_count;
    uint32_t value;
    uint32_t tmp;

    /*
     * The time spent scanning each channel is:
     *      aBaseSuperframeDuration * (2 ^ n + 1) symbols
     * And 1) each symbol time is defined in OT_RADIO_SYMBOL_TIME (16 us/symbol),
     *     2) the aBaseSuperframeDuration is 960
     *
     * Hence,
     *      960 * (2 ^ n + 1) * OT_RADIO_SYMBOL_TIME * channel_num = scan_time (us)
     */
    scan_time_for_each_channel = scan_time / channel_num; // us
    symbol_count               = scan_time_for_each_channel / OT_RADIO_SYMBOL_TIME;
    value                      = symbol_count / 960;
    value                      = value - 1;

    tmp = value >> 1;
    while (tmp)
    {
        tmp = tmp >> 1;
        scan_duration++;
    }

    /* Round up if necessary. */
    if (value != ((uint32_t)1 << scan_duration))
    {
        scan_duration++;
    }

    return scan_duration;
}

/*
 * Load the IEEE EUI64 address.
 */
static void radio_ieee_eui64_load(void)
{
#if (RADIO_IEEE_EUI64_STORING_METHOD == RADIO_IEEE_EUI64_STORING_METHOD_NVRAM)

    uint16_t       nb_bytes;
    wiced_result_t status;
#if defined(I15DOT4_IEEE_EUI64)
    uint8_t *p_index;
    uint8_t  i;
#endif // I15DOT4_IEEE_EUI64

#if defined(I15DOT4_IEEE_EUI64)
    /* Check the assigned EUI64 address length. */
    if (strlen(I15DOT4_IEEE_EUI64) != (2 * sizeof(radio_cb.eui64)))
    {
        RADIO_TRACE("Err: Incorrect EUI64 address assigned\n");

        /* Generate an EUI64 address randomly. */
        otPlatEntropyGet((uint8_t *)&radio_cb.eui64, sizeof(radio_cb.eui64));
    }
    else
    {
        p_index = (uint8_t *)I15DOT4_IEEE_EUI64;

        for (i = 0; i < sizeof(radio_cb.eui64); i++)
        {
            radio_cb.eui64.m8[i] = (*p_index++ - '0') * 16;
            radio_cb.eui64.m8[i] += (*p_index++ - '0');
        }
    }

    /* Write EUI64 address to NVRAM. */
    wiced_hal_write_nvram(RADIO_IEEE_EUI64_NVRAM_ID, sizeof(radio_cb.eui64), (uint8_t *)&radio_cb.eui64, &status);
#else  // !defined(I15DOT4_IEEE_EUI64)
    /* Read EUI64 address from NVRAM. */
    nb_bytes = wiced_hal_read_nvram_static(RADIO_IEEE_EUI64_NVRAM_ID, sizeof(radio_cb.eui64), &radio_cb.eui64, &status);
    if ((nb_bytes != sizeof(radio_cb.eui64)) || (status != WICED_SUCCESS))
    { // Entry does not exist.
        printf("Error: Failed to read EUI64\n");
    }
#endif // defined(I15DOT4_IEEE_EUI64)

    RADIO_TRACE("EUI64: ");
    radio_utils_data_display(sizeof(radio_cb.eui64), (uint8_t *)&radio_cb.eui64);
    RADIO_TRACE("\n");

    radio_i15dot4_mlme_set_req(I15DOT4_MAC_EXTENDED_ADDRESS, sizeof(radio_cb.eui64), (uint8_t *)&radio_cb.eui64);

#endif // RADIO_IEEE_EUI64_STORING_METHOD == RADIO_IEEE_EUI64_STORING_METHOD_NVRAM
}
