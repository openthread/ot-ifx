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
/*
 **************************************************************************************************
 *
 * File Name:       ifx_ot_cli_cmd.c
 *
 * Abstract:        Infineon specific OpenThread CLI commands.
 *
 * Special Notices:
 *
 **************************************************************************************************
 */

//=================================================================================================
//  Includes
//=================================================================================================
#include <stdio.h>

#include <common/code_utils.hpp>
#include <openthread-core-config.h>
#include <openthread/cli.h>
#include <openthread/instance.h>

#if PACKET_STATISTICS
#include "packet_statistics.h"
#endif // PACKET_STATISTICS

//=================================================================================================
// Type Definitions and Enums
//=================================================================================================

//=================================================================================================
//  Structure
//=================================================================================================

//=================================================================================================
//  Global Variables
//=================================================================================================

//=================================================================================================
//  Declaration of Static Functions
//=================================================================================================
#if PACKET_STATISTICS
#if (OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)
static void ifx_ot_cli_cmd_handler_packet_statistics(uint8_t aArgsLength, char *aArgs[]);
#else  // OPENTHREAD_CONFIG_THREAD_VERSION != OT_THREAD_VERSION_1_1
static void ifx_ot_cli_cmd_handler_packet_statistics(void *aContext, uint8_t aArgsLength, char *aArgs[]);
#endif // OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1
#endif // PACKET_STATISTICS

//=================================================================================================
//  Static Variables
//=================================================================================================
static const otCliCommand ifx_ot_cli_commands[] = {
#if PACKET_STATISTICS
    {"ifx_packet_statistics", ifx_ot_cli_cmd_handler_packet_statistics},
#endif // PACKET_STATISTICS
};

//=================================================================================================
//	Global Functions
//=================================================================================================

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
void ifx_ot_cli_cmd_install(otInstance *ot_instance)
{
#if (OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)
    OT_UNUSED_VARIABLE(ot_instance);
    otCliSetUserCommands(ifx_ot_cli_commands, OT_ARRAY_LENGTH(ifx_ot_cli_commands));
#else
    otCliSetUserCommands(ifx_ot_cli_commands, OT_ARRAY_LENGTH(ifx_ot_cli_commands), ot_instance);
#endif
}

//=================================================================================================
//	Local (Static) Functions
//=================================================================================================
#if PACKET_STATISTICS
/**************************************************************************************************
 * Function:     ifx_ot_cli_cmd_handler_packet_statistics
 *
 * Abstract:     Infineon specific OpenThread CLI command handler for ifx_packet_statistics.
 *
 * Input/Output:
 *   void *aContext (I)      - instance
 *   uint8_t aArgsLength (I) - number of input arguments
 *   char *aArgs[]           - arguments
 *
 * Return:       None
 *
 * Notices:
 **************************************************************************************************/
#if (OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)
static void ifx_ot_cli_cmd_handler_packet_statistics(uint8_t aArgsLength, char *aArgs[])
#else
static void ifx_ot_cli_cmd_handler_packet_statistics(void *aContext, uint8_t aArgsLength, char *aArgs[])
#endif
{
    PACKET_STATISTICS_t statistics = {0};

#if !(OPENTHREAD_CONFIG_THREAD_VERSION == OT_THREAD_VERSION_1_1)
    OT_UNUSED_VARIABLE(aContext);
#endif
    OT_UNUSED_VARIABLE(aArgsLength);
    OT_UNUSED_VARIABLE(aArgs);

    /* Collect information. */
    otPlatRadioPacketStatisticsGet(&statistics);

    /* Send result to host device. */
    otCliOutputFormat("Tx. - total: %ld, success: %ld, no_ack: %ld, channel_access_failure: %ld\n", statistics.tx.num,
                      statistics.tx.status.success, statistics.tx.status.no_ack,
                      statistics.tx.status.channel_access_failure);
    otCliOutputFormat("Rx. - total: %ld\n", statistics.rx.num);

    otCliAppendResult(OT_ERROR_NONE);
}
#endif // PACKET_STATISTICS

//=================================================================================================
//	End of File (ifx_ot_cli_cmd.c)
//=================================================================================================
