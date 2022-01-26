/*
 *  Copyright (c) 2021, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the Infineon specific OpenThread CLI commands.
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
