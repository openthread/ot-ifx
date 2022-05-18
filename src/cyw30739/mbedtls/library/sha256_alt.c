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
 *  The SHA-256 Secure Hash Standard was published by NIST in 2002.
 *
 *  http://csrc.nist.gov/publications/fips/fips180-2/fips180-2.pdf
 */

#include <wiced_rtos.h>
#include "mbedtls/error.h"
#include "mbedtls/sha256.h"

#if defined(MBEDTLS_SHA256_ALT)
#define mbedtls_calloc calloc
#define mbedtls_free free

#define SHA256_VALIDATE_RET(cond) MBEDTLS_INTERNAL_VALIDATE_RET(cond, MBEDTLS_ERR_SHA256_BAD_INPUT_DATA)
#define SHA256_VALIDATE(cond) MBEDTLS_INTERNAL_VALIDATE(cond)

#define SHA_IN_MAX_LENGTH 1500

void mbedtls_sha256_init(mbedtls_sha256_context *ctx)
{
    SHA256_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_sha256_context));
}

void mbedtls_sha256_free(mbedtls_sha256_context *ctx)
{
    if (ctx == NULL)
        return;

    free(ctx->p_sha_in);
    mbedtls_platform_zeroize(ctx, sizeof(mbedtls_sha256_context));
}

void mbedtls_sha256_clone(mbedtls_sha256_context *dst, const mbedtls_sha256_context *src)
{
    SHA256_VALIDATE(dst != NULL);
    SHA256_VALIDATE(src != NULL);

    mbedtls_sha256_init(dst);

    dst->p_sha_in = calloc(1, SHA_IN_MAX_LENGTH);
    if (dst->p_sha_in == NULL)
    {
        printf("Err: clone mem fail\n");
        return;
    }

    memcpy(dst->p_sha_in, src->p_sha_in, src->sha_in_len);
    dst->sha_in_len = src->sha_in_len;
    dst->sha2_mode  = src->sha2_mode;
}

/*
 * SHA-256 context setup
 */
int mbedtls_sha256_starts_ret(mbedtls_sha256_context *ctx, int is224)
{
    SHA256_VALIDATE_RET(ctx != NULL);
    SHA256_VALIDATE_RET(is224 == 0 || is224 == 1);

    if (is224 == 1)
        ctx->sha2_mode = HW_SHA224;
    else
        ctx->sha2_mode = HW_SHA256;

    if (ctx->p_sha_in == NULL)
    {
        ctx->p_sha_in = calloc(1, SHA_IN_MAX_LENGTH);
        if (ctx->p_sha_in == NULL)
        {
            printf("Err: alloc mem fail\n");
            return MBEDTLS_ERR_SHA256_HW_ACCEL_FAILED;
        }
    }
    ctx->sha_in_len = 0;
    memset(ctx->p_sha_in, 0, SHA_IN_MAX_LENGTH);
    return (0);
}

void mbedtls_sha256_starts(mbedtls_sha256_context *ctx, int is224)
{
    mbedtls_sha256_starts_ret(ctx, is224);
}

int mbedtls_internal_sha256_process(mbedtls_sha256_context *ctx, const unsigned char data[64])
{
    SHA256_VALIDATE_RET(ctx != NULL);
    SHA256_VALIDATE_RET((const unsigned char *)data != NULL);
    return 0;
}

void mbedtls_sha256_process(mbedtls_sha256_context *ctx, const unsigned char data[64])
{
    mbedtls_internal_sha256_process(ctx, data);
}

int mbedtls_sha256_update_ret(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen)
{
    SHA256_VALIDATE_RET(ctx != NULL);
    SHA256_VALIDATE_RET(ilen == 0 || input != NULL);
    SHA256_VALIDATE_RET(ctx->p_sha_in != NULL);

    if (ilen == 0)
        return (0);

    if ((ctx->sha_in_len + ilen) >= SHA_IN_MAX_LENGTH)
    {
        printf("Err: large sha_input\n");
        ctx->sha_in_len = SHA_IN_MAX_LENGTH;
        return MBEDTLS_ERR_SHA256_BAD_INPUT_DATA;
    }

    memcpy(ctx->p_sha_in + ctx->sha_in_len, input, ilen);
    ctx->sha_in_len += ilen;

    return (0);
}

void mbedtls_sha256_update(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen)
{
    mbedtls_sha256_update_ret(ctx, input, ilen);
}

/*
 * SHA-256 final digest
 */
int mbedtls_sha256_finish_ret(mbedtls_sha256_context *ctx, unsigned char output[32])
{
    uint8_t  sha_ret;
    tHW_SHA2 hw_sha256;

    SHA256_VALIDATE_RET(ctx != NULL);
    SHA256_VALIDATE_RET((unsigned char *)output != NULL);
    SHA256_VALIDATE_RET(ctx->p_sha_in != NULL);
    SHA256_VALIDATE_RET(ctx->sha_in_len != SHA_IN_MAX_LENGTH);

    hw_sha256.polling_flag = HW_SECENG_POLLING;
    hw_sha256.hmac_en      = 0;
    hw_sha256.sha2_mode    = ctx->sha2_mode;
    hw_sha256.msg_len      = ctx->sha_in_len;
    hw_sha256.key_len      = 0;
    hw_sha256.key_ptr      = NULL;
    hw_sha256.in_ptr       = (uint32_t *)ctx->p_sha_in;
    hw_sha256.out_ptr      = output;
    hw_sha256.callback     = NULL;

    sha_ret = hw_sha2_engine(&hw_sha256);

    if (sha_ret != HW_SHA2_COMPLETE)
    {
        printf("HW sha fail %d\n", sha_ret);
        return MBEDTLS_ERR_SHA256_HW_ACCEL_FAILED;
    }

    return (0);
}

void mbedtls_sha256_finish(mbedtls_sha256_context *ctx, unsigned char output[32])
{
    mbedtls_sha256_finish_ret(ctx, output);
}

#endif /* MBEDTLS_SHA256_ALT */
