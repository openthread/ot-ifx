/*
 *  Copyright (c) 2016, The OpenThread Authors.
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
 *   This file implements AES-CCM using mbedtls CCM.
 */

#include "aes_ccm.hpp"

#if defined(OPENTHREAD_AESCCM_ALT)

#include <limits.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/encoding.hpp"
#include "mbedtls/ccm.h"

#define aes_ccm_calloc calloc
#define aes_ccm_free free

namespace ot {
namespace Crypto {

void AesCcm::SetKey(const uint8_t *aKey, uint16_t aKeyLength)
{
    mbedtls_ccm_setkey(&mContext, MBEDTLS_CIPHER_ID_AES, aKey, aKeyLength << 3);
}

void AesCcm::SetKey(const Mac::Key &aMacKey)
{
    mbedtls_ccm_setkey(&mContext, MBEDTLS_CIPHER_ID_AES, aMacKey.GetKey(), Mac::Key::kSize << 3);
}

void AesCcm::Init(uint32_t    aHeaderLength,
                  uint32_t    aPlainTextLength,
                  uint8_t     aTagLength,
                  const void *aNonce,
                  uint8_t     aNonceLength)
{
    mAadLength       = aHeaderLength;
    mAadCurLength    = 0;
    mNonceLength     = aNonceLength;
    mNoncePtr        = (uint8_t *)aNonce;
    mInputTextLength = aPlainTextLength;
    mTagLength       = aTagLength;
    mResult          = kCcmInitFailure;
    mAadPtr          = NULL;

    mbedtls_ccm_init(&mContext);
    mAadPtr = (uint8_t *)aes_ccm_calloc(1, mAadLength);
    if (mAadPtr == NULL)
        goto outofmemory;

    return;

outofmemory:
    mResult = kCcmOutOfMemory;
}

void AesCcm::Header(const void *aHeader, uint32_t aHeaderLength)
{
    if (mResult != kCcmInitFailure)
        return;

    if ((mAadCurLength + aHeaderLength) <= mAadLength)
    {
        memcpy(mAadPtr + mAadCurLength, aHeader, aHeaderLength);
        mAadCurLength += aHeaderLength;
    }
    else
    {
        mResult = kCcmAadLenMismatch;
    }
}

void AesCcm::Payload(void *aPlainText, void *aCipherText, uint32_t aLength, Mode aMode)
{
    int ret;

    if ((mResult != kCcmInitFailure) && (mResult != kCcmSuccess))
        return;

    if (aLength != mInputTextLength)
    {
        mResult = kCcmPayloadLenMismatch;
        return;
    }

    switch (aMode)
    {
    case kEncrypt:
        ret = mbedtls_ccm_encrypt_and_tag(&mContext, mInputTextLength, mNoncePtr, mNonceLength, mAadPtr, mAadLength,
                                          (uint8_t *)aPlainText, (uint8_t *)aCipherText, (uint8_t *)mTag, mTagLength);
        break;
    case kDecrypt:
        ret = mbedtls_ccm_auth_decrypt(&mContext, mInputTextLength, mNoncePtr, mNonceLength, mAadPtr, mAadLength,
                                       (uint8_t *)aCipherText, (uint8_t *)aPlainText, (uint8_t *)mTag, mTagLength);
        break;
    default:
        mResult = kCcmBadInput;
        return;
    }

    if (ret == 0)
        mResult = kCcmSuccess;
    else if (ret == MBEDTLS_ERR_CCM_AUTH_FAILED)
        mResult = kCcmAuthFail;
    else if (ret == MBEDTLS_ERR_CCM_BAD_INPUT)
        mResult = kCcmBadInput;
}

void AesCcm::Finalize(void *aTag)
{
    if (mResult == kCcmSuccess)
        memcpy(aTag, mTag, mTagLength);

    if (mResult != kCcmOutOfMemory)
        aes_ccm_free(mAadPtr);

    if (mResult != kCcmSuccess)
        printf("AesCcmAlt result %d\n", mResult);
}

void AesCcm::GenerateNonce(const Mac::ExtAddress &aAddress,
                           uint32_t               aFrameCounter,
                           uint8_t                aSecurityLevel,
                           uint8_t *              aNonce)
{
    memcpy(aNonce, aAddress.m8, sizeof(Mac::ExtAddress));
    aNonce += sizeof(Mac::ExtAddress);

    Encoding::BigEndian::WriteUint32(aFrameCounter, aNonce);
    aNonce += sizeof(uint32_t);

    aNonce[0] = aSecurityLevel;
}

} // namespace Crypto
} // namespace ot

#endif
