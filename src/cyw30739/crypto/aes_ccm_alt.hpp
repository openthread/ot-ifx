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
 *   This file includes definitions for performing AES-CCM computations.
 */

#ifndef AES_CCM_ALT_HPP_
#define AES_CCM_ALT_HPP_

#include <stdint.h>
#include "crypto/aes_ccm.hpp"
#include "mbedtls/ccm.h"

/**
 * @addtogroup core-security
 *
 * @{
 *
 */

/**
 * This class implements AES CCM computation.
 *
 */
enum
{
    kMinTagLength = 4,  ///< Minimum tag length (in bytes).
    kNonceSize    = 13, ///< Size of IEEE 802.15.4 Nonce (in bytes).
};

/**
 * This enumeration type represent the encryption vs decryption mode.
 *
 */
enum Mode
{
    kEncrypt, // Encryption mode.
    kDecrypt, // Decryption mode.
};

/**
 * This structure type represent the context that platform-ccm needs.
 *
 */
typedef struct
{
    mbedtls_ccm_context                   mMbed_ccm_ctx;
    uint32_t                              mInputTextLength;
    uint8_t                               mNonceLength;
    uint8_t                              *mNoncePtr;
    uint8_t                               mAadLength;
    uint8_t                              *mAadPtr;
    uint8_t                               mAadCurLength;
    uint8_t                               mTagLength;
    uint32_t                              mTag[4];
    ot::Crypto::AesCcm::aesccm_context_t *mOriginal_ctx;
} platform_ccm_ctx_t;

/**
 * @}
 *
 */
#endif // AES_CCM_ALT_HPP_
