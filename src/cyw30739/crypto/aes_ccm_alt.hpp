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

#include "openthread-core-config.h"

#include <stdint.h>

#include "ccm.h"
#include "crypto/aes_ecb.hpp"
#include "mac/mac_types.hpp"
#include "openthread/error.h"

namespace ot {
namespace Crypto {

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
class AesCcm
{
public:
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
     * This enumeration type represent the result of aes ccm.
     *
     */
    enum
    {
        kCcmInitFailure = 0,    // default result
        kCcmOutOfMemory,        // out of memory
        kCcmAadLenMismatch,     // aad length is different between Aesccm.Init and Aesccm.Header
        kCcmPayloadLenMismatch, // payload length is different between Aesccm.Init and Aesccm.Payload
        kCcmSuccess,            // success
        kCcmAuthFail,           // mic error reported from hw aes engine
        kCcmBadInput,           // the input hw aes engine cannot accept
    };

    /**
     * This method sets the key.
     *
     * @param[in]  aKey        A pointer to the key.
     * @param[in]  aKeyLength  Length of the key in bytes.
     *
     */
    void SetKey(const uint8_t *aKey, uint16_t aKeyLength);

    /**
     * This method sets the key.
     *
     * @param[in]  aMacKey        A MAC key.
     *
     */
    void SetKey(const Mac::Key &aMacKey);

    /**
     * This method initializes the AES CCM computation.
     *
     * @param[in]  aHeaderLength     Length of header in bytes.
     * @param[in]  aPlainTextLength  Length of plaintext in bytes.
     * @param[in]  aTagLength        Length of tag in bytes (must be even and in `[kMinTagLength, kMaxTagLength]`).
     * @param[in]  aNonce            A pointer to the nonce.
     * @param[in]  aNonceLength      Length of nonce in bytes.
     *
     */
    void Init(uint32_t    aHeaderLength,
              uint32_t    aPlainTextLength,
              uint8_t     aTagLength,
              const void *aNonce,
              uint8_t     aNonceLength);

    /**
     * This method processes the header.
     *
     * @param[in]  aHeader        A pointer to the header.
     * @param[in]  aHeaderLength  Length of header in bytes.
     *
     */
    void Header(const void *aHeader, uint32_t aHeaderLength);

    /**
     * This method processes the payload.
     *
     * @param[inout]  aPlainText   A pointer to the plaintext.
     * @param[inout]  aCipherText  A pointer to the ciphertext.
     * @param[in]     aLength      Payload length in bytes.
     * @param[in]     aMode        Mode to indicate whether to encrypt (`kEncrypt`) or decrypt (`kDecrypt`).
     *
     */
    void Payload(void *aPlainText, void *aCipherText, uint32_t aLength, Mode aMode);

    /**
     * This method returns the tag length in bytes.
     *
     * @returns The tag length in bytes.
     *
     */
    uint8_t GetTagLength(void) const { return mTagLength; }

    /**
     * This method generates the tag.
     *
     * @param[out]  aTag        A pointer to the tag (must have `GetTagLength()` bytes).
     *
     */
    void Finalize(void *aTag);

    /**
     * This static method generates IEEE 802.15.4 nonce byte sequence.
     *
     * @param[in]  aAddress        An extended address.
     * @param[in]  aFrameCounter   A frame counter.
     * @param[in]  aSecurityLevel  A security level.
     * @param[out] aNonce          A buffer (with `kNonceSize` bytes) to place the generated nonce.
     *
     */
    static void GenerateNonce(const Mac::ExtAddress &aAddress,
                              uint32_t               aFrameCounter,
                              uint8_t                aSecurityLevel,
                              uint8_t *              aNonce);

private:
    mbedtls_ccm_context mContext;
    uint32_t            mInputTextLength;
    uint8_t             mNonceLength;
    uint8_t *           mNoncePtr;
    uint8_t             mAadLength;
    uint8_t *           mAadPtr;
    uint8_t             mAadCurLength;
    uint8_t             mTagLength;
    uint32_t            mTag[4];
    uint8_t             mResult;
};

/**
 * @}
 *
 */

} // namespace Crypto
} // namespace ot

#endif // AES_CCM_ALT_HPP_
