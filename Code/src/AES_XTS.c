#include "AES_XTS.h"


/**
  * @brief  AES-128-XTS Encryption
  * @param  InputMessage: pointer to input message to be encrypted
  * @param  InputMessageLength: input data message length in byte
  * @param  AES_Key: pointer to the AES key to be used in the operation
  * @param  KeyLength: length of the AES key
  * @param  Tweak: Initialization vector
  * @param  OutputMessage: pointer to output parameter that will handle the encrypted message
  * @retval error status: can be AES_SUCCESS if success or one of
  *         AES_ERR_BAD_INPUT_SIZE, AES_ERR_BAD_OPERATION, AES_ERR_BAD_CONTEXT
  *         AES_ERR_BAD_PARAMETER if error occured.
  */
void AES_XTSinit (void) {

	RCC -> AHB1ENR |= RCC_AHB1ENR_CRCEN;	// Обязательно

	SCB_EnableICache();
	SCB_EnableDCache();

}

int32_t STM32_AES_XTS_Encrypt(const uint8_t *InputMessage,
                              uint32_t   InputMessageLength,
                              const uint8_t *AES_Key,
                              uint32_t   KeyLength,
                              const uint8_t *Tweak,
                              uint8_t   *OutputMessage)
{
  /* AES context, error status and output length */

  uint32_t error_status = AES_SUCCESS;
  int32_t outputLength = 0;

  /* Set flag field to default value */
  AESctx.mFlags = E_SK_DEFAULT;
  /* Set the key size in AES status */
  AESctx.mKeySize = KeyLength;
  /* Set the Tweak size in AES status */
  AESctx.mTweakSize = CRL_AES_BLOCK;
  /* Initialize the operation, by passing key and Tweak */
  error_status = AES_XTS_Encrypt_Init(&AESctx, AES_Key, Tweak);

  /* check for initialization errors */
  if (error_status == AES_SUCCESS)
  {
    /* Encrypt Data in XTS mode */
    error_status = AES_XTS_Encrypt_Append(&AESctx, InputMessage,
                                          InputMessageLength, OutputMessage, &outputLength);

    /* check for encryption errors in XTS mode */
    if (error_status == AES_SUCCESS)
    {
      /* Finalize data */
      error_status = AES_XTS_Encrypt_Finish(&AESctx, OutputMessage,
                                            &outputLength);
    }
  }

  return error_status;
}


/**
  * @brief  AES-128-XTS Decryption
  * @param  InputMessage: pointer to input message to be decrypted
  * @param  InputMessageLength: input data message length in byte
  * @param  AES_Key: pointer to the AES key to be used in the operation
  * @param  KeyLength: length of the AES key
  * @param  Tweak: Initialization vector
  * @param  OutputMessage: pointer to output parameter that will handle the decrypted message
  * @retval error status: can be AES_SUCCESS if success or one of
  *         AES_ERR_BAD_INPUT_SIZE, AES_ERR_BAD_OPERATION, AES_ERR_BAD_CONTEXT
  *         AES_ERR_BAD_PARAMETER if error occured.
  */
int32_t STM32_AES_XTS_Decrypt(const uint8_t *InputMessage,
                              uint32_t   InputMessageLength,
                              const uint8_t *AES_Key,
                              uint32_t   KeyLength,
                              const uint8_t *Tweak,
                              uint8_t   *OutputMessage)
{

  uint32_t error_status = AES_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  AESctx.mFlags = E_SK_DEFAULT;
  /* Set key size in AES status */
  AESctx.mKeySize = KeyLength;
  /* Set the Tweak size in AES status */
  AESctx.mTweakSize = CRL_AES_BLOCK;
  /* Initialize the operation, by passing the key and Tweak */
  error_status = AES_XTS_Decrypt_Init(&AESctx, AES_Key, Tweak);

  /* check for initialization errors */
  if (error_status == AES_SUCCESS)
  {
    /* Decrypt Data in XTS mode */
    error_status = AES_XTS_Decrypt_Append(&AESctx, InputMessage,
                                          InputMessageLength, OutputMessage, &outputLength);

    /* check for decryption errors in XTS mode */
    if (error_status == AES_SUCCESS)
    {
      /* Finalize data */
      error_status = AES_XTS_Decrypt_Finish(&AESctx, OutputMessage,
                                            &outputLength);
    }
  }

  return error_status;
}

TestStatus Buffercmp(const uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer != *pBuffer1)
    {
      return FAILED;
    }

    pBuffer++;
    pBuffer1++;
  }

  return PASSED;
}
