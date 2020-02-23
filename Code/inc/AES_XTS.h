#include "crypto.h"

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

int32_t check;
uint8_t temp[512]; /* temporary buffer to store result */

AESXTSctx_stt AESctx;

/* Private function prototypes -----------------------------------------------*/

void AES_XTSinit (void);

int32_t STM32_AES_XTS_Encrypt(const uint8_t *InputMessage,
                              uint32_t   InputMessageLength,
                              const uint8_t *AES_Key,
                              uint32_t   KeyLength,
                              const uint8_t *Tweak,
                              uint8_t   *OutputMessage);

int32_t STM32_AES_XTS_Decrypt(const uint8_t *InputMessage,
                              uint32_t   InputMessageLength,
                              const uint8_t *AES_Key,
                              uint32_t   KeyLength,
                              const uint8_t *Tweak,
                              uint8_t   *OutputMessage);

TestStatus Buffercmp(const uint8_t* pBuffer,
                     uint8_t* pBuffer1,
                     uint16_t BufferLength);

