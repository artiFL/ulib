#include "main.h"

u8 new_array [100];


int main () {

	HSE_Enable();
	UASRT_init(ON, 115200);
	indication_led_init(ON, BLUE);

	AES_XTSinit();


	//random_gen(new_array, 40);
    //send_array(new_array, 40);


	for(ever) {

		if (flag_get_array_byte != 0) {

			STM32_AES_XTS_Encrypt(bufferRessive, sizeof(bufferRessive), Key, CRL_AES128_KEY, SequenceNumber, temp);

			send_array(temp);

			print_str("////////////////\n");

			STM32_AES_XTS_Decrypt(temp, sizeof(temp), Key, CRL_AES128_KEY, SequenceNumber, new_array);

			send_array(new_array);

			print_str("////////////////\n");

			flag_get_array_byte = !1;

		}





	}
}



