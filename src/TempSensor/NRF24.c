#include "NRF24.h"
extern char* myRxData;

uint64_t RxpipeAddrs = 0x11223344AA;
char myAckPayload[32] = "ok";

void StartNRFTask(void const * argument) {
	char* myRxData = (char*)argument;
	//NRF init
	NRF24_begin(GPIOA, SPI1_CSN_Pin, SPI1_CE_Pin, hspi1);
	nrf24_DebugUART_Init(huart4);
	printRadioSettings();
	NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(32);
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();
	NRF24_startListening();

	for (;;)
	{
		//Read NRF
		if (NRF24_available())
		{
			NRF24_read(myRxData, 20);

			NRF24_writeAckPayload(1, myAckPayload, 32);
			HAL_UART_Transmit(&huart4, (uint8_t *)myRxData, 32 + 2, 10);
		}

		osDelay(1000);
	}
}