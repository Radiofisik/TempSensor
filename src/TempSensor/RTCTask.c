#include "RTCTask.h"

void StartRTCTask(void const * argument) {
	char * Time = (char*)argument;
	osDelay(1000);
	//Init RTC
	uint8_t dc3231Addr = (uint8_t)0xD0;
	DS3231_sendData(hi2c2, dc3231Addr);
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
	}

	//Read
	//	I2C_ReadCalendarData(hi2c2, dc3231Addr);   
	//    setHour(0x20);
	//    setMinutes(0x24);
	//    setSeconds(00);
	//    DS3231_setDate(hi2c2, dc3231Addr);  //call to update set data

	for (;;)
	{
		//Read RTC
		DS3231_sendData(hi2c2, dc3231Addr);
		while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
		}
		I2C_ReadCalendarData(hi2c2, dc3231Addr);
		sprintf(Time, "%s:%s:%s", readHours(), readMinutes(), readSeconds());

		osDelay(1000);
	}
}