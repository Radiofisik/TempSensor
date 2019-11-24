#include "Sensor.h"

void StartSensorTask(void const * argument)
{
	SensorStruct * ss = (SensorStruct *) argument;
	
	DHT22_Init(GPIOE, GPIO_PIN_6);
	
	for (;;) {
		
		taskENTER_CRITICAL();
		if (DHT22_GetTemp_Humidity(ss->temperature, ss->humidity) == 0)
		{
			ss->temperature = 0;
		}
		taskEXIT_CRITICAL();

		vTaskDelay(1000);
	}
}