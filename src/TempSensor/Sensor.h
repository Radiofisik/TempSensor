#include "FreeRTOS.h"
#include "task.h"
#include "MY_DHT22.h"

typedef struct
{
	float * temperature;
	float * humidity;
}SensorStruct;

void StartSensorTask(void const * argument);