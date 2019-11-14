#include "lcd_hd44780_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "strings.h"
#include "stdio.h"

typedef struct
{
	I2C_HandleTypeDef * i2c;
	float * temperature;
	float * humidity;
	char* time;
}DisplayStruct;

static void DisplayInit(I2C_HandleTypeDef * i2c);

static void PrintHumidity(float humidity);

static void PrintTemp(float temp);

void StartDisplayTask(void const * argument);