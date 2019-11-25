#include "Display.h"
#include "math.h"
#include "string.h"
#include "stdio.h"

static void DisplayInit(I2C_HandleTypeDef * i2c)
{
	lcdInit(i2c, (uint8_t)0x27, (uint8_t)4, (uint8_t)20);
    
	// Print text and home position 0,0
	char * initString = "Temperature sensor";
	
	lcdPrintStr((uint8_t*)initString, strlen(initString));
}

static void ToString(char* str, float num)
{
	char *tmpSign = (num < 0) ? "-" : "";
	float tmpVal = (num < 0) ? -num : num;
	int tmpInt1 = tmpVal;                  
	float tmpFrac = tmpVal - tmpInt1;      
	int tmpInt2 = trunc(tmpFrac * 10); 
	sprintf(str, "%s%d.%01d", tmpSign, tmpInt1, tmpInt2);
}

static void PrintTemp(float temp)
{
	char numstr[5];
	char str[20];
	ToString(numstr, temp);
	sprintf(str, "Ti = %s", numstr);
	
	lcdSetCursorPosition(0, 1);	
	lcdPrintStr((uint8_t *)str, strlen(str));
}

static void PrintHumidity(float humidity)
{
	char numstr[5];
	char str[20];
	ToString(numstr, humidity);
	sprintf(str, "Hi = %s", numstr);
				
	lcdSetCursorPosition(10, 1);
	lcdPrintStr((uint8_t *)str, strlen(str));
}

static void PrintTime(char * time)
{
	lcdSetCursorPosition(0, 3);
	lcdPrintStr((uint8_t *)time, strlen(time));
}

static void PrintOutside(char * time)
{
	lcdSetCursorPosition(0, 2);
	lcdPrintStr((uint8_t *)time, strlen(time));
}

void StartDisplayTask(void const * argument)
{
	DisplayStruct* ds = (DisplayStruct *) argument;

	DisplayInit(ds->i2c);
	
	for (;;)
	{
		PrintTemp(*ds->temperature);
		PrintHumidity(*ds->humidity);
		PrintTime(ds->time);
		PrintOutside(ds->outside);
		vTaskDelay(1000);
	}
}