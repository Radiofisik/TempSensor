#include "Display.h"

static void DisplayInit(I2C_HandleTypeDef * i2c)
{
	lcdInit(i2c, (uint8_t)0x27, (uint8_t)4, (uint8_t)20);
    
	// Print text and home position 0,0
	char * initString = "Temperature sensor";
	
	lcdPrintStr((uint8_t*)initString, strlen(initString));
}

static void PrintTemp(float temp)
{
	char str[20];
	char *tmpSign = (temp < 0) ? "-" : "";
	float tmpVal = (temp < 0) ? -temp : temp;
	int tmpInt1 = tmpVal;                  
	float tmpFrac = tmpVal - tmpInt1;      
	int tmpInt2 = trunc(tmpFrac * 10); 
	
	sprintf(str, "Temperature = %s%d.%01d", tmpSign, tmpInt1, tmpInt2);
	
	lcdSetCursorPosition(0, 1);
	
	lcdPrintStr((uint8_t *)str, strlen(str));
}

static void PrintHumidity(float humidity)
{
	char str[20];
	char *tmpSign = (humidity < 0) ? "-" : "";
	float tmpVal = (humidity < 0) ? -humidity : humidity;
	int tmpInt1 = tmpVal;                  
	float tmpFrac = tmpVal - tmpInt1;      
	int tmpInt2 = trunc(tmpFrac * 10); 
	
	sprintf(str, "Humidity = %s%d.%01d", tmpSign, tmpInt1, tmpInt2);
				
	lcdSetCursorPosition(0, 2);
	
	lcdPrintStr((uint8_t *)str, strlen(str));
}

void StartDisplayTask(void const * argument)
{
	DisplayStruct* ds = (DisplayStruct *) argument;

	DisplayInit(ds->i2c);
	
	for (;;)
	{
		PrintTemp(*ds->temperature);
		PrintHumidity(*ds->humidity);
		vTaskDelay(1000);
	}
}