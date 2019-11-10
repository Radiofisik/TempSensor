---
title: STM32 подключение экрана и датчика температуры
description: использование RTOS, CubeMx, VisualGDB
---

Для начала работы необходимо:

- установить Visual Studio 2017 c поддержкой С++ ( необходим VC++ 2017 toolset см https://visualgdb.com/support/getvcpp/).
- установить VisualGDB (https://visualgdb.com/)
- установить СubeMx (https://www.st.com/en/development-tools/stm32cubemx.html)

Создадим проект в CubeMx на основании модели контроллера STM32F407VG. В данном случае и платы STM32F4DISCOVERY.

Для подключения дисплея будем использовать I2C1. Включим его 

![i2c1](pict/i2c1.png)

В экране используется микросхема PCF8574  которая поддерживает только 100kGz режим

![i2c1params](pict/i2c1params.png)

Настроим DMA

![i2cdma](pict/i2cdma.png)

В настойках GPIO видно к каким портам цеплять экран

![i2cgpi0](pict/i2cgpi0.png)

Включим прерывания

![i2c1int](pict/i2c1int.png)

Для подключения экрана будем использовать библиотеку http://blog.bulki.me/STM32-LCD-HD44780-I2C/ https://github.com/firebull/STM32-LCD-HD44780-I2C которая использует функцию vTaskDelayUntil() из FreeRTOS. Включим ее.

![delayuntil](pict/delayuntil.png)

Настраиваем выгрузку

![generate](pict/generate.png)

и жмем generate. Далее запускаем Visual Studio 2017. создаем проект в той же папке

![vc1](pict/vc1.png)

Импортируем проект

![vc2](pict/vc2.png)

Выбираем контроллер

![vc3](pict/vc3.png)

выбираем отладчик, предварительно подключив его

![vc4](pict/vc4.png)

Добавим в папки и подключим в проект файлы библиотеки для LCD.

изменим код задачи по умолчанию

```c++
void StartDefaultTask(void const * argument)
{
   /* USER CODE BEGIN StartDefaultTask */
	lcdInit(&hi2c1, (uint8_t)0x27, (uint8_t)4, (uint8_t)20);
    
	// Print text and home position 0,0
	lcdPrintStr((uint8_t*)"Hello,", 6);
    
	// Set cursor at zero position of line 3
	lcdSetCursorPosition(0, 2);

	// Print text at cursor position
	lcdPrintStr((uint8_t*)"World!", 6);

	for (;;) {
		vTaskDelay(1000);
	}
}
```

После запуска получим долгожданный hello world на экране

##  DHT22

определимся с портом к которым будем подключать DHT22 пусть это будет PE6. в cube настроим этот порт как outpt. Добавим в проект файлы библиотеки [библиотеки](https://github.com/MYaqoobEmbedded/STM32-Tutorials/tree/master/Tutorial 25 - DHT22 Temperature Sensor). 



> To be continued...