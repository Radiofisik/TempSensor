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

