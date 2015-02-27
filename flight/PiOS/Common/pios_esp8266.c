/*
 * pios_esp8266.c
 *
 *  Created on: 12 lut 2015
 *      Author: Bobasek
 */

#include "pios_esp8266.h"
#include "pios.h"
#include "pios_thread.h"


void PIOS_ESP8266_ResetPin_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // Use the alternative pin functions
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // GPIO speed - has nothing to do with the timer timing
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // Setup pull-up resistors
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}


void PIOS_ESP8266_Reset()
{

	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);

	PIOS_Thread_Sleep(100);

    GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
}
