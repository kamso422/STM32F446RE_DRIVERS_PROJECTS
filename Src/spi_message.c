/*
 * spi_message.c
 *
 *  Created on: Aug 9, 2025
 *      Author: chikamso
 */

#include "STM32F44xx__driver.h"
#include "STM32F44xxgpio__driver.h"
#include "STM32F44xxSPI__driver.h"
#include <string.h>
#include <stdio.h>
#define MAX_LEN = 500;


int main(void){
	gpio_handle_t signalPin;
	gpio_handle_t spiPins;
	SPI_Handle_t theSpi;

	memset(&signalPin,0,sizeof(gpio_handle_t));
	memset(&spiPins,0,sizeof(gpio_handle_t));
	memset(&theSpi,0,sizeof(SPI_Handle_t));

	GPIO_PeriClockControl(GPIOA, ENABLE);

	//setup the signal pin
	signalPin.gpioReg = GPIOA;
	signalPin.gpioConfigure.MODE = GPIO_MODE_INPUT;
	signalPin.gpioConfigure.OSPEED = GPIO_LOW_SPEED;
	signalPin.gpioConfigure.OTYPE = GPIO_PP;
	signalPin.gpioConfigure.PUPD = GPIO_PD;
	signalPin.gpioConfigure.pinNumber = GPIO_PIN_NO_8;

	gpioInit(&signalPin);

	//Setting up the SPI pins
	spiPins.gpioReg = GPIOA;
	spiPins.gpioConfigure.ALTF = AF5;
	spiPins.gpioConfigure.MODE = GPIO_MODE_ALFN;
	spiPins.gpioConfigure.OSPEED = GPIO_LOW_SPEED;
	spiPins.gpioConfigure.OTYPE = GPIO_PP;
	spiPins.gpioConfigure.PUPD = GPIO_NO_PUPD;

	//the MOSI line
	spiPins.gpioConfigure.pinNumber = GPIO_PIN_NO_7;
	gpioInit(&spiPins);

	//the MISO line
	spiPins.gpioConfigure.pinNumber = GPIO_PIN_NO_6;
	gpioInit(&spiPins);

	//the SCK line
	spiPins.gpioConfigure.pinNumber = GPIO_PIN_NO_5;
	gpioInit(&spiPins);

	//the NSS line
	spiPins.gpioConfigure.pinNumber = GPIO_PIN_NO_4;
	gpioInit(&spiPins);

	theSpi.pSPIx = SPI1;
	theSpi.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	theSpi.SPIConfig.SPI_CPHA = SPI_CPHA1;
	theSpi.SPIConfig.SPI_CPOL = SPI_CPOL0;
	theSpi.SPIConfig.SPI_DFF = DFF8;
	theSpi.SPIConfig.SPI_DeviceMode = MASTER;
	theSpi.SPIConfig.SPI_SSM = SSM_HW;
	theSpi.SPIConfig.SPI_SclkSpeed = SPI_CLK_SPEED_DIV128;


	SPI_PeriClockControl(SPI1,ENABLE);
	SPI_IRQConfig(SPI1_IRQ_NUM, ENABLE);
	spiInit(&theSpi);

	uint8_t length = 0;
	uint8_t *message = NULL;
	uint8_t dummyWrite = 0xFF;
	uint8_t dummyRead = 0;

	while(!GPIO_ReadFromInputPin(signalPin.gpioReg, signalPin.gpioConfigure.pinNumber));

	spiEnable(SPI1,ENABLE);

	//SPI_SendDataIT(&theSpi,&dummyWrite,1);
	SPI_ReceiveDataIT(&theSpi,&length,1);

	//SPI_SendDataIT(&theSpi,&dummyWrite,1);
	SPI_ReceiveDataIT(&theSpi,message,length);

	spiEnable(SPI1,DISABLE);

	for(uint8_t i = 0; i < length; i++){
		printf("%c",message[i]);
	}

}
