/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    FRDMKL02Z_Project_UART.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL02Z4.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 4000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

#include "sdk_hal_uart0.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
	gpio_pin_config_t led_config = {
	        kGPIO_DigitalOutput, 0,
	    };
	BOARD_InitPins();
	BOARD_BootClockRUN();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    GPIO_PinInit(GPIOB, 6U, &led_config);

    //Configura LED VERDE (PTB7) como salida
    GPIO_PinInit(GPIOB, 7U, &led_config);

    //Configura LED AZUL (PTB10) como salida
    GPIO_PinInit(GPIOB, 10U, &led_config);
    //Se asegura que los LED inicien apagados
    GPIO_PortSet(GPIOB, 1u << 10U);
    GPIO_PortSet(GPIOB, 1u << 7U);
    GPIO_PortSet(GPIOB, 1u << 60U);
    (void)uart0Inicializar(115200);

    while(1) {
    	status_t status;
    	uint8_t nuevo_byte;
    	//Se verifica que haya datos en el buffer
    	if(uart0NuevosDatosEnBuffer()>0){
    		//Se actualiza el status y el valor del nuevo byte
    		status=uart0LeerByteDesdeBufferCircular(&nuevo_byte);
    		if(status==kStatus_Success){
    			//Se muestra en consola el valor del nuevo byte si el estado es correcto
    			printf("dato:%c\r\n",nuevo_byte);
    		}else{
    			//En caso de tener un error se notifica en consola
    			printf("error\r\n");
    		}
    	}
    	//Se usa un switch case para manejar el LED que se vaya a encender o apagar dependiendo de la entrada
    	switch(nuevo_byte){
    	case 'A':
    		//Sacar 0 por pin LED AZUL (enciende)
    		GPIO_PortClear(GPIOB, 1u << 10U);
    		delay();
    		break;
    	case 'a':
    		//Sacar 1 por pin LED AZUL (apaga)
    		GPIO_PortSet(GPIOB, 1u << 10U);
    		delay();
    		break;
    	case 'V':
    		GPIO_PortClear(GPIOB, 1u << 7U);
    		delay();
    		break;
    	case 'v':
    	    GPIO_PortSet(GPIOB, 1u << 7U);
    	    delay();
    	    break;
    	case 'R':
    	    GPIO_PortClear(GPIOB, 1u << 6U);
    	    delay();
    	    break;
    	case 'r':
    	    GPIO_PortSet(GPIOB, 1u << 6U);
    	    delay();
    	    break;
    	}
    }
    return 0 ;
}
