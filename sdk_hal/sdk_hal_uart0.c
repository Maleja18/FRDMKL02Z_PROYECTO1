/*! @file : sdk_hal_uart0.c
 * @author  Maria alejandra pabon
 * @version 1.0.0
 * @date    14/01/2021
 * @brief   Driver para uart0
 * @details
 *
*/
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_lpsci.h"
#include <sdk_hal_uart0.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART0_IRO_FUNCTION UART0_IRQHandler
#define UART0_IRO_INDEX  UART0_IRQn

/*******************************************************************************
 * Private Prototypes
 ******************************************************************************/


/*******************************************************************************
 * External vars
 ******************************************************************************/


/*******************************************************************************
 * Local vars
 ******************************************************************************/
uint8_t uart0_buffer_circular[LONGITUD_BUFFER_CIRCULAR];

volatile uint16_t txIndex; /*datos a enviar */
volatile uint16_t rxIndex; /*memoria para guardar datos nuevos */


/*******************************************************************************
 * Private Source Code
 ******************************************************************************/
void UART0_IRO_FUNCTION(void)
{
	uint8_t data;

	if((kLPSCI_RxDataRegFullFlag)&LPSCI_GetStatusFlags(UART0))
	{
		data = LPSCI_ReadByte(UART0);

		if (((rxIndex + 1 )% LONGITUD_BUFFER_CIRCULAR) != txIndex)
		{
			uart0_buffer_circular[rxIndex] = data;
			rxIndex++;
			rxIndex %= LONGITUD_BUFFER_CIRCULAR;
		}
	}
}

/*******************************************************************************
 * Public Source Code
 ******************************************************************************/
 status_t uart0Inicializar(uint32_t baud_rate){
	 lpsci_config_t config;
	 status_t status;

	 LPSCI_GetDefaultConfig(&config);
	 config.baudRate_Bps = baud_rate;
	 config.enableTx = true;
	 config.enableRx = true;

	 status=LPSCI_Init(UART0, &config, CLOCK_GetFreq(kCLOCK_McgFllClk));
	 if (status != kStatus_Success)
		 return (status);

	 LPSCI_EnableInterrupts(UART0, kLPSCI_RxDataRegFullInterruptEnable);
	 status = EnableIRQ(UART0_IRO_INDEX);
	 return (status);
 }

uint8_t uart0NuevosDatosEnBuffer(void){
	uint8_t NumeroDeDatosNuevosEnBuffer;
	NumeroDeDatosNuevosEnBuffer = (uint8_t) (rxIndex - txIndex);
	return (NumeroDeDatosNuevosEnBuffer);
}


status_t uart0LeerByteDesdeBufferCircular(uint8_t *Nuevo_Byte){
	if (rxIndex != txIndex){
		*Nuevo_Byte=uart0_buffer_circular[txIndex];
		txIndex++;
		txIndex %= LONGITUD_BUFFER_CIRCULAR;
		return (kStatus_Success);
	}else {
		return(kStatus_Fail);
	}
}
