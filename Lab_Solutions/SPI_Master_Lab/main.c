/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the SPI_Master Lab for the PSOC
*              Peripherals training.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include <stdio.h>

#ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
/* The timeout value in microseconds used to wait for CM55 core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC    (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR          (CYMEM_CM33_0_m55_nvm_START + \
                                        CYBSP_MCUBOOT_HEADER_SIZE)
#endif

/* Allocate context for SPI operation */
cy_stc_scb_spi_context_t spiContext;

void spi_isr(void)
{
    Cy_SCB_SPI_Interrupt(SPI_MASTER_HW, &spiContext);
}

void print_buffer(char * message, uint8_t * buffer, uint8_t size)
{
    Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    for (uint8_t i = 0; i < size; i++)
    {
        char print[8];
        sprintf(print, "0x%x; ", buffer[i]);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, print);
    }
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
}

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    
    /* Configure SPI to operate */
    (void) Cy_SCB_SPI_Init(SPI_MASTER_HW, &SPI_MASTER_config, &spiContext);

    /* Populate configuration structure */
    const cy_stc_sysint_t spi_intr_config =
    {
        .intrSrc      = SPI_MASTER_IRQ,
        .intrPriority = 3U,
    };

    /* Hook interrupt service routine and enable interrupt */
    (void)Cy_SysInt_Init(&spi_intr_config, &spi_isr);
    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ(SPI_MASTER_IRQ);

    /* Enable the SPI Master block */
    Cy_SCB_SPI_Enable(SPI_MASTER_HW);


    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, NULL);

    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "SPI Master\r\n");


    /* Enable global interrupts */
    __enable_irq();

    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    #endif

    cy_en_scb_spi_status_t master_status;
    uint8_t tx_buffer[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t rx_buffer[4];

    for (;;)
    {
        if (0UL == Cy_GPIO_Read(CYBSP_SW2_PORT, CYBSP_SW2_PIN))
        {
            /* Initiate SPI Master write transaction. */
            master_status = Cy_SCB_SPI_Transfer(SPI_MASTER_HW, tx_buffer, rx_buffer, 4, &spiContext);
            
            if (CY_SCB_SPI_BAD_PARAM == master_status)
            {
                CY_ASSERT(0);
            }

            /* Blocking wait for transfer completion */
            while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(SPI_MASTER_HW, &spiContext)))
            {
            }

            print_buffer("Send bytes: ", tx_buffer, 4);
            print_buffer("Receive bytes: ", rx_buffer, 4);

            
            /* Simple de-bounce on the GPIO press. */
            Cy_SysLib_Delay(500);
            
        }

    }
}

/* [] END OF FILE */
