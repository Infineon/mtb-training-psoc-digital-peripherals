/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the SPI_Slave Lab for the PSOC
*              Peripherals training.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
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
volatile bool print_data;
uint8_t rx_buffer[4];
uint8_t tx_buffer[4] = {0xBE, 0xEF, 0xCA, 0xFE};


void spi_sr(void)
{
    Cy_SCB_SPI_Interrupt(SPI_SLAVE_HW, &spiContext);
}

void spi_callback(uint32_t event)
{
    if (CY_SCB_SPI_TRANSFER_CMPLT_EVENT == event)
    {
        Cy_SCB_SPI_Transfer(SPI_SLAVE_HW, tx_buffer, rx_buffer, 4, &spiContext);
        print_data = true;
    }
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
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure SPI to operate */
    (void) Cy_SCB_SPI_Init(SPI_SLAVE_HW, &SPI_SLAVE_config, &spiContext);

    Cy_SCB_SPI_RegisterCallback(SPI_SLAVE_HW, spi_callback, &spiContext);

    /* Populate configuration structure */
    const cy_stc_sysint_t spiIntrConfig =
    {
        .intrSrc      = SPI_SLAVE_IRQ,
        .intrPriority = 3U,
    };

    /* Hook interrupt service routine and enable interrupt */
    (void) Cy_SysInt_Init(&spiIntrConfig, &spi_sr);
    NVIC_EnableIRQ(spiIntrConfig.intrSrc);

    /* Enable the SPI Slave block */
    Cy_SCB_SPI_Enable(SPI_SLAVE_HW);


    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, NULL);

    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "SPI Slave\r\n");

    /* Enable global interrupts */
    __enable_irq();

    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    #endif

    /* Setup the first transfer. */
    Cy_SCB_SPI_Transfer(SPI_SLAVE_HW, tx_buffer, rx_buffer, 4, &spiContext);

    for (;;)
    {
        if(true == print_data)
        {
            print_data = false;
            print_buffer("Receive bytes: ", rx_buffer, 4);
            print_buffer("Send bytes: ", tx_buffer, 4);
        }
    }

}

/* [] END OF FILE */
