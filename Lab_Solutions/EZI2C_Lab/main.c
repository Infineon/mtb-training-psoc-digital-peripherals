/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the EZI2C Lab for the PSOC Peripherals
*              training.
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

#define BUFFER_SIZE     (8UL)
#define TURN_LED_OFF    (0x5A)
#define TURN_LED_ON     (0xA5)

#ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
/* The timeout value in microseconds used to wait for CM55 core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC    (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR          (CYMEM_CM33_0_m55_nvm_START + \
                                        CYBSP_MCUBOOT_HEADER_SIZE)
#endif


uint8_t buffer[BUFFER_SIZE];
cy_stc_scb_ezi2c_context_t ezI2cContext;
volatile uint32_t tick = 0;

void EZI2C_Isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezI2cContext);
}

void SysTick_Callback(void)
{
    tick++;
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
    
    /* Configure EZI2C slave */
    (void) Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezI2cContext);

    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, buffer, BUFFER_SIZE, BUFFER_SIZE, &ezI2cContext);

    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_LF, ((uint32_t)(((1000)/1000000.0)*32768)));
    #endif
    /* Register one of the SysTick callbacks */
    Cy_SysTick_SetCallback(0UL, &SysTick_Callback);


    /* Populate configuration structure */
    const cy_stc_sysint_t ezI2cIntrConfig =
    {
        .intrSrc      = CYBSP_EZI2C_IRQ,
        .intrPriority = 3U,
    };

    /* Hook interrupt service routine and enable interrupt */
    (void) Cy_SysInt_Init(&ezI2cIntrConfig, &EZI2C_Isr);
    NVIC_EnableIRQ(ezI2cIntrConfig.intrSrc);

    /* Enable EZI2C to operate */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

    /* Enable global interrupts */
    __enable_irq();

    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    #endif

    uint32_t old_tick = tick;

    for (;;)
    {
        if (tick != old_tick)
        {
            old_tick = tick;
            buffer[0] = ((tick>>24) & 0xFF);
            buffer[1] = ((tick>>16) & 0xFF);
            buffer[2] = ((tick>>8) & 0xFF);
            buffer[3] = (tick & 0xFF);
        }
        
        if (CY_SCB_EZI2C_STATUS_WRITE1 == 
        (CY_SCB_EZI2C_STATUS_WRITE1 & Cy_SCB_EZI2C_GetActivity(CYBSP_EZI2C_HW, &ezI2cContext)))
        {
            /* If a write to the first buffer occurred */
            if (TURN_LED_OFF == buffer[4])
            {
                Cy_GPIO_Clr(CYBSP_LED2_PORT, CYBSP_LED2_PIN);
            }
            else if (TURN_LED_ON == buffer[4])
            {
                Cy_GPIO_Set(CYBSP_LED2_PORT, CYBSP_LED2_PIN);
            }
        }
    }

}

/* [] END OF FILE */
