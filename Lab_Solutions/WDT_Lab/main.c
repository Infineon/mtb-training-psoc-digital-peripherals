/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the WDT Lab for the PSOC Peripherals
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

#ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
/* The timeout value in microseconds used to wait for CM55 core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC    (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR          (CYMEM_CM33_0_m55_nvm_START + \
                                        CYBSP_MCUBOOT_HEADER_SIZE)
#endif

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, NULL);

    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "WDT with system reset on expiration \r\n");

    #if defined (TARGET_APP_CY8CPROTO_041TP) || defined (TARGET_APP_CY8CPROTO_040T_MS) || defined (TARGET_APP_CY8CKIT_041S_MAX)
    /* Enables the watchdog timer reset generation. for PSOC 4. */
    Cy_WDT_Enable();
    #else
    /* Enables the watchdog timer reset generation. for PSOC 6 or PSOC Edge. */
    Cy_WDT_Unlock();
    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* PSOC™ Edge default match bits is 0,
    * set to 14 so that first 14 match bits are used. Only for PSOC™ Edge. */
    Cy_WDT_SetMatchBits(14);
    #endif
    Cy_WDT_SetMatch(0x8000);
    Cy_WDT_ClearInterrupt();
    Cy_WDT_Enable();
    Cy_WDT_Lock();
    #endif

    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    #endif

    /* Enable global interrupts */
    __enable_irq();
    
    for (;;)
    {
        if (0UL == Cy_GPIO_Read(CYBSP_SW4_PORT, CYBSP_SW4_PIN))
        {
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Button pressed, entering infinite loop \r\n");
            while(1);
        }
        Cy_WDT_ClearWatchdog();
    }

}

/* [] END OF FILE */
