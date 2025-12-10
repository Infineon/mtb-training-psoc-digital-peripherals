/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the GPIO Lab for the PSOC Peripherals
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

#include "cy_gpio.h"
#include "cy_pdl.h"
#include "cy_syslib.h"
#include "cybsp.h"

#ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
/* The timeout value in microseconds used to wait for CM55 core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC    (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR          (CYMEM_CM33_0_m55_nvm_START + \
                                        CYBSP_MCUBOOT_HEADER_SIZE)

#define NEW_LED_PORT    GPIO_PRT16
#define NEW_LED_PIN     (7u)
#elif defined(TARGET_APP_CY8CPROTO_041TP)
#define NEW_LED_PORT    GPIO_PRT5
#define NEW_LED_PIN     (4u)
#elif defined(TARGET_APP_CY8CPROTO_040T_MS)
#define NEW_LED_PORT    GPIO_PRT3
#define NEW_LED_PIN     (0u)
#elif defined(TARGET_APP_CY8CKIT_062S2_43012)
#define NEW_LED_PORT    GPIO_PRT1
#define NEW_LED_PIN     (5u)
#elif defined(TARGET_APP_CY8CKIT_041S_MAX)
#define NEW_LED_PORT    GPIO_PRT7
#define NEW_LED_PIN     (3u)
#endif

static bool enable_blink = false;

void button_isr(void)
{
    enable_blink ^= 1;
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN1_PORT, CYBSP_USER_BTN1_PIN);
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
    
    Cy_GPIO_SetDrivemode(NEW_LED_PORT, NEW_LED_PIN, CY_GPIO_DM_STRONG);
    
    cy_stc_sysint_t intrCfg = 
    {
        /*.intrSrc =*/ CYBSP_USER_BTN1_IRQ,
        /*.intrPriority =*/ 3UL 
    };
    Cy_SysInt_Init(&intrCfg, button_isr);
    
    /* Enable the interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    #endif

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        if (1UL == enable_blink)
        {
            #ifndef TARGET_APP_CY8CPROTO_040T_MS
            Cy_GPIO_Set(CYBSP_LED2_PORT, CYBSP_LED2_PIN);
            #endif	
            Cy_GPIO_Clr(NEW_LED_PORT, NEW_LED_PIN);
            Cy_SysLib_Delay(1000);
            #ifndef TARGET_APP_CY8CPROTO_040T_MS
            Cy_GPIO_Clr(CYBSP_LED2_PORT, CYBSP_LED2_PIN);
            #endif
            Cy_GPIO_Set(NEW_LED_PORT, NEW_LED_PIN);
            Cy_SysLib_Delay(1000);
        }
        else
        {
            #if !defined(TARGET_APP_CY8CKIT_062S2_43012) && !defined(TARGET_APP_CY8CKIT_041S_MAX)
            Cy_GPIO_Clr(NEW_LED_PORT, NEW_LED_PIN);
            #ifndef TARGET_APP_CY8CPROTO_040T_MS
            Cy_GPIO_Clr(CYBSP_LED2_PORT, CYBSP_LED2_PIN);
            #endif
            #else
            Cy_GPIO_Set(NEW_LED_PORT, NEW_LED_PIN);
            Cy_GPIO_Set(CYBSP_LED2_PORT, CYBSP_LED2_PIN);
            #endif
        }

    }
}

/* [] END OF FILE */
