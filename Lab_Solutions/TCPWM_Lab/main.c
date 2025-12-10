/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC4  Application
*              for ModusToolbox.
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


volatile bool timer_flag = false;
volatile bool capture_flag = false;

void capture_isr(void)
{
    capture_flag = true;
    Cy_TCPWM_ClearInterrupt(CAPTURE_HW, CAPTURE_NUM, Cy_TCPWM_GetInterruptStatus(CAPTURE_HW, CAPTURE_NUM));
}


void timer_isr(void)
{
    timer_flag = true;
    #if defined(TARGET_APP_CY8CKIT_062S2_43012) || defined(TARGET_APP_CY8CKIT_041S_MAX) || defined(TARGET_APP_KIT_PSE84_EVAL_EPC2)
    Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN);
    #else
    Cy_GPIO_Inv(CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN);
    #endif
    Cy_TCPWM_ClearInterrupt(TIMER_HW, TIMER_NUM, Cy_TCPWM_GetInterruptStatus(TIMER_HW, TIMER_NUM));
}

void print_float(float value) 
{
    char print_float[20];
    int int_part = (int)value; /* Integer part */ 
    int frac_part = (int)((value - int_part) * 100); /* Fractional part (2 decimal places) */
    sprintf(print_float, "%d.%02d", int_part, frac_part);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, print_float);
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

    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(TIMER_HW, TIMER_NUM, &TIMER_config))
    {
        /* Handle possible errors */
    }

    cy_stc_sysint_t timer_intr_cfg =
    {
        /*.intrSrc =*/ TIMER_IRQ,
        /*.intrPriority =*/ 3UL 
    };

    Cy_SysInt_Init(&timer_intr_cfg, timer_isr);

    /* Enable the interrupt */
    NVIC_EnableIRQ(timer_intr_cfg.intrSrc);

    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(PWM_HW, PWM_NUM, &PWM_config))
    {
        /* Handle possible errors */
    }    


    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(EVENT_COUNTER_HW, EVENT_COUNTER_NUM, &EVENT_COUNTER_config))
    {
        /* Handle possible errors */
    }	

    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(CAPTURE_HW, CAPTURE_NUM, &CAPTURE_config))
    {
        /* Handle possible errors */
    }	

    cy_stc_sysint_t capture_intr_cfg = 
    {
        /*.intrSrc =*/ CAPTURE_IRQ,
        /*.intrPriority =*/ 3UL 
    };

    Cy_SysInt_Init(&capture_intr_cfg, capture_isr);

    /* Enable the interrupt */
    NVIC_EnableIRQ(capture_intr_cfg.intrSrc);


    #if defined(TARGET_APP_CY8CPROTO_041TP) || defined(TARGET_APP_CY8CKIT_041S_MAX) || defined(TARGET_APP_CY8CPROTO_040T_MS)
    /* Enable the initialized counters */
    Cy_TCPWM_Enable_Multiple(PWM_HW, TIMER_MASK | PWM_MASK | EVENT_COUNTER_MASK | CAPTURE_MASK);
    /* Then start the counters */
    Cy_TCPWM_TriggerReloadOrIndex(PWM_HW, TIMER_MASK | PWM_MASK | EVENT_COUNTER_MASK | CAPTURE_MASK);
    #elif defined(TARGET_APP_CY8CKIT_062S2_43012)
    /* Enable the initialized counters */
    Cy_TCPWM_Enable_Multiple(PWM_HW, TIMER_MASK | PWM_MASK);
    Cy_TCPWM_Enable_Multiple(EVENT_COUNTER_HW, EVENT_COUNTER_MASK | CAPTURE_MASK);
    /* Then start the counters */
    Cy_TCPWM_TriggerReloadOrIndex(PWM_HW, TIMER_MASK | PWM_MASK );
    Cy_TCPWM_TriggerReloadOrIndex(EVENT_COUNTER_HW, EVENT_COUNTER_MASK | CAPTURE_MASK);
    #else

    /* Enable the TIMER counter */
    Cy_TCPWM_Counter_Enable(TIMER_HW, TIMER_NUM);
    /* Then start the counter */
    Cy_TCPWM_TriggerStart_Single(TIMER_HW, TIMER_NUM);

    /* Enable the PWM */
    Cy_TCPWM_PWM_Enable(PWM_HW, PWM_NUM);
    /* Then start the PWM */
    Cy_TCPWM_TriggerStart_Single(PWM_HW, PWM_NUM);
    
    /* Enable the EVENT_COUNTER counter */
    Cy_TCPWM_Counter_Enable(EVENT_COUNTER_HW, EVENT_COUNTER_NUM);
    /* Then start the counter */
    Cy_TCPWM_TriggerStart_Single(EVENT_COUNTER_HW, EVENT_COUNTER_NUM);

    /* Enable the CAPTURE counter */
    Cy_TCPWM_Counter_Enable(CAPTURE_HW, CAPTURE_NUM);
    /* Then start the counter */
    Cy_TCPWM_TriggerStart_Single(CAPTURE_HW, CAPTURE_NUM);
    #endif


    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, NULL);

    /* Enable global interrupts */
    __enable_irq();

	#ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
    #endif

    Cy_SCB_UART_Enable(CYBSP_UART_HW);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "Hello PSOC TCPWM\r\n");


    /* Get the TCPWM counter period. */
    uint32_t counter_period = Cy_TCPWM_Counter_GetPeriod(EVENT_COUNTER_HW, EVENT_COUNTER_NUM);

    #if defined(TARGET_APP_CY8CPROTO_041TP) || defined(TARGET_APP_CY8CKIT_041S_MAX) || defined(TARGET_APP_CY8CPROTO_040T_MS)
    /* Get the clock divider type and number. */
    uint32_t clock_divider = Cy_SysClk_PeriphGetAssignedDivider(PCLK_TCPWM_CLOCKS2);
    uint32_t divider_type = (clock_divider&PERI_PCLK_CTL_SEL_TYPE_Msk)>>PERI_PCLK_CTL_SEL_TYPE_Pos;
    uint32_t divider_number = (clock_divider&PERI_PCLK_CTL_SEL_DIV_Msk)>>PERI_PCLK_CTL_SEL_DIV_Pos;
    #else

    #ifdef TARGET_APP_CY8CKIT_062S2_43012
    /* Get the clock divider type and number for PSOC 6. */
    uint32_t clock_divider = Cy_SysClk_PeriphGetAssignedDivider(PCLK_TCPWM1_CLOCKS0);
    #endif
    #ifdef TARGET_APP_KIT_PSE84_EVAL_EPC2
    /* Get the clock divider type and number for PSOC Edge. */
    uint32_t clock_divider = Cy_SysClk_PeriphGetAssignedDivider(PCLK_TCPWM0_CLOCK_COUNTER_EN2);
    #endif
    
    
    uint32_t divider_type = (clock_divider&CY_PERI_CLOCK_CTL_TYPE_SEL_Msk)>>CY_PERI_CLOCK_CTL_TYPE_SEL_Pos;
    uint32_t divider_number = (clock_divider&CY_PERI_CLOCK_CTL_DIV_SEL_Msk)>>CY_PERI_CLOCK_CTL_DIV_SEL_Pos;
    #endif

    /* To divide the TCPWM clock by 1 requires the pre-scaler register to be set to 0, so add 1 to the pre-scaler. */
    uint32_t prescaler = TIMER_config.clockPrescaler + 1U;

    /* Determine the frequency of the clock driving the TCPWM counter. */
    uint32_t clock_frequency = Cy_SysClk_PeriphGetFrequency(divider_type, divider_number)/prescaler;

    /* Determine the period in seconds of the TCPWM counter. */
    float counter_time = (1/(float)clock_frequency)*(float)counter_period;
    uint32_t capture_result_high = 0;
    uint32_t capture_result_low = 0;

    for (;;)
    {
        if (true == timer_flag)
        {
            timer_flag = false;
            uint32_t event_count = Cy_TCPWM_Counter_GetCounter(EVENT_COUNTER_HW, EVENT_COUNTER_NUM);
            Cy_TCPWM_Counter_SetCounter(EVENT_COUNTER_HW, EVENT_COUNTER_NUM, 0);
            
            float signal_frequency = (float)event_count/counter_time;

            Cy_SCB_UART_PutString(CYBSP_UART_HW, "******************************************************************\n\r");
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Signal Frequency = ");
            print_float(signal_frequency);
            Cy_SCB_UART_PutString(CYBSP_UART_HW, " hz \r\n");
            
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Signal Duty Cycle =  ");
            print_float(((float)capture_result_high * 100)/((float)capture_result_low + (float)capture_result_high-1));
            Cy_SCB_UART_PutString(CYBSP_UART_HW, " % \r\n");

            Cy_SCB_UART_PutString(CYBSP_UART_HW, "******************************************************************\n\r");
            
            uint32_t pwm_period = Cy_TCPWM_PWM_GetPeriod0(PWM_HW, PWM_NUM);
            uint32_t pwm_compare = Cy_TCPWM_PWM_GetCompare0(PWM_HW, PWM_NUM) + 100;

            if (pwm_period > pwm_compare)
            {
                Cy_TCPWM_PWM_SetCompare0(PWM_HW, PWM_NUM, pwm_compare+100);
            }
            else
            {
                Cy_TCPWM_PWM_SetCompare0(PWM_HW, PWM_NUM, 100);
            }

        }
        
        if (true == capture_flag)
        {
            capture_flag = false;
            if (0UL == Cy_GPIO_Read(FEEDBACK_PIN_PORT, FEEDBACK_PIN_PIN))
            {
                capture_result_low = Cy_TCPWM_Counter_GetCaptureBuf(CAPTURE_HW, CAPTURE_NUM);
            }
            else 
            {
                capture_result_high = Cy_TCPWM_Counter_GetCaptureBuf(CAPTURE_HW, CAPTURE_NUM);
            }
            Cy_TCPWM_Counter_SetCounter(CAPTURE_HW, CAPTURE_NUM, 0);
        }

    }


}

/* [] END OF FILE */
