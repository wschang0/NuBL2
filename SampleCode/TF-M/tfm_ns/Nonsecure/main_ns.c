/**************************************************************************//**
 * @file     main_ns.c
 * @version  V1.00
 * @brief    Non-secure sample code for TrustZone
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

#include <arm_cmse.h>
#include "NuMicro.h"

#define LED_INIT()  {GPIO_SetMode(PA, BIT11 | BIT10, GPIO_MODE_OUTPUT);}
#define LED_GREEN   PA10
#define LED_RED     PA11

typedef int32_t (*funcptr)(uint32_t);

void App_Init(uint32_t u32BootBase);
void DEBUG_PORT_Init(void);


/*----------------------------------------------------------------------------
  NonSecure functions used for callbacks
 *----------------------------------------------------------------------------*/
int32_t NonSecure_LED_On(uint32_t num)
{
    printf("Nonsecure LED On call by Secure\n");
    LED_GREEN = 0;
    return 0;
}

int32_t NonSecure_LED_Off(uint32_t num)
{
    printf("Nonsecure LED Off call by Secure\n");
    LED_GREEN = 1;
    return 0;
}

/*----------------------------------------------------------------------------
  NonSecure LED control
 *----------------------------------------------------------------------------*/
void LED_On(uint32_t us)
{
    printf("Nonsecure LED On\n");
    LED_RED = 0;
}

void LED_Off(uint32_t us)
{
    printf("Nonsecure LED Off\n");
    LED_RED = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* SysTick IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

    switch(u32Ticks++)
    {
        case   0:
            LED_On(7u);
            break;
        case 30:
            LED_Off(7u);
            break;
        default:
            if(u32Ticks > 60)
            {
                u32Ticks = 0;
            }
    }
}

extern int sd(void);

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    printf("\n");
    printf("+---------------------------------------------+\n");
    printf("|           Nonsecure is running ...          |\n");
    printf("+---------------------------------------------+\n");

    /* Init PC for Nonsecure LED control */
    LED_INIT();

    /* Generate Systick interrupt each 10 ms */
    SystemCoreClockUpdate();
    //SysTick_Config(SystemCoreClock / 100);

    
    sd();
    
    while(1);
}



void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void App_Init(uint32_t u32BootBase)
{
    funcptr fp;
    uint32_t u32StackBase;

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    fp = ((funcptr)(*(((uint32_t *)SCB->VTOR) + 1)));

    /* Check if the stack is in secure SRAM space */
    u32StackBase = M32(u32BootBase);
    if((u32StackBase >= 0x30000000UL) && (u32StackBase < 0x40000000UL))
    {
        printf("Execute non-secure code ...\n");
        /* SCB.VTOR points to the target Secure vector table base address. */
        SCB->VTOR = u32BootBase;

        fp(0); /* Non-secure function call */
    }
    else
    {
        /* Something went wrong */
        printf("No code in non-secure region!\n");

        while(1);
    }
}


uint32_t CLK_GetCPUFreq()
{
    return 64000000;
}

uint32_t CLK_GetHXTFreq()
{
    return __HXT;
}

uint32_t CLK_GetPLLClockFreq()
{
    return CLK_GetCPUFreq();
}

uint32_t CLK_GetHCLKFreq()
{
    return CLK_GetCPUFreq();
}

uint32_t CLK_GetPCLK0Freq()
{
    return CLK_GetCPUFreq();
}

uint32_t CLK_GetPCLK1Freq()
{
    return CLK_GetCPUFreq();
}

uint32_t CLK_GetModuleClockSource()
{
    return 0;
}

uint32_t CLK_GetModuleClockDivider()
{
    return 0;
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
