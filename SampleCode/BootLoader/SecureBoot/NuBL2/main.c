/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to generate a boot image(NuBL2) and can be authenticated by Secure Botloader(NuBL1).
 *           After NuBL2 runs, NuBL2 will authenticate NuBL32 and NuBL33 then jumpt to execute in NuBL32.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL2.h"



#define SET_SECURE_BOOT     (0) // Set 1 to support modify CFG0[5] MBS 0 for booting from Secure Bootloader 
#define ENABLE_XOM0_REGION  (0) // Set 1 to configure VerifyNuBL3x.c code in XOM0 region, and cannot trace VerifyNuBL3x.c flow in ICE debug mode

extern int32_t UpgradeProcess(void);
extern uint32_t *g_NuBL32InfoStart;
extern uint32_t *g_NuBL32InfoEnd;
extern uint32_t *g_NuBL33InfoStart;
extern uint32_t *g_NuBL33InfoEnd;

extern const uint32_t g_InitialFWinfo[]; // A global variable to store NuBL2 FWINFO address, declared in FwInfo.c


/*---------------------------------------------------------------------------------------------------------*/
/*  Check Booting status and show F/W info data                                                            */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t CheckBootingStatus(void)
{
    int32_t     i;
    uint32_t    u32CFG0, au32OTP[8];
    uint32_t    *pu32Info, u32Size;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_ENABLE_ISP();

    u32CFG0 = FMC_Read(FMC_USER_CONFIG_0);
    //printf("Current CFG0: 0x%x.\n\n", u32CFG0);

    if((u32CFG0 & BIT5) == BIT5)
    {
        //printf("Device is booting from %s.\n", FMC_GetBootSource()==0?"APROM":"LDROM");

#if (SET_SECURE_BOOT == 1) // enable to configure booting from Secure Bootloader
        {
            char ch;
            printf("Hit [S/s] to configure booting from Secure Bootloader(NuBL1).\n\n");
            ch = getchar();
            if((ch == 's') || (ch == 'S'))
            {
                FMC_ENABLE_CFG_UPDATE();
                FMC_Write(FMC_USER_CONFIG_0, (u32CFG0 & ~BIT5));

                /* Reset chip to enable booting from Secure Bootloader. */
                SYS_ResetChip();
                while(1) {};
            }
        }
#endif

        //return -1;
    }

    printf("[Device is successfully booting from Secure Bootloader(NuBL1) and device PID is 0x%08x]\n\n", FMC_ReadPID());

    /*
        Notes of NuBL2 ECC public key and its SHA-256 Key hash:
        * Public Key 1 = 755B3819F05A3E9F32D4D599062834AAC5220F75955378414A8F63716A152CE2
          Public Key 2 = 91C413F1915ED7B47473FD797647BA3D83E8224377909AF5B30C530EAAD79FD7
        * The Key hash = 145e73e48865222fa4d9741671c0c670ed45fe06b24cdb5dd507d7ab35ee9363
        * Stored in M2351 OTP0~3 for identification in secure boot are:
                Index   Low word    High word
            --------------------------------------
                OTP0:   0xe4735e14, 0x2f226588
                OTP1:   0x1674d9a4, 0x70c6c071
                OTP2:   0x06fe45ed, 0x5ddb4cb2
                OTP3:   0xabd707d5, 0x6393ee35
    */

    /* Read NuBL2 ECC public key hash */
    FMC_Read_OTP(0, &au32OTP[0], &au32OTP[1]);
    FMC_Read_OTP(1, &au32OTP[2], &au32OTP[3]);
    FMC_Read_OTP(2, &au32OTP[4], &au32OTP[5]);
    FMC_Read_OTP(3, &au32OTP[6], &au32OTP[7]);


    printf("NuBL2 ECC public key hash are:\n");
    for(i = 7; i >= 0; i--)
    {
        printf("%08x", au32OTP[i]);
    }
    printf("\n");


    /* Show NuBL2 F/W info data */
    pu32Info = (uint32_t *)g_InitialFWinfo;
    u32Size = sizeof(FW_INFO_T);

    return 0;
}

void EnableXOM0(void)
{
    int32_t i32Status;
    uint32_t u32Base = 0x10000, u32Page = 4;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    if((FMC->XOMSTS & 0x1) != 0x1)
    {
        printf("\nXOM0 base: 0x%x, page count: %d.\n\n", u32Base, u32Page);

        if(FMC_GetXOMState(XOMR0) == 0)
        {
            i32Status = FMC_ConfigXOM(XOMR0, u32Base, u32Page);
            if(i32Status == 0)
            {
                printf("Configure XOM0 Success.\n");
            }
            else
            {
                printf("Configure XOM0 FAIL.\n");
                while(1) {}
            }
        }
        else
        {
            printf("Get XOM0 status FAIL.\n\n");
            while(1) {}
        }

        printf("Reset chip to enable XOM region.\n\n");
        UART_WAIT_TX_EMPTY((UART_T *)DEBUG_PORT);

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();
        while(1) {};
    }
    else
    {
        printf("XOM0 region is already actived.\n\n");
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC48, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));


    // Initial for SPI0 for storage

    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_SDH0SEL_Msk) | CLK_CLKSEL0_SDH0SEL_HCLK;

    /* select multi-function pin */
    /* CD: PB12(9), PD13(3) */
    //SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SD0_nCD_PB12;
    SYS->GPD_MFPH = (SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD13MFP_Msk)) | SD0_nCD_PD13;

    /* CLK: PB1(3), PE6(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SD0_CLK_PB1;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE6MFP_Msk)) | SD0_CLK_PE6;

    /* CMD: PB0(3), PE7(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SD0_CMD_PB0;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE7MFP_Msk)) | SD0_CMD_PE7;

    /* D0: PB2(3), PE2(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SD0_DAT0_PB2;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE2MFP_Msk)) | SD0_DAT0_PE2;

    /* D1: PB3(3), PE3(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SD0_DAT1_PB3;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE3MFP_Msk)) | SD0_DAT1_PE3;

    /* D2: PB4(3), PE4(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SD0_DAT2_PB4;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE4MFP_Msk)) | SD0_DAT2_PE4;

    /* D3: PB5(3)-, PE5(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SD0_DAT3_PB5;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE5MFP_Msk)) | SD0_DAT3_PE5;

    /* Enable IP clock */
    CLK->AHBCLK |= CLK_AHBCLK_SDH0CKEN_Msk; // SD Card driving clock.
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open((UART_T *)DEBUG_PORT, 115200);
}

void SwapCpy(uint8_t *pu8Dest, uint8_t *pu8Src, int32_t nBytes)
{
    int32_t i;

    for(i = 0; i < nBytes; i++)
    {
        pu8Dest[i] = pu8Src[nBytes - i - 1];
    }
}

void(*func)(void);

extern int sd(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i;
    uint32_t u32Tmp;
    uint32_t u32Cfg0;
    uint32_t au32Pk1[8], au32Pk2[8];
    uint32_t au32PkHash[8];
    FW_INFO_T fwinfo;

    uint32_t u32NuBL32Base;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("=== Nuvoton PSA Bootloader v1.0 ===\n");

    /* Check Root of Trusted Function Status */
    /* Enable FMC ISP function */
    FMC_ENABLE_ISP();
    u32Cfg0 = FMC_Read(FMC_USER_CONFIG_0);
    FMC_Read_OTP(0, &au32PkHash[0], &au32PkHash[1]);
    FMC_Read_OTP(1, &au32PkHash[2], &au32PkHash[3]);
    FMC_Read_OTP(2, &au32PkHash[4], &au32PkHash[5]);
    FMC_Read_OTP(3, &au32PkHash[6], &au32PkHash[7]);
    u32Tmp = -1ul;
    for(i = 0; i < 8; i++)
    {
        u32Tmp &= au32PkHash[i];
        //printf("0x%08x\n", au32PkHash[i]);
    }

    printf("Root of Trusted Function ........ %s!\n", (((u32Cfg0 & BIT5) == 0) && (u32Tmp != 0xfffffffful)) ? "Enabled" : "Disabled");

    /* Check firmware upgrade when PB0 == 0 after reset */
    if(PB0 == 0)
    {
        printf("!!! Start Firmware Upgrade Procedure !!!\n");
        UpgradeProcess();
    }
    
    
    /* Verify NuBL32 identity and F/W integrity */
    SwapCpy((uint8_t *)&au32Pk1[0], (uint8_t *)g_InitialFWinfo, sizeof(au32Pk1));
    SwapCpy((uint8_t *)&au32Pk2[0], (uint8_t *)g_InitialFWinfo + 32, sizeof(au32Pk2));
    
#if 1
    /* It is necessary to copy FwInfo to SRAM before doing SHA256 calculation */
    memcpy((void *)&fwinfo, (void *)&g_NuBL32InfoStart, sizeof(FW_INFO_T));
    if(VerifyNuBL3x(&fwinfo, au32Pk1, au32Pk2, NULL) < 0)
    {
        printf("TF-M verify ..................... FAILED!\n");
        goto lexit;
    }
    else
    {
        printf("TF-M verify ..................... PASSED!\n");
        u32NuBL32Base = fwinfo.mData.au32FwRegion[0].u32Start;
    }
#else
    u32NuBL32Base = 0x8000;
    /* Check TF-M existed */
    if(M8(u32NuBL32Base + 3) != 0x20)
    {
        printf("Failed to detect TF-M.\n");
        goto lexit;
    }

#endif

#if 1
    /* Verify NuBL33 identity and F/W integrity */
    memcpy((void *)&fwinfo, (void *)&g_NuBL33InfoStart, sizeof(FW_INFO_T));
    if(VerifyNuBL3x(&fwinfo, au32Pk1, au32Pk2, NULL) < 0)
    {
        printf("Non-secure application verify ... FAILED!\n");
        goto lexit;
    }
    else
    {
        printf("Non-secure application verify ... PASSED!\n");
    }

#endif


#if 0
    printf("Press any key to boot TF-M\n");
    UART_WAIT_TX_EMPTY((UART_T *)DEBUG_PORT);
    getchar();
#endif    


    SCB->VTOR = u32NuBL32Base;
    func = (void(*)(void))M32(u32NuBL32Base + 4);
    __set_MSP(M32(u32NuBL32Base));
    func();

lexit:
    printf("Trusted Boot Failed!\n");

    for(;;)
    {
        __WFI();
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
