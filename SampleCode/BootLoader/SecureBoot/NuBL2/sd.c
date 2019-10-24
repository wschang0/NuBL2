/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Access a SD card formatted in FAT file system
 *
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "diskio.h"
#include "ff.h"
#include "NuBL2.h"

#define BUFF_SIZE       (8*1024)
#define NSCBA           0x40000
//#define ENCRYPT         1

//45EBFB9BFE1BBE9CE31F9F2063BAAC9DB6D75916B2CD72A0FD20AE66CFCF402E
//uint32_t g_au32Key[8] = {0xCFCF402E,0xFD20AE66,0xB2CD72A0,0xB6D75916,0x63BAAC9D,0xE31F9F20,0xFE1BBE9C,0x45EBFB9B};
uint32_t g_au32Key[8] = {0x45EBFB9B,0xFE1BBE9C,0xE31F9F20,0x63BAAC9D,0xB6D75916,0xB2CD72A0,0xFD20AE66, 0xCFCF402E};
//CF177955792C5D97AF0B7E6FA5F37766
uint32_t g_au32IV[4] ={0xCF177955,0x792C5D97,0xAF0B7E6F,0xA5F37766};


extern uint32_t *g_NuBL32InfoStart;
extern uint32_t *g_NuBL33InfoStart;

extern void Cal_SHA256_Flash(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest);
extern void SwapCpy(uint8_t *pu8Dest, uint8_t *pu8Src, int32_t nBytes);
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t g_au8BuffPool[BUFF_SIZE] ;       /* Working buffer */
#else
__attribute__((aligned(4))) uint8_t g_au8BuffPool[BUFF_SIZE] = {0};       /* Working buffer */
#endif
uint32_t volatile g_u32Sec = 0;

extern uint8_t volatile g_u8SDDataReadyFlag;
extern uint8_t g_u8R3Flag;


void put_rc(FRESULT rc)
{
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");

    uint32_t i;
    for(i = 0; (i != (UINT)rc) && *p; i++)
    {
        while(*p++) ;
    }
    printf(_T("FILE ERR: %s\n"), p);
}


void SDH0_IRQHandler(void)
{
    uint32_t volatile u32Isr;

    /* FMI data abort interrupt */
    if(SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    /* ----- SD interrupt status */
    u32Isr = SDH0->INTSTS;
    if(u32Isr & SDH_INTSTS_BLKDIF_Msk)
    {
        /* Block down */
        g_u8SDDataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if(u32Isr & SDH_INTSTS_CDIF_Msk)    // card detect
    {
        /* ----- SD interrupt status */
        /* it is work to delay 50 times for SD_CLK = 200KHz */
        {
            int volatile i;
            for(i = 0; i < 0x500; i++); /* delay to make sure got updated value from REG_SDISR. */
            u32Isr = SDH0->INTSTS;
        }

        if(u32Isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n  Card removed!\n");
            SD0.IsCardInsert = FALSE;   /* SDISR_CD_Card = 1 means card remove for GPIO mode */
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("  Card insert!\n");
            SDH_Open(SDH0, CardDetect_From_GPIO);
            //SDH_Open(SDH0, CardDetect_From_DAT3);
            SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    /* CRC error interrupt */
    if(u32Isr & SDH_INTSTS_CRCIF_Msk)
    {
        if(!(u32Isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if(!(u32Isr & SDH_INTSTS_CRC7_Msk))
        {
            if(!g_u8R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        /* Clear interrupt flag */
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;
    }

    if(u32Isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    /* Response in timeout interrupt */
    if(u32Isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

volatile uint32_t g_u32IsAES_done = 0;

void CRPT_IRQHandler()
{
    if(AES_GET_INT_FLAG(CRPT))
    {
        g_u32IsAES_done = 1;
        AES_CLR_INT_FLAG(CRPT);
    }
}


#if 0
void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | XT1_OUT_PF2;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | XT1_IN_PF3;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL and set HCLK divider to 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

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

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk; // UART0 Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;  /* clock source is HXT */

    CLK_EnableModuleClock(TMR0_MODULE);

    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Lock protected registers */
    SYS_LockReg();

}
#endif

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long u64Tmr;

    u64Tmr = 0x00000;

    return u64Tmr;
}

int32_t AES_Decrypt(uint32_t *in, uint32_t *out, uint32_t u32Size)
{
    NVIC_EnableIRQ(CRPT_IRQn);
    AES_ENABLE_INT(CRPT);

    XAES_Open(XCRPT, 0, 0, AES_MODE_CFB, AES_KEY_SIZE_256, AES_IN_OUT_SWAP);
    XAES_SetKey(XCRPT, 0, g_au32Key, AES_KEY_SIZE_256);
    XAES_SetInitVect(XCRPT, 0, g_au32IV);
    XAES_SetDMATransfer(XCRPT, 0, (uint32_t)in, (uint32_t)out, (int32_t)u32Size);

    g_u32IsAES_done = 0;
    XAES_Start(XCRPT, 0, CRYPTO_DMA_ONE_SHOT);
    while(g_u32IsAES_done == 0) {}
     
    NVIC_DisableIRQ(CRPT_IRQn);

    return 0;

}


void Cal_SHA256_SD(char *filename, uint32_t au32Hash[8])
{
    int32_t i, addr, bytes, data;
    FIL file;
    FRESULT res;
    uint32_t *pu32;
    uint32_t u32BufSize;
    uint32_t u32DmaOpt;

#ifdef ENCRYPT    
    NVIC_EnableIRQ(CRPT_IRQn);
    AES_ENABLE_INT(CRPT);
    
    XAES_Open(XCRPT, 0, 0, AES_MODE_CFB, AES_KEY_SIZE_256, AES_IN_OUT_SWAP);
    XAES_SetKey(XCRPT, 0, g_au32Key, AES_KEY_SIZE_256);
    XAES_SetInitVect(XCRPT, 0, g_au32IV);
#endif    
    
    
    res = f_open(&file, filename, FA_READ);
    if(res != FR_OK)
    {
        put_rc(res);
        goto lexit;
    }
    bytes = file.obj.objsize;
    
    printf("open file %s,  size = %d, buf size = %d\n", filename, bytes, BUFF_SIZE);

    CRPT->HMAC_CTL = (SHA_MODE_SHA256 << CRPT_HMAC_CTL_OPMODE_Pos) | CRPT_HMAC_CTL_INSWAP_Msk | CRPT_HMAC_CTL_OUTSWAP_Msk;
    CRPT->HMAC_DMACNT = 64;
    CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk;

    /* Start to calculate ... */
    if(bytes <= BUFF_SIZE)
    {
        u32DmaOpt = CRYPTO_DMA_ONE_SHOT;
    }
    else
    {
        /* Need to decode chunk by chunk. */
        u32DmaOpt = 0;
    }
    while(bytes > 0)
    {
        if(u32BufSize == 0)
        {
            /* Read file into buffer */
            u32BufSize = BUFF_SIZE;
            res = f_read(&file, g_au8BuffPool, u32BufSize, &u32BufSize);
            if(res != FR_OK)
            {
                put_rc(res);
                f_close(&file);
                goto lexit;
            }
            pu32 = (uint32_t *)&g_au8BuffPool[0];
            
#ifdef ENCRYPT            
            // Decrypt the data in buffer
            XAES_SetDMATransfer(XCRPT, 0, (uint32_t)pu32, (uint32_t)pu32, (int32_t)u32BufSize);

            if(u32DmaOpt == 0)
            {
                u32DmaOpt = CRYPTO_DMA_FIRST;
            }
            else if((u32DmaOpt == CRYPTO_DMA_FIRST) || (u32DmaOpt == CRYPTO_DMA_CONTINUE))
            {
                if(bytes > BUFF_SIZE)
                {
                    u32DmaOpt = CRYPTO_DMA_CONTINUE;
                }
                else
                {
                    u32DmaOpt = CRYPTO_DMA_LAST;
                }
            }
            
            g_u32IsAES_done = 0;
            XAES_Start(XCRPT, 0, u32DmaOpt);
            while(g_u32IsAES_done == 0) {}

#endif            
            
        }
        
        if(bytes < 64)
            CRPT->HMAC_DMACNT = bytes;

        if(CRPT->HMAC_STS & CRPT_HMAC_STS_DATINREQ_Msk)
        {
            data = *pu32++;
            u32BufSize -= 4;
            addr += 4;
            bytes -= 4;

            if(bytes <= 0)
                bytes = 0;

            /* bytes means remain byte counts */
            if(bytes != 0)
            {
                CRPT->HMAC_DATIN = data;
            }
            else
            {
                /* It's last word ... *-* */
                CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk | CRPT_HMAC_CTL_DMALAST_Msk;
                CRPT->HMAC_DATIN = data;
                while(CRPT->HMAC_STS & CRPT_HMAC_STS_BUSY_Msk);

                for(i = 0; i < 8; i++)
                    au32Hash[i] = *(uint32_t *)((uint32_t) & (CRPT->HMAC_DGST[0]) + (i * 4));
            }
        }
    }
    
    f_close(&file);

#if 0
    printf("Cal_SHA256_Flash:\n    ");
    for(i=0;i<32;i++)
        printf("%02x", *((uint8_t*)au32Hash + i));
    printf("\n");
    printf("Golden Hash:\n    ");
    printf("1DA6EE945766DF6D6A2C3AC94507ACDDD543D86E56DC2333BF001EE1F4D3A7AE\n");
#endif
    
lexit:
    
#ifdef ENCRYPT
    NVIC_DisableIRQ(CRPT_IRQn);
#endif    
    return;
}

/*
fname: Secure image file name
finfo: Secure image information block file name
NS:    Flag to indicate it is secure or nonsecure image
*/
int32_t FwUpdate(char *fname, char *finfo, int32_t NS)
{
    uint8_t *pu8Buff;
    FIL file1, file2;             /* File objects */
    TCHAR sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */
    FRESULT res;
    int32_t i;
    uint32_t u32Size, u32Addr;
    uint32_t u32Cnt;
    uint32_t hash[8] = {0};
    uint8_t *pu8;
    FW_INFO_T fwinfo = {0};
    uint32_t au32Pk1[8], au32Pk2[8];
    uint32_t u32DmaOpt;
    uint32_t u32SecureCnt;
    
    pu8Buff = (BYTE *)g_au8BuffPool;
// ------------------------------------------------------------
// Get firmware information
    res = f_open(&file1, finfo, FA_READ);
    if(res != FR_OK)
    {
        put_rc(res);
        return -1;
    }
    u32Size = file1.obj.objsize;
    printf("Open file %s, size = %d\n",finfo, u32Size);
    
    /* Simple size check to check if the information block is valid or not. */
    if(u32Size != sizeof(fwinfo))
    {
        f_close(&file1);
        printf("ERR: Firmware information file is not correct!\n");
        return -1;
    }
        
    u32Cnt = u32Size;
    res = f_read(&file1, pu8Buff, u32Cnt, &u32Cnt);
    if(res != FR_OK)
    {
        put_rc(res);
        return -1;
    }
    
    f_close(&file1);
        
#ifdef ENCRYPT
    // We need to decrypt the firmware informaton
    AES_Decrypt((uint32_t *)pu8Buff, (uint32_t *)&fwinfo, sizeof(fwinfo));
#else
    memcpy(&fwinfo, pu8Buff, sizeof(fwinfo));
#endif    
    
    // Calculate firmware hash
    Cal_SHA256_SD(fname, hash);

    SwapCpy((uint8_t *)&au32Pk1[0], (uint8_t *)&fwinfo.pubkey.au32Key0[0], sizeof(au32Pk1));
    SwapCpy((uint8_t *)&au32Pk2[0], (uint8_t *)&fwinfo.pubkey.au32Key1[0], sizeof(au32Pk2));
    
    // verify the signature
    if(VerifyNuBL3x(&fwinfo, au32Pk1, au32Pk2, hash) < 0)
    {
        printf("%s verify ..................... FAILED!\n", fname);
        return -1;
    }
    else
    {
        printf("%s verify ..................... PASSED!\n", fname);

        //-------------------------------------------------------------        
        // firmware udpate
        
        // Check Secure Count
        u32SecureCnt = M32((uint32_t)&g_NuBL32InfoStart+0x5c);
        memcpy((void *)&fwinfo, (void *)&g_NuBL32InfoStart, sizeof(FW_INFO_T));
        if(fwinfo.mData.au32ExtInfo[1] < u32SecureCnt)
        {
            printf("Secure count check ............ FAILED!\n");
            return -1;
        }
         
        // Check the region to avoid overlap secure boot
        if(fwinfo.mData.au32FwRegion[0].u32Size)
        {
            if(fwinfo.mData.au32FwRegion[0].u32Start < 0x8000)
                return -1;
        }
        if(fwinfo.mData.au32FwRegion[1].u32Size)
        {
            if(fwinfo.mData.au32FwRegion[1].u32Start < 0x8000)
                return -1;
        }
        
        // Check secure/nonsecure region according to NS flag
        if(NS)
        {
            if(fwinfo.mData.au32FwRegion[0].u32Size)
            {
                if(fwinfo.mData.au32FwRegion[0].u32Start < 0x10040000)
                    return -1;
            }
            if(fwinfo.mData.au32FwRegion[1].u32Size)
            {
                if(fwinfo.mData.au32FwRegion[1].u32Start < 0x10040000)
                    return -1;
            }
        }

        
        //------ Update information block ------
        FMC_Open();
        FMC_ENABLE_AP_UPDATE();
        
        // Backup information block page (secure / non-secure use the same page)
        memcpy(g_au8BuffPool, (void *)0x7800, 0x800);
        if(NS)
        {
            // Update nonsecure information block
            memcpy(&g_au8BuffPool[sizeof(FW_INFO_T)], &fwinfo, sizeof(FW_INFO_T));
        }
        else
        {
            // Update secure information block
            memcpy(&g_au8BuffPool[0], &fwinfo, sizeof(FW_INFO_T));
        }
        // Write new information block to flash
        FMC_Erase(0x7800);
        for(i=0;i<0x800;i+=4)
        {
            FMC_Write(0x7800+i, M32((uint32_t)&g_au8BuffPool[i]));
        }
        
        
        
        //------ Update firmware image ------
        res = f_open(&file1, fname, FA_READ);
        if(res != FR_OK)
        {
            put_rc(res);
            return -1;
        }
        
#ifdef ENCRYPT    
        NVIC_EnableIRQ(CRPT_IRQn);
        AES_ENABLE_INT(CRPT);
        
        XAES_Open(XCRPT, 0, 0, AES_MODE_CFB, AES_KEY_SIZE_256, AES_IN_OUT_SWAP);
        XAES_SetKey(XCRPT, 0, g_au32Key, AES_KEY_SIZE_256);
        XAES_SetInitVect(XCRPT, 0, g_au32IV);
#endif    
        
        
        u32Size = fwinfo.mData.au32FwRegion[0].u32Size ;
        printf("Open file %s, size = %d\n",fname, u32Size);
        printf("updateing ");
        
        if(u32Size <= BUFF_SIZE)
        {
            u32DmaOpt = CRYPTO_DMA_ONE_SHOT;
        }
        else
        {
            /* Need to decode chunk by chunk. */
            u32DmaOpt = 0;
        }
        
        /* Update region 0 */
        u32Addr = fwinfo.mData.au32FwRegion[0].u32Start;
        while(u32Size > 0)
        {
            u32Cnt = u32Size;
            if(u32Cnt > BUFF_SIZE)
                u32Cnt = BUFF_SIZE;

            res = f_read(&file1, pu8Buff, u32Cnt, &u32Cnt);
            if(res != FR_OK)
            {
                put_rc(res);
                return -1;
            }
             
#ifdef ENCRYPT            
            // Decrypt the data in buffer
            XAES_SetDMATransfer(XCRPT, 0, (uint32_t)pu8Buff, (uint32_t)pu8Buff, (int32_t)u32Cnt);

            if(u32DmaOpt == 0)
            {
                u32DmaOpt = CRYPTO_DMA_FIRST;
            }
            else if((u32DmaOpt == CRYPTO_DMA_FIRST) || (u32DmaOpt == CRYPTO_DMA_CONTINUE))
            {
                if(u32Size > BUFF_SIZE)
                {
                    u32DmaOpt = CRYPTO_DMA_CONTINUE;
                }
                else
                {
                    u32DmaOpt = CRYPTO_DMA_LAST;
                }
            }
            
            g_u32IsAES_done = 0;
            XAES_Start(XCRPT, 0, u32DmaOpt);
            while(g_u32IsAES_done == 0) {}

#endif            
            
            /* Write all data in buffer */
            for(i=0;i<u32Cnt;i+=4)
            {
                /* Erase page */
                if((u32Addr&0x7ff) == 0)
                {
                    FMC_Erase(u32Addr);
                    printf(".");
                }
                
                /* Programming data */
                FMC_Write(u32Addr, *((uint32_t *)&pu8Buff[i]));
                u32Addr += 4;
                
            }
            u32Size -= u32Cnt;
        }
        
        printf(" Done\n");
        
        
        /* Update Region 1 if necessary*/
        if(fwinfo.mData.au32FwRegion[1].u32Size > 0)
        {
            /* Seek to the end or region 0 */
            f_lseek(&file1, fwinfo.mData.au32FwRegion[0].u32Size);
            
            u32Size = fwinfo.mData.au32FwRegion[1].u32Size ;
            printf("updateing region 1 ");
            
            if(u32Size <= BUFF_SIZE)
            {
                u32DmaOpt = CRYPTO_DMA_ONE_SHOT;
            }
            else
            {
                /* Need to decode chunk by chunk. */
                u32DmaOpt = 0;
            }
            
            /* Update region 0 */
            u32Addr = fwinfo.mData.au32FwRegion[1].u32Start;
            while(u32Size > 0)
            {
                u32Cnt = u32Size;
                if(u32Cnt > BUFF_SIZE)
                    u32Cnt = BUFF_SIZE;

                res = f_read(&file1, pu8Buff, u32Cnt, &u32Cnt);
                if(res != FR_OK)
                {
                    put_rc(res);
                    return -1;
                }
                 
    #ifdef ENCRYPT            
                // Decrypt the data in buffer
                XAES_SetDMATransfer(XCRPT, 0, (uint32_t)pu8Buff, (uint32_t)pu8Buff, (int32_t)u32Cnt);

                if(u32DmaOpt == 0)
                {
                    u32DmaOpt = CRYPTO_DMA_FIRST;
                }
                else if((u32DmaOpt == CRYPTO_DMA_FIRST) || (u32DmaOpt == CRYPTO_DMA_CONTINUE))
                {
                    if(u32Size > BUFF_SIZE)
                    {
                        u32DmaOpt = CRYPTO_DMA_CONTINUE;
                    }
                    else
                    {
                        u32DmaOpt = CRYPTO_DMA_LAST;
                    }
                }
                
                g_u32IsAES_done = 0;
                XAES_Start(XCRPT, 0, u32DmaOpt);
                while(g_u32IsAES_done == 0) {}

    #endif            
                
                /* Write all data in buffer */
                for(i=0;i<u32Cnt;i+=4)
                {
                    /* Erase page */
                    if((u32Addr&0x7ff) == 0)
                    {
                        FMC_Erase(u32Addr);
                        printf(".");
                    }
                    
                    /* Programming data */
                    FMC_Write(u32Addr, *((uint32_t *)&pu8Buff[i]));
                    u32Addr += 4;
                    
                }
                u32Size -= u32Cnt;
            }
                
            printf(" Done\n");
        }
        
        f_close(&file1);
    }
    
    
#ifdef ENCRYPT
    NVIC_DisableIRQ(CRPT_IRQn);
#endif    
    
    return 0;
}




/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t UpgradeProcess(void)
{
    uint8_t *pu8Buff;
    FIL file1, file2;             /* File objects */
    TCHAR sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */
    FRESULT res;
#ifndef ENCRYPT    
    char *filename = "tfm_ns.bin";
    char *infoname = "tfm_ns_i.bin";
    char *filename_s = "tfm_s_c.bin";
    char *infoname_s = "tfm_s_i.bin";
    
#else
    char *filename = "tfm_ns.enc";
    char *infoname = "tfm_ns_i.enc";
    char *filename_s = "tfm_s_c.enc";
    char *infoname_s = "tfm_s_i.enc";
#endif    
    int32_t i;
    uint32_t u32Size, u32Addr;
    uint32_t u32Cnt;
    uint32_t hash[8] = {0};
    uint8_t *pu8;

    pu8Buff = (BYTE *)g_au8BuffPool;

    printf("\n\nM2351 SDH FATFS TEST!\n");
    /* Enable NVIC SDH0 IRQ */
    NVIC_EnableIRQ(SDH0_IRQn);
    
    /*
        SD initial state needs 400KHz clock output, driver will use HIRC for SD initial clock source.
        And then switch back to the user's setting.
    */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    //SDH_Open_Disk(SDH0, CardDetect_From_DAT3);
    
    f_chdrive(sd_path);          /* set default path */
    
    while(!SDH_CardDetection(SDH0)){}
    
    
    // Check NSCBA
    if(SCU->FNSADDR != NSCBA)
    {
        printf("Nonsecure boundary setting is not match!\n");
        goto lexit;
    }

//-------------------------------------------------------------        
// tfm-s udpate
    if(FwUpdate(filename_s, infoname_s, 0) < 0)
        goto lexit;

    
//-------------------------------------------------------------        
// tfm-ns udpate
    
    if(FwUpdate(filename, infoname, 1) < 0)
        goto lexit;

    

    
lexit:
    
    for(;;){__WFI();}
}





/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
