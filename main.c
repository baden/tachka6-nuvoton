#include <stdio.h>
#include <stdint.h>
#include "NUC100Series.h"
#include "uart.h"
#include "i2c.h"

#define PLL_CLOCK           50000000

// void SYS_Init(void)
// {
//     /* Enable IP clock */
//     CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

//     /* Update System Core Clock */
//     /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
//     SystemCoreClockUpdate();

//     /* Set GPB multi-function pins for UART0 RXD and TXD */
//     SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
//     SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
// }

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

  
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA8_Msk | SYS_GPA_MFP_PA9_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA8_I2C0_SDA | SYS_GPA_MFP_PA9_I2C0_SCL;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    // printf("I2C clock %ld Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Enable I2C interrupt */
    // I2C_EnableInt(I2C0);
    // NVIC_EnableIRQ(I2C0_IRQn);
}

#define MCP4725_SLAVE_ADDR          0x60                       /*!< slave address for MCP4725 sensor. Not 0x62? */
#define MCP4725_WRITE_FAST_MODE     0x00                       /*!< MCP4725 fast write command */
#define MCP4725_WRITE_DAC           0x60                       /*!< MCP4725 DAC write command */


void mcp4725_set_value(uint16_t value)
{
    if(value > 4095) value = 4095;
    

    /* Send START */
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    /* Send device address */
    I2C_SET_DATA(I2C0, MCP4725_SLAVE_ADDR << 1);
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    I2C_WAIT_READY(I2C0);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C0, (uint8_t)(MCP4725_WRITE_FAST_MODE | (value >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    I2C_WAIT_READY(I2C0);

    /* Send data */
    I2C_SET_DATA(I2C0, (uint8_t)(value & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    I2C_WAIT_READY(I2C0);

    /* Send STOP */
    I2C_STOP(I2C0);

}

void Delay(int count)
{
    volatile uint32_t i;
    for(i = 0; i < count ; i++) {
        __NOP();
        __NOP();
    }
}

static void SendChar_ToUART(int ch)
{
    while (UART0->FSR & UART_FSR_TX_FULL_Msk);

    UART0->DATA = ch;
    if(ch == '\n')
    {
        while (UART0->FSR & UART_FSR_TX_FULL_Msk);
        UART0->DATA = '\r';
    }
}

// static void PutString(char *str)
// {
//     while (*str != '\0')
//     {
//         SendChar_ToUART(*str++);
//     }
// }

int _write(int file, char *ptr, int len)
{
    for(int idx = 0; idx < len; idx++)
    {
        SendChar_ToUART(*ptr++);
    }
    return len;
}

int main()
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init I2C0 */
    I2C0_Init();

    printf("+------------------------+\n");
    printf("|    Tachka 6 driver     |\n");
    printf("+------------------------+\n\n");

    GPIO_SetMode(PA, BIT10, GPIO_PMD_OUTPUT);
    PA10 = 0;

    // int8_t ch;

    // /* Unlock protected registers */
    // SYS_UnlockReg();

    // SYS_Init();

    // /* Lock protected registers */
    // SYS_LockReg();

    // /* Init UART0 to 115200-8n1 for print message */
    // // UART_Open(UART0, 115200);

    // // printf("Simple Demo Code\n\n");

    // // printf("Please Input Any Key\n\n");

    int counter = 0;

    do
    {
        // printf("Input: ");
        // ch = getchar();
        // printf("%c\n", ch);
        PA10 = 1;
        Delay(1000000);
        PA10 = 0;
        Delay(1000000);
        printf("Task tick. %d\n", counter);
        counter += 50;
        mcp4725_set_value(counter);
        if(counter >= 4096) counter = 0;
        // UART_WRITE(UART0, 'a');

    }
    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/