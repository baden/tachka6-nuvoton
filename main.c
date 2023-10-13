#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
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

bool led_state = false;

volatile bool bus_pause = false;
static __INLINE void restart_pause_detector_timer()
{
    TIMER_Stop(TIMER0);
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 333);
    TIMER_EnableInt(TIMER0);
    // TIMER_ResetCounter(TIMER0);
    TIMER_Start(TIMER0);
    bus_pause = false;
}

void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
        bus_pause = true;

                led_state = !led_state;
                if(led_state) {
                    PA10 = 1;
                } else {
                    PA10 = 0;
                }
        
    }
}


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

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, 0); // HCLK, HIRC

    /* Enable UART modules clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
  
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 and UART1 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk |
                    SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD |
                    SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD);

    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA8_Msk | SYS_GPA_MFP_PA9_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA8_I2C0_SDA | SYS_GPA_MFP_PA9_I2C0_SCL;
}


uint8_t bus_data[32] = {0};
size_t bus_data_len = 0;

volatile int bus_packet_counter = 0;

enum bus_state {
    BUS_GET_LENGTH,
    BUS_GET_DATA,
    BUS_GET_CRCL,
    BUS_GET_CRCH,
    BUS_DISCARD
};

enum bus_state bus_state = BUS_GET_LENGTH;

uint16_t axes[8] = {0};
volatile bool axes_updated = false;

void parseBusCommand(uint8_t cmd, uint8_t *data, size_t data_len)
{
    if(cmd == 0x40) {
        axes[0] = data[0] | (data[1] << 8);
        axes[1] = data[2] | (data[3] << 8);
        axes[2] = data[4] | (data[5] << 8);
        axes[3] = data[6] | (data[7] << 8);
        axes[4] = data[8] | (data[9] << 8);
        axes[5] = data[10] | (data[11] << 8);
        axes[6] = data[12] | (data[13] << 8);
        axes[7] = data[14] | (data[15] << 8);
        axes_updated = true;
    }
    // printf("Bus command: %02x \t", cmd);
    // for(int i = 0; i < data_len; i++) {
    //     printf("%02x ", data[i]);
    // }
    printf("\r");
}

// <len><cmd><data....><chkl><chkh>

void parseBusPacket(uint8_t b)
{
    static uint8_t buffer[32] = {0};
    static size_t len = 0;
    static size_t ptr = 0;
    static uint16_t chksum = 0;

    if(bus_pause) {
        bus_state = BUS_GET_LENGTH;
    }
    restart_pause_detector_timer();

    switch(bus_state) {
        case BUS_GET_LENGTH:
            // Only max 0x20 is allowed
            if(b <= 0x20) {
                ptr = 0;
                len = b - 3;
                chksum = 0xFFFF - b;
                bus_state = BUS_GET_DATA;
            } else {
                bus_state = BUS_DISCARD;
            }
            break;
        case BUS_GET_DATA:
            buffer[ptr++] = b;
            chksum -= b;
            if(ptr >= len) {
                bus_state = BUS_GET_CRCL;
            }
            break;
        case BUS_GET_CRCL:
            if((chksum & 0xFF) == b) {
                bus_state = BUS_GET_CRCH;
            } else {
                bus_state = BUS_DISCARD;
            }
            break;
        case BUS_GET_CRCH:
            if(((chksum >> 8) & 0xFF) == b) {
                // Packet is valid
                parseBusCommand(buffer[0], buffer + 1, len);
                bus_state = BUS_GET_LENGTH;
                // bus_state = BUS_DISCARD;
            } else {
                bus_state = BUS_DISCARD;
            }
            break;
        case BUS_DISCARD:
            break;
        default:
            break;
    }
}

void UART1_IRQHandler(void)
{
    volatile __attribute__((unused)) uint32_t u32IntSts = UART1->ISR;
    volatile uint8_t u8InChar;

    /* Rx Ready or Time-out INT */
    if(UART_GET_INT_FLAG(UART1, UART_ISR_RDA_INT_Msk) || UART_GET_INT_FLAG(UART1, UART_ISR_TOUT_INT_Msk)) {
        /* Handle received data */
        u8InChar = UART_READ(UART1);

        bus_packet_counter++;

        parseBusPacket(u8InChar);

        // if(bus_data_len < 32) {
        //     bus_data[bus_data_len++] = UART_READ(UART1);
        // } else {
        //     // Read UART1 to clear interrupt flag
        //     UART_READ(UART1);
        // }

    }
}


void UART0_Init()
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void UART1_Init()
{
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);
    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
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

#define MCP4725_SLAVE_ADDR1          0x60                       /*!< slave address for MCP4725 sensor. Not 0x62? */
#define MCP4725_SLAVE_ADDR2          0x61               /*!< slave address for MCP4725 sensor */
#define MCP4725_WRITE_FAST_MODE     0x00                        /*!< MCP4725 fast write command */
#define MCP4725_WRITE_DAC           0x60                       /*!< MCP4725 DAC write command */


void mcp4725_set_value(uint16_t value, bool channel)
{
    if(value > 4095) value = 4095;
    

    /* Send START */
    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    /* Send device address */
    I2C_SET_DATA(I2C0, (channel ? MCP4725_SLAVE_ADDR2 : MCP4725_SLAVE_ADDR1) << 1);
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

    /* Open Timer0 in one shot mode, ~3msec */
    // TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    /* Reset Timer0 and Start Timer0 counting */


    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for FS-IA6B receiver bus */
    UART1_Init();
    UART_ENABLE_INT(UART1, UART_IER_RDA_IEN_Msk);
    NVIC_EnableIRQ(UART1_IRQn);

    /* Init I2C0 */
    I2C0_Init();

    printf("+------------------------+\n");
    printf("|    Tachka 6 driver     |\n");
    printf("+------------------------+\n\n");

    GPIO_SetMode(PA, BIT10, GPIO_PMD_OUTPUT);
    PA10 = 1;

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
        if(axes_updated) {
            axes_updated = false;
            printf("Axes:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r", axes[0], axes[1], axes[2], axes[3], axes[4], axes[5], axes[6], axes[7]);

            // Transform axes[4] 1000..2000 to 0..4096
            int a_value = (axes[4] - 1000) * 4096 / 1000;
            int b_value = (axes[5] - 1000) * 4096 / 1000;

            mcp4725_set_value(a_value, false);
            mcp4725_set_value(b_value, true);
        }


        // printf("Input: ");
        // ch = getchar();
        // printf("%c\n", ch);
        // PA10 = 1;
        // Delay(1000000);
        // PA10 = 0;
        // Delay(1000000);
        // printf("Task tick. %d (bus: %d)\n", counter, bus_packet_counter);
        // counter += 50;

        if(counter >= 4096) counter = 0;
        // UART_WRITE(UART0, 'a');

    }
    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/