/*
 * SPI.c
 *
 *  Created on: Jul 30, 2020
 *      Author: kartikeya
 */
#include "Lab.h"                // Main include file
#include "F2837xD_device.h"



//#define LTC1864 1 //if LTC1864 16-bit 250 KSPS SAR ADC IC is being used
#define LTC2338 2 //if LTC2338 18-bit 1 MSPS SAR ADC IC is being used

#define SPI_INT_ENABLE 1   //Comment to disable SPI interrupts

//Data rate selection
//#define DataRate_1_7_MBPS 1     //14.28 MHz Serial Clock
//#define DataRate_2_0_MBPS 1   //16.66 MHz Serial Clock
#define DataRate_2_5_MBPS 1   //20 MHz Serial Clock
//#define DataRate_6_25_MBPS 1 //50 MHz Serial Clock //Caution!! LTC1864 will get damaged at this clock rate!! LTC2338 Wont :)

#define WORDS 2

__interrupt void spiarx_isr(void);


uint32_t temp;
uint16_t rxfifobuf[WORDS];
uint32_t ADC_18bitval;
uint16_t rxdata1;
int i;

void InitSPI(void)
{

#if LTC1864
    SpiaRegs.SPICCR.bit.SPISWRESET = 0; //Reset the SPI peripheral
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1; //Configure C2000 as an SPI master

    //SpiaRegs.SPICCR.bit.SPILBK = 1; //Disable Loopback

    //SpiaRegs.SPICTL.bit.TALK = 1;
    //Referring to LT2338 Clock Polarity is 0: ie, clock signal is zero when no transfers are taking place

    //The clock phase refers to which clock edge the data is captured on and which clock edge does the data change.
    //Refer to your microcontroller’s data sheet for the bit value for the clock phase.
    //According to this and referring to the data transfer waveform diagram provided in the datasheet
    //of LT2338 ADC IC it is found that the data has to be stable at the falling edge and can change at the
    // rising edge.
    //Referring to Table 18-3 on Pg 2060 of the technical refernce manual of F28379D, the appropriate settings is
    //Rising edge without delay
    //i.e. CLKPOLARITY 0 and CLK_PHASE 0
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; //Based on the previous explanation
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0; //Based on the previous explanation

    //Baud rate settings
    //Core clock at 200MHz
    //LSPCLK @100MHz
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 1; //LSPCLK = / 2 (default on reset)
    EDIS;
/*
  SPI Baud Rate =    LSPCLK         (Formula for calculating the baud rate)
                  ------------
                   SPIBRR + 1
*/
#if DataRate_2_5_MBPS
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x04; //SPI Baud Rate = LSPCLK/7 (20 MHz)
#endif

#if DataRate_2_0_MBPS
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x05; //SPI Baud Rate = LSPCLK/7 (16.66 MHz)
#endif

#if DataRate_1_7_MBPS
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x06; //SPI Baud Rate = LSPCLK/7 (14.28 MHz)
#endif
    SpiaRegs.SPICCR.bit.SPICHAR = 0xF; //16-bit word

    //Clear the flags
    SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1; //clear the SPI Receiver Overrun Flag

#if SPI_INT_ENABLE
    //Interrupt Settings
    SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0x1; //Enable Receiver Overrun
    SpiaRegs.SPICTL.bit.SPIINTENA = 0x1; //Enable SPI Interrupt
#endif

    //Reset Release
    SpiaRegs.SPICCR.bit.SPISWRESET = 1; //Release the SPI peripheral from RESET State
#endif


#if LTC2338
    //Setup SPI to do two 9-bit FIFO transfers so that it subsequently receives 9 bits over SPI receive
    //The transmits are dummy; the receives contain the data

    SpiaRegs.SPICCR.bit.SPISWRESET = 0; //Reset the SPI peripheral
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1; //Configure C2000 as an SPI master

    //The clock phase refers to which clock edge the data is captured on and which clock edge does the data change.
    //Refer to your microcontroller’s data sheet for the bit value for the clock phase.
    //According to this and referring to the data transfer waveform diagram provided in the datasheet
    //of LT2338 ADC IC it is found that the data has to be stable at the falling edge and can change at the
    // rising edge.
    //Referring to Table 18-3 on Pg 2060 of the technical refernce manual of F28379D, the appropriate settings is
    //Rising edge without delay
    //i.e. CLKPOLARITY 0 and CLK_PHASE 0
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; //Based on the previous explanation
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0; //Based on the previous explanation

    //Baud rate settings
    //Core clock at 200MHz
    //LSPCLK @100MHz
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 1; //LSPCLK = / 2 (default on reset)
    EDIS;
#if DataRate_6_25_MBPS
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x01; //SPI Baud Rate = LSPCLK/2 (50 MHz)
#endif

#if DataRate_2_5_MBPS
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x04; //SPI Baud Rate = LSPCLK/7 (20 MHz)
#endif

#if DataRate_2_0_MBPS
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x05; //SPI Baud Rate = LSPCLK/7 (16.66 MHz)
#endif

#if DataRate_1_7_MBPS
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 0x06; //SPI Baud Rate = LSPCLK/7 (14.28 MHz)
#endif

    //Clear the flags
    SpiaRegs.SPISTS.bit.OVERRUN_FLAG = 1; //clear the SPI Receiver Overrun Flag

#if SPI_INT_ENABLE
    //Interrupt Settings
    SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0x1; //Enable Receiver Overrun
    SpiaRegs.SPICTL.bit.SPIINTENA = 0x1; //Enable SPI Interrupt

    EALLOW;
       PieVectTable.SPIA_RX_INT = &spiarx_isr;
       IER |= 0x0020; //Enable INT6
       PieCtrlRegs.PIEIER6.bit.INTx1 = 1; //Enable PIE Group 3's Channel 1 (Which happens to be the ePWM1 )
                  //PieCtrlRegs.PIEIFR3.bit.INTx1 = 0; //Interrupt flag is cleared. Ready to be set by the interrupt peripheral
#endif
//--------------------------------------------------------------
    //SPI FIFO Settings

    //---------------------------
    //SPIFFTX register Settings
    //---------------------------
    //Reset the SPI transmit and receive channels. The SPI FIFO register configuration bits will be left as is.
    SpiaRegs.SPIFFTX.bit.SPIRST = 0;

    //reset the FIFO pointer to zero, and hold in reset
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;



    //TXFIFO Interrupt Clear; 1h (R/W) = Write 1 to clear SPIFFTX[TXFFINT] flag
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

    //Disable TX interrupt
    SpiaRegs.SPIFFTX.bit.TXFFIENA = 0;

    //SPI FIFO Enhancements Enable; SPI FIFO enhancements are enabled.
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;

    //TX FIFO Reset; Release transmit FIFO from reset
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;

    //SPI FIFO can resume transmit or receive. No effect to the SPI registers bits
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;


    //---------------------------
    //SPIFFRX register Settings
    //---------------------------


    //reset the FIFO pointer to zero, and hold in reset.
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;

    //Receive FIFO Interrupt Clear; Write 1 to clear SPIFFRX[RXFFINT] flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;

    //RX FIFO interrupt based on RXFFIL match (greater than or equal to) will be enabled
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

    //Bit 14: , RXFFOVFCLR; reset the overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;

    //Bit 5: Enable RX FIFO Interrupts
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

    //Bit 0-4: RXFFIL: Trigger interrupt after two receives
    SpiaRegs.SPIFFRX.bit.RXFFIL = 2;

    //Receive FIFO Reset; Re-enable receive FIFO operation.
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;


//-----------------------------------------------------------

    SpiaRegs.SPIFFCT.all = 0x0; //SPIFFCT = 0000 0000 0000 0000

    SpiaRegs.SPICCR.bit.SPICHAR = 0x8; //9-bit word


    //Reset Release
    SpiaRegs.SPICCR.bit.SPISWRESET = 1; //Release the SPI peripheral from RESET State




#endif

}

#if SPI_INT_ENABLE

__interrupt void spiarx_isr(void)
{
    static volatile Uint16 GPIO31_count = 0;            // Counter for pin toggle

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;         // Must acknowledge the PIE group
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;  //Clear the interrupt flag

    //Read the RXFIFO
    for (i=0;i<WORDS;i++)
    {
        rxfifobuf[i] = SpiaRegs.SPIRXBUF;
    }

    temp = rxfifobuf[0];
    ADC_18bitval |= (temp<<8);
    temp = rxfifobuf[1];
    ADC_18bitval |= (temp<<0);


    if(GPIO31_count++ > 25000)                  // Toggle slowly to see the LED blink
                  {
                      GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  // Toggle the pin
                      GPIO31_count = 0;                       // Reset the counter
                  }
}
#endif
