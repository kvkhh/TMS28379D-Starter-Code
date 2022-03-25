/**********************************************************************
* File: EPwm.c -- Lab File
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"				// Main include file

#define DEADBANDS_ACTIVE 1
#define ACTIVATE_INTERRUPTS 1
//#define LTC1864 1 //if LTC1864 16-bit 250KHz SAR ADC IC is being used
#define LTC2338 2 //if LTC2338 18-bit 1MSPS SAR ADC IC is being used


__interrupt void epwm1_isr(void);

//uint16_t rxdata1;
//uint16_t rxfifobuf[WORDS];
//uint32_t ADC_18bitval;
//uint32_t temp;
//int i;
/**********************************************************************
* Function: InitEPwm()
*wy
* Description: Initializes the Enhanced PWM modules on the F28x7x
**********************************************************************/
void InitEPwm(void)
{
asm(" EALLOW");						// Enable EALLOW protected register access

	// Configure the prescaler to the ePWM modules.  Max ePWM input clock is 100 MHz.
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;		// EPWMCLK divider from PLLSYSCLK.  0=/1, 1=/2

	// Must disable the clock to the ePWM modules if you want all ePWM modules synchronized.
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

	asm(" EDIS");						// Disable EALLOW protected register access


	//=====================================================================
	// Config
	// Initialization Time
	//========================
	// EPWM Module 1 config
	EPwm1Regs.TBPRD = 500;
	// Period = 500 TBCLK counts
	EPwm1Regs.TBPHS.bit.TBPHS = 0;

	//EPwm1Regs.TBCTL.bit.PHSDIR = 0; //count down after sync
	// Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = 3; //stop timer

	// Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = 0; //enable phase loading

	EPwm1Regs.TBCTL.bit.PHSDIR = 0; //count down after sync
	// Master module
	EPwm1Regs.TBCTL.bit.PRDLD = 0; //load on CTR = 0
	EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; //CTR = 0
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; //using 100MHz clock

	// Sync down-stream module
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0; //shadow updates
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0; //shadow updates
	EPwm1Regs.CMPCTL.bit.LOADAMODE = 2; //load on CTR = PRD or CTR = 0
	// load on CTR=Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = 2; //load on CTR = PRD or CTR = 0
	// load on CTR=Zero
	EPwm1Regs.AQCTLA.bit.CAU = 1;  //clear on compare
	// set actions for EPWM1A
	EPwm1Regs.AQCTLA.bit.PRD = 2; //set on ctr = PRD

	EPwm1Regs.AQCTLB.bit.CAU = 2;  //clear on compare
	// set actions for EPWM1A
	EPwm1Regs.AQCTLB.bit.PRD = 1; //set on ctr = PRD

#if DEADBANDS_ACTIVE
    // enable Dead-band module
    EPwm1Regs.DBCTL.bit.POLSEL = 2; //Active high (AH) mode. Neither EPWMxA nor EPWMxB is inverted (default)
    EPwm1Regs.DBCTL.bit.IN_MODE = 0; //EPWMxA In (from the action-qualifier) is the source for both falling-edge and rising-edge delay.
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm1Regs.DBCTL.bit.SHDWDBREDMODE = 1;
    EPwm1Regs.DBCTL.bit.SHDWDBFEDMODE = 1;
    EPwm1Regs.DBCTL.bit.LOADREDMODE = 0;    // Load on Counter == 0
    EPwm1Regs.DBCTL.bit.LOADFEDMODE = 0;    // Load on Counter == 0nt:
    EPwm1Regs.DBCTL.bit.HALFCYCLE = 1;
    EPwm1Regs.DBRED.bit.DBRED = 34;
    EPwm1Regs.DBREDHR.bit.DBREDHR = 0x0;
    EPwm1Regs.DBFED.bit.DBFED = 34;
    EPwm1Regs.DBFEDHR.bit.DBFEDHR = 0x0;
#endif

    //Interrupt settings
    //Interrupt from EPWM-1A
#if ACTIVATE_INTERRUPTS
    EPwm1Regs.ETSEL.bit.INTSEL = 011; // Enable event time-base counter equal to zero or period
    EPwm1Regs.ETSEL.bit.INTEN = 1; // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = 01; // Generate INT on 1st event

          EALLOW;
          PieVectTable.EPWM1_INT = &epwm1_isr;
          IER |= 0x0004; //Enable INT3
          PieCtrlRegs.PIEIER3.bit.INTx1 = 1; //Enable PIE Group 3's Channel 1 (Which happens to be the ePWM1 )
            //PieCtrlRegs.PIEIFR3.bit.INTx1 = 0; //Interrupt flag is cleared. Ready to be set by the interrupt peripheral
#endif

	//EPWM2 Configurations
	    EPwm2Regs.TBPRD = 500;
	    // Period = 900 TBCLK counts
	    EPwm2Regs.TBPHS.bit.TBPHS = 333;
	    // Set Phase register to zero
	    EPwm2Regs.TBCTL.bit.CTRMODE = 3; //stop timer
	    // Symmetrical mode
	    EPwm2Regs.TBCTL.bit.PHSEN = 1; //enable phase loading

	    EPwm2Regs.TBCTL.bit.PHSDIR = 0; //count down after sync
	    // Master module
	    EPwm2Regs.TBCTL.bit.PRDLD = 0; //load on CTR = 0
	    EPwm2Regs.TBCTL.bit.SYNCOSEL = 1; //CTR = 0
	    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; //using 100MHz clock

	    // Sync down-stream module
	    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0; //shadow updates
	    EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0; //shadow updates
	    EPwm2Regs.CMPCTL.bit.LOADAMODE = 2; //load on CTR = PRD or CTR = 0
	    // load on CTR=Zero
	    EPwm2Regs.CMPCTL.bit.LOADBMODE = 2; //load on CTR = PRD or CTR = 0
	    // load on CTR=Zero
	    EPwm2Regs.AQCTLA.bit.CAU = 1;  //clear on compare
	    // set actions for EPWM1A
	    EPwm2Regs.AQCTLA.bit.PRD = 2; //set on ctr = PRD

	    // load on CTR=Zero
	    EPwm2Regs.AQCTLB.bit.CAU = 2;  //clear on compare
	    // set actions for EPWM1A
	    EPwm2Regs.AQCTLB.bit.PRD = 1; //set on ctr = PRD

#if DEADBANDS_ACTIVE
    // enable Dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL = 2; //Active high (AH) mode. Neither EPWMxA nor EPWMxB is inverted (default)
    EPwm2Regs.DBCTL.bit.IN_MODE = 0; //EPWMxA In (from the action-qualifier) is the source for both falling-edge and rising-edge delay.
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm2Regs.DBCTL.bit.SHDWDBREDMODE = 1;
    EPwm2Regs.DBCTL.bit.SHDWDBFEDMODE = 1;
    EPwm2Regs.DBCTL.bit.LOADREDMODE = 0;    // Load on Counter == 0
    EPwm2Regs.DBCTL.bit.LOADFEDMODE = 0;    // Load on Counter == 0
    EPwm2Regs.DBCTL.bit.HALFCYCLE = 1;
    EPwm2Regs.DBRED.bit.DBRED = 34;
    EPwm2Regs.DBREDHR.bit.DBREDHR = 0x0;
    EPwm2Regs.DBFED.bit.DBFED = 34;
    EPwm2Regs.DBFEDHR.bit.DBFEDHR = 0x0;
#endif

	//PWM3 Configuration
	    // EPWM Module 1 config
	        EPwm3Regs.TBPRD = 500;
	        // Period = 900 TBCLK counts
	        EPwm3Regs.TBPHS.bit.TBPHS = 333;
	        // Set Phase register to zero
	        EPwm3Regs.TBCTL.bit.CTRMODE = 3; //stop timer
	        // Symmetrical mode
	        EPwm3Regs.TBCTL.bit.PHSEN = 1; //enable phase loading

	        EPwm3Regs.TBCTL.bit.PHSDIR = 0; //count down after sync

	        // Master module
	        EPwm3Regs.TBCTL.bit.PRDLD = 0; //load on CTR = 0
	        EPwm3Regs.TBCTL.bit.SYNCOSEL = 1; //CTR = 0
	        EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; //using 100MHz clock

	        // Sync down-stream module
	        EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0; //shadow updates
	        EPwm3Regs.CMPCTL.bit.SHDWBMODE = 0; //shadow updates
	        EPwm3Regs.CMPCTL.bit.LOADAMODE = 2; //load on CTR = PRD or CTR = 0
	        // load on CTR=Zero
	        EPwm3Regs.CMPCTL.bit.LOADBMODE = 2; //load on CTR = PRD or CTR = 0
	        // load on CTR=Zero
	        EPwm3Regs.AQCTLA.bit.CAU = 1;  //clear on compare
	        // set actions for EPWM1A
	        EPwm3Regs.AQCTLA.bit.PRD = 2; //set on ctr = PRD

	        // load on CTR=Zero
	        EPwm3Regs.AQCTLB.bit.CAU = 2;  //clear on compare
	        // set actions for EPWM1A
	        EPwm3Regs.AQCTLB.bit.PRD = 1; //set on ctr = PRD

#if DEADBANDS_ACTIVE
    // enable Dead-band module
    EPwm3Regs.DBCTL.bit.POLSEL = 2; //Active high (AH) mode. Neither EPWMxA nor EPWMxB is inverted (default)
    EPwm3Regs.DBCTL.bit.IN_MODE = 0; //EPWMxA In (from the action-qualifier) is the source for both falling-edge and rising-edge delay.
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm3Regs.DBCTL.bit.SHDWDBREDMODE = 1;
    EPwm3Regs.DBCTL.bit.SHDWDBFEDMODE = 1;
    EPwm3Regs.DBCTL.bit.LOADREDMODE = 0;    // Load on Counter == 0
    EPwm3Regs.DBCTL.bit.LOADFEDMODE = 0;    // Load on Counter == 0
    EPwm3Regs.DBCTL.bit.HALFCYCLE = 1;
    EPwm3Regs.DBRED.bit.DBRED = 34;
    EPwm3Regs.DBREDHR.bit.DBREDHR = 0x0;
    EPwm3Regs.DBFED.bit.DBFED = 34;
    EPwm3Regs.DBFEDHR.bit.DBFEDHR = 0x0;
#endif
	// Run Time (Note: Example execution of one run-time instant)
	//===========================================================
	EPwm1Regs.CMPA.bit.CMPA = 100;
	EPwm1Regs.CMPB.bit.CMPB = 100;
	// adjust duty for output EPWM1A
	EPwm2Regs.CMPA.bit.CMPA = 100;
	EPwm2Regs.CMPB.bit.CMPB = 100;
	// adjust duty for output EPWM2A
	EPwm3Regs.CMPA.bit.CMPA = 100;
	EPwm3Regs.CMPB.bit.CMPB = 100;
	// adjust duty for output EPWM3A

	EPwm1Regs.TBCTL.bit.CTRMODE = 0; //Count up timer
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; //Count up timer
    EPwm3Regs.TBCTL.bit.CTRMODE = 0; //Count up timer
//---------------------------------------------------------------------
//--- Enable the clocks to the ePWM module.                   
//--- Note: this should be done after all ePWM modules are configured
//--- to ensure synchronization between the ePWM modules.
//---------------------------------------------------------------------
	asm(" EALLOW");							// Enable EALLOW protected register access
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;	// TBCLK to ePWM modules enabled
	asm(" EDIS");							// Disable EALLOW protected register access

} // end InitEPwm()


__interrupt void epwm1_isr(void)
{
    static volatile Uint16 GPIO34_count = 0;            // Counter for pin toggle

       PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;         // Must acknowledge the PIE group

       EPwm1Regs.ETCLR.bit.INT = 1; //clear the EPWM interrupt flag

#if LTC1864
       //Start a SPI data transmit
       //Pull the pin high; starts the conversion on the chip
       GpioDataRegs.GPASET.bit.GPIO19 = 1; //GPIO Set
       DelayUs(3);  //wait for 2.75 us for conversion to take place
       GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;  //GPIO Clear
       //SPI communication begins
       SpiaRegs.SPITXBUF=0xAAAA;      // Send data. This is a dummy byte
       //The above step automatically starts an SPI data transmit and a parallel receive
       // WAIT FOR THE BYTE TO BE SENT AND THE OTHER BYTE TO BE RECEIVED AND PLACED IN THE RXBUF REGISTER
       while(SpiaRegs.SPISTS.bit.INT_FLAG == 0);
       if(SpiaRegs.SPISTS.bit.INT_FLAG == 1);
       {
           if(GPIO34_count++ > 25000)                  // Toggle slowly to see the LED blink
                  {
                      GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Toggle the pin
                      GPIO34_count = 0;                       // Reset the counter
                  }
           rxdata1 = SpiaRegs.SPIRXBUF;//extarct the data (automatically clears the interrupt flag)
       }
#endif

#if LTC2338
       //Start a SPI data transmit
              //Pull the pin high; starts the conversion on the chip
              GpioDataRegs.GPASET.bit.GPIO19 = 1; //GPIO Set
              DelayUs(3);  //wait for 2.75 us for conversion to take place
              GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;  //GPIO Clear
              //SPI communication begins
              SpiaRegs.SPITXBUF=0x0000;      // Send data. First dummy byte in FIFO
              SpiaRegs.SPITXBUF=0x0000;      // Send data. Second dummy byte in FIFO
              //SpiaRegs.SPITXBUF=0x00;      // Send data. Third dummy byte in FIFO

              //The above step automatically starts an SPI data transmit and a subsequent receive
              // WAIT FOR THE BYTE TO BE SENT AND THE OTHER BYTE TO BE RECEIVED AND PLACED IN THE RXBUF REGISTER
              //while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);


              //Bit 13: Receive FIFO Reset; Re-enable receive FIFO operation.

              //SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;  //Clear the interrupt flag

              //Read the RXFIFO
              //for (i=0;i<WORDS;i++)
              //{
              //    rxfifobuf[i] = SpiaRegs.SPIRXBUF;
              //}

              //SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1; //Re-enable FIFO Receives

              //Reconstruct the ADC word by concatenating the ADC words
              //temp = rxfifobuf[0];
              //ADC_18bitval |= (temp<<9);
              //temp = rxfifobuf[1];
              //ADC_18bitval |= (temp<<0);

              if(GPIO34_count++ > 25000)                  // Toggle slowly to see the LED blink
              {
                  GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Toggle the pin
                  GPIO34_count = 0;                       // Reset the counter
              }
                  //rxdata1 = SpiaRegs.SPIRXBUF;//extarct the data (automatically clears the interrupt flag)

#endif


}

//--- end of file -----------------------------------------------------
