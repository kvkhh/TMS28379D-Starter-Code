#include "Lab.h"                // Main include file
#include <stdbool.h> //boolean library

#define Forward_Mode_Gate_Drivers 1
#define Reverse_Mode_Gate_Drivers 2

#define DRIVERS 12 //Change this value as the demands and necssities dictate in the future;
#define Node1_Gate_Drivers 3
#define Node2_Gate_Drivers 4
#define Node3_Gate_Drivers 5

#define Node1_Driver1 1
#define Node1_Driver2 2
#define Node1_Driver3 3
#define Node1_Driver4 4

#define Node2_Driver1 5
#define Node2_Driver2 6
#define Node2_Driver3 7
#define Node2_Driver4 8

#define Node3_Driver1 9
#define Node3_Driver2 10
#define Node3_Driver3 11
#define Node3_Driver4 12

#define All_Node_Drivers 13

#define DRIVER_GOOD 1
#define DRIVER_FAULT 0

void GPIO_SetupXINT1Gpio(Uint16 gpioNumber);
void InitGateDrvGPIO(void);
void ActivateDrivers(int DriverMode);
void DeactivateDrivers(int DriverMode);
void ResetDriver(int DriverNumber);
void FaultScan(void);

interrupt void xint1_GateDriverFault_isr(void);

int Gate_Driver_Fault_Status[DRIVERS];

//This is a shared resource
struct conpar
{
    //general parameters
    //uint16_t duty; //duty value
    //uint16_t vsensed; //adc sampled voltage
    //uint16_t temperature; //adc sampled MOSFET temperature
    //uint16_t timer_count; //converter switching frequency

    //status register bits
    //bool status; //active or inactive

    //bool alert; //INA300 over current alert signal
    //bool overtemperature; //MOSFET temperature bits
    //bool controller; //controller status

    //uint8_t status_reg; //status register

    //INA300 control bits (not a part of status register)
    //bool enable; //INA300 current sense IC Enable after latch
    //bool latch; //IAN300 current sense IC latch

    //Gate Driver Controls
    bool DrvOneRdy; //Driver One Ready bit
    bool DrvOneFlt; //Driver One Fault Bit
    bool DrvOneCmdCtrl; //Driver One Switch ON Request by user software

    bool DrvTwoRdy; //Driver Two Ready bit
    bool DrvTwoFlt; //Driver Two Fault Bit
    bool DrvTwoCmdCtrl; //Driver Two Switch ON Request by user software

    bool DrvThreeRdy; //Driver Three Ready bit
    bool DrvThreeFlt; //Driver Three Fault Bit
    bool DrvThreeCmdCtrl; //Driver Three Switch ON Request by user software

    bool DrvFourRdy; //Driver Four Ready bit
    bool DrvFourFlt; //Driver Four Fault Bit
    bool DrvFourCmdCtrl; //Driver Four Switch ON Request by user software


    //controller parameters
    //uint16_t setpoint; //set point of the target voltage
    //uint16_t error;
    //uint16_t controller_effort;

};

struct conpar Node1;

void InitGateDrvGPIO(void)
{
    //Setting for the FLT Pin
        //This is a pin that can be pulled high or low; doesn't need pull up/down resistors
        //specify the direction of the pin as output
        //Currently assigning GPIO 49 as the FLT Pin
        //FLT Pins from all the gate drivers are AND-ed to get one single line to trigger the EXT ISR

    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0; //Configured as input

    //Settings for the EXT ISR FLT Pin
    //Activate PIE Group #1 Channel #4; XINT1
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    IER |= 0x0001; //Enable Channel 1

    GPIO_SetupXINT1Gpio(49); //Register GPIO pin 49 as an XINT1 source in the XBAR

    EALLOW;   // This is needed to write to EALLOW protected registers
    PieVectTable.XINT1_INT = &xint1_GateDriverFault_isr; //register the ISR
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //Settings for the EN/RST Pin
        //EN/RST Pin in UCC21710-Q1 is an NOT an Open Drain Pin
        //Therefore need not use internal pullup resistors on the input sense pin
        GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1; //Configured as output
        GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1; //Configured as output


        GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1; //Pull up resistor disabled
        GpioCtrlRegs.GPBPUD.bit.GPIO42 = 1; //Pull up resistor disabled


    //Enable all the specific fault sense input pins as inputs
        GpioCtrlRegs.GPBDIR.bit.GPIO37 = 0; //Configured as input; Assigned to Node1 Q1 FLT
        GpioCtrlRegs.GPBDIR.bit.GPIO60 = 0; //Configured as input; Assigned to Node1 Q2 FLT
        GpioCtrlRegs.GPBDIR.bit.GPIO63 = 0; //Configured as input; Assigned to Node1 Q3 FLT
        GpioCtrlRegs.GPCDIR.bit.GPIO65 = 0; //Configured as input; Assigned to Node1 Q4 FLT

    //Enable all specific fault sense pin's pull up's
        GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0; //Pull up resistor enabled
        GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0; //Pull up resistor enabled
        GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; //Pull up resistor enabled
        GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0; //Pull up resistor enabled
}


/*
 * void xint1_GateDriverFault_isr(void)
 * Desc: Fires whenever an Overcurrent event is triggered by any of the gate drivers
 * Modifies: PWM registers; Converter status bits
 *
 */
void xint1_GateDriverFault_isr(void)
{


    //Shut down all the PWM channels
    EPwm1Regs.TBCTL.bit.CTRMODE = 3; //stop timer
    EPwm2Regs.TBCTL.bit.CTRMODE = 3; //stop timer
    EPwm3Regs.TBCTL.bit.CTRMODE = 3; //stop timer

    //Scan all the gate fault pins on the  assigned GPIO and assert the bit of the gate driver
    if(GpioDataRegs.GPBDAT.bit.GPIO37 == 0) //if flt pin on GPIO48 is low
    {
        asm(" NOP");
        //Add Code to register this gate driver's output
    }
    if(GpioDataRegs.GPBDAT.bit.GPIO60 == 0) //if flt pin on GPIO48 is low
    {
        asm(" NOP");
        //Add Code to register this gate driver's output
    }
    if(GpioDataRegs.GPBDAT.bit.GPIO63 == 0) //if flt pin on GPIO48 is low
    {
        asm(" NOP");
        //Add Code to register this gate driver's output
    }
    if(GpioDataRegs.GPCDAT.bit.GPIO65 == 0) //if flt pin on GPIO48 is low
    {
        asm(" NOP");
        //Add Code to register this gate driver's output
    }

    //
    // Acknowledge this interrupt to get more from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//Registers the GPIO pin to XINT1
void GPIO_SetupXINT1Gpio(Uint16 gpioNumber)
    {
        EALLOW;
        InputXbarRegs.INPUT4SELECT = gpioNumber;      //Set XINT1 source to GPIO-pin
        EDIS;
    }

/*
 * void ActivateDrivers(int DriverNumber)
 * Desc: Activates the selected driver and modifies the status bits for the said driver in the structure
 * Input: Identifier for which side gate drivers are to be activated
 */

void ActivateDrivers(int DriverMode)
{
    //Pull the RST/EN Pin high, to activate the driver
    if(DriverMode == Forward_Mode_Gate_Drivers)
    {
        GpioDataRegs.GPBSET.bit.GPIO41 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1;
    }
    else if (DriverMode == Reverse_Mode_Gate_Drivers)
    {
        GpioDataRegs.GPBSET.bit.GPIO42 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;
    }
}


/*
 * void DeactivateDrivers(int DriverNumber)
 * Desc: Activates the selected driver and modifies the status bits for the said driver in the structure
 * Input: Identifier for which side gate drivers are to be activated
 */

void DeactivateDrivers(int DriverMode)
{
    //Pull the RST/EN Pin high, to activate the driver
    if(DriverMode == Forward_Mode_Gate_Drivers)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;
    }
    else if(DriverMode == Reverse_Mode_Gate_Drivers)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1;
    }
    else if(DriverMode == Node1_Gate_Drivers)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1;
    }

}

/*
 * void ResetDriver(int DriverNumber)
 * Desc: Reactivates the fault triggered gate driver IC for normal operation (unmutes the gate driver output and brings the fault line high)
 * Input: Driver Number
 * Warning: Make sure that the fault is repaired before firing this function. Otherwise they'll again trip!
 */
void ResetDriver(int DriverNumber)
{
    //Node 1 Primary Drivers
    if((DriverNumber == Node1_Driver1) || (DriverNumber == Node1_Driver2))
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1; //pull the RST/EN pin low
        DelayUs(1); //hold the pin low for atleast 650 ns; so, waiting for 1us
        GpioDataRegs.GPBSET.bit.GPIO41 = 1; //pull the RST/EN pin hi
        //the fault pin should go high by now; need to add logic to confirm that it FLT pin has actually gone high
        FaultScan();
        if(Gate_Driver_Fault_Status[0] == DRIVER_FAULT || Gate_Driver_Fault_Status[1] == DRIVER_FAULT)
        {
            asm(" NOP");
            //Later on we have to add code here to let the user know that a fault has occured over here
            //Because any user interface is not active as of now this subroutine is left with a no-op
        }

    }
    //Node 1 Secondary Drivers
    if((DriverNumber == Node1_Driver3) || (DriverNumber == Node1_Driver4))
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1; //pull the RST/EN pin low
        DelayUs(1); //hold the pin low for atleast 650 ns; so, waiting for 1us
        GpioDataRegs.GPBSET.bit.GPIO42 = 1; //pull the RST/EN pin hi
        //the fault pin should go high by now; need to add logic to confirm that it FLT pin has actually gone high
        FaultScan();
        if(Gate_Driver_Fault_Status[2] == DRIVER_FAULT || Gate_Driver_Fault_Status[3] == DRIVER_FAULT)
        {
            asm(" NOP");
            //Later on we have to add code here to let the user know that a fault has occured over here
            //Because any user interface is not active as of now this subroutine is left with a no-op
        }

    }
}

/*
 * int FaultScan(int Node)
 * Desc: Scans each of the GPIO pins corresponding to the FLT lines of the GAte Driver IC's and loads an array with 1's and 0's; 1 defines as DRIVER_GOOD
 * within each cell and indicates that there is no fault in the IC that corresponds to that particular cell array number; 0 defined as DRIVER_FAULT indicates
 * that the fault has occurred or is still in effect
 * Input: None
 * Output: None
 */
void FaultScan(void)
{

    int i;

    for(i=0;i<DRIVERS; i++)
    {
        Gate_Driver_Fault_Status[i] = DRIVER_GOOD;
    }

        if(GpioDataRegs.GPBDAT.bit.GPIO37 == 0)
        {
            Gate_Driver_Fault_Status[0] = DRIVER_FAULT;
        }
        if(GpioDataRegs.GPBDAT.bit.GPIO60 == 0)
        {
            Gate_Driver_Fault_Status[1] = DRIVER_FAULT;
        }
        if(GpioDataRegs.GPBDAT.bit.GPIO63 == 0)
        {
            Gate_Driver_Fault_Status[2] = DRIVER_FAULT;
        }
        if(GpioDataRegs.GPCDAT.bit.GPIO65 == 0)
        {
            Gate_Driver_Fault_Status[3] = DRIVER_FAULT;
        }

        //Extend when necessary...
}


