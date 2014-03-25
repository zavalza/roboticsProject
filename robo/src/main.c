#include <plib.h>					/* Peripheral Library */

// *****************************************************************************
// *****************************************************************************
// Section: Configuration bits
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 10 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
// *****************************************************************************
// *****************************************************************************
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_8


/******************************************************************************/
// Section: System Macros
/******************************************************************************/
#define CONFIG          (CN_ON | CN_IDLE_CON)
#define INTERRUPT       (CHANGE_INT_ON | CHANGE_INT_PRI_2)
#define	GetSystemClock() 	(80000000ul)
#define SYS_FREQ 			(80000000L)
#define PB_DIV         		8
#define PRESCALE       		1
#define TOGGLES_PER_SEC		10000
#define TOGGLES_PER_SEC2	100000
#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define T2_TICK				(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC2)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())
#define DESIRED_BAUDRATE    	(9600)      //The desired UART BaudRate

/*************GLOBALS**********************/
unsigned int counterDistanceMeasure = 0, counterTrigger=0;
unsigned short frontDistance=0, backDistance=0, leftDistance=0, rightDistance=0; //in cms
unsigned short timeFront = 0, timeBack = 0, timeLeft = 0, timeRight=0;
unsigned short delayFront = 0, delayBack = 0, delayLeft = 0, delayRight=0;
unsigned volatile char timesRead = 0;
unsigned volatile int portValue = 0;
int main(void)
{

	SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

/* TIMER1 - now configured to interrupt at 10 khz (every 100us) */
	OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, T1_TICK);
	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
/* TIMER2 - 100 khz interrupt for distance measure*/
	OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, T2_TICK);
	ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_3); 


/* some bits of PORTB for ultrasonic sensors */
	PORTResetPins(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11 );	
	PORTSetPinsDigitalOut(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11); //trigger

/* Use input capture module for echo time measure */
	//interrupt on every risging/falling edge starting with a rising edge
	PORTSetPinsDigitalIn(IOPORT_D, BIT_12); //INC5 Pin
	mIC1ClearIntFlag();
	OpenCapture5(  IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_ON );//front
	ConfigIntCapture5(IC_INT_ON | IC_INT_PRIOR_4 | IC_INT_SUB_PRIOR_3);

/* PORTD for LEDs - DEBUGGING */
	mPORTDClearBits(BIT_0 | BIT_1 | BIT_2);
	mPORTDSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2);

	
// Congifure Change/Notice Interrupt Flag
	ConfigIntCN(INTERRUPT);
// configure for multi-vectored mode
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
// enable interrupts
    INTEnableInterrupts();

	counterDistanceMeasure=600; //measure distance each 60 ms

	// Let interrupt handler do the work
	while (1) {

		if(counterDistanceMeasure==0){ //Measure distance with ultrasonic sensors
			counterTrigger=6; //Sends trigger signal during 5 interrupts of timer2 (50us)
			counterDistanceMeasure=600; //measure distance again after 60ms
		}
	}
}
/************** INTERRUPT HANDLERS *******************/

/* TIMER 1 Interrupt Handler - configured to 100us periods */
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

	if(counterDistanceMeasure !=0)
		counterDistanceMeasure--;
}

/* TIMER 2 Interrupt Handler - configured to 10us periods */
void __ISR(_TIMER_2_VECTOR, ipl3) Timer2Handler(void)
{
    // clear the interrupt flag
    mT2ClearIntFlag();
	counterTrigger--;
	if(counterTrigger==5){
	mPORTBSetBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Sends trigger signal to the four sensors
	}
	if(counterTrigger == 0){
		mPORTBClearBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Shut down trigger signal
	}
	
	delayFront++;

}

void __ISR(_INPUT_CAPTURE_5_VECTOR, ipl4) IC5Handler(void)
{
    // clear the interrupt flag
    //mIC5ClearIntFlag();
	INTClearFlag(INT_IC5);
	mPORTDSetBits(BIT_2); 	//DEBUGGING
	portValue = PORTReadBits(IOPORT_D, BIT_12);
	if(portValue>0 && timesRead==0){
		mPORTDSetBits(BIT_0); 
		timeFront=delayFront;
		timesRead++;
	}
	
	if(portValue==0 && timesRead==1){
	mPORTDSetBits(BIT_1); 
	timeFront= delayFront - timeFront;
	timesRead=0;
	frontDistance= timeFront * 10.0 / 58.0; //counterTimer2*periodTimer2 (10us)/58 = cm;
	delayFront=0;
	mPORTDClearBits(BIT_2|BIT_1|BIT_0);
	}

}