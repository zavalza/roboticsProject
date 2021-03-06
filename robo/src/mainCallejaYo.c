/*********************************************************************
 *
 *                  
 *
 *********************************************************************
 * FileName:        main.c
 * Dependencies:
 * Processor:       PIC32
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE
 *
 **********************************************************************/

/***********************************************************************************
									 README
 ***********************************************************************************
 * Objective: 
 *
 * Tools:
 *			1. MPLAB with PIC32MX support
 *
 *
 ***********************************************************************************
 ***********************************************************************************/

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
#define	GetSystemClock() 	(80000000ul)
#define SYS_FREQ 			(80000000L)
#define PB_DIV         		8
#define PB2_DIV				16
#define PRESCALE       		1
#define TOGGLES_PER_SEC		10000
#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
#define T2_TICK				(SYS_FREQ/PB2_DIV/PRESCALE/TOGGLES_PER_SEC)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())
#define DESIRED_BAUDRATE    	(9600)      //The desired UART BaudRate

/*****************************************************************************/


/********PROCEDURE declarations*********/
void WriteString(const char *string);
void PutCharacter(const char character);
void step (unsigned char data, unsigned char motorNumber);
/***************************************/


/*************GLOBALS**********************/
unsigned char COMMAND;
unsigned char MotorsON = 1;
unsigned char M1forward = 1, M2forward = 1;
unsigned int M1_counter = 0, M2_counter = 0;
unsigned int counterDistanceMeasure = 0, counterTrigger=0;
unsigned short frontDistance=0, backDistance=0, leftDistance=0, rightDistance=0; //in cms

#define SERVOMAXPERIOD 200
unsigned int servo_counter = 0, servo_period = 200;
int servo_angle = 0;

int auxcounter = 10000;
/******************************************/

int main(void)
{
//LOCALS
	unsigned int temp, slow = 1;
	unsigned int channel1, channel2;
	unsigned int M1_stepPeriod, M2_stepPeriod;
	M1_stepPeriod = M2_stepPeriod = 1000; // in tens of u-seconds
	unsigned char M1_state = 0, M2_state = 0;
	unsigned int tMedido = 0;
	unsigned char timesReaded=0;
	unsigned short delayFront = 0, delayBack = 0, delayLeft = 0, delayRight=0;
	unsigned short risingFront = 0, risingBack = 0, risingLeft = 0, risingRight=0;

	SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

/* TIMER1 - now configured to interrupt at 10 khz (every 100us) */
	OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, T1_TICK);
	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
/* TIMER2 - 20 khz interrupt for distance measure*/
	OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, T2_TICK);
	ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_3); 

/* PORTA.0 for servo-PWM */
	mPORTAClearBits(BIT_2);
	mPORTASetPinsDigitalOut(BIT_2);

/* some bits of PORTB for ultrasonic sensors */
	PORTResetPins(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11 );	
	PORTSetPinsDigitalOut(IOPORT_B, BIT_8 | BIT_9| BIT_10 | BIT_11); //trigger

/* external interrupts for echo signals
	ConfigINT1(EXT_INT_PRI_1 | RISING_EDGE_INT | EXT_INT_ENABLE); //RE8, highest priority
	ConfigINT2(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_ENABLE); //RE9
	ConfigINT3(EXT_INT_PRI_3 | RISING_EDGE_INT | EXT_INT_ENABLE); //RA14
	ConfigINT4(EXT_INT_PRI_4 | RISING_EDGE_INT | EXT_INT_ENABLE); //RA15
*/

//ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_0);

/* Use input capture module for echo time measure */
	//interrupt on every risging/falling edge starting with a rising edge
	PORTSetPinsDigitalIn(IOPORT_D, BIT_12);
	mIC1ClearIntFlag();
	OpenCapture5(  IC_SP_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_FEDGE_RISE | IC_ON );//front
	OpenCapture2( IC_SP_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_FEDGE_RISE | IC_ON );//back
	OpenCapture3( IC_SP_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_FEDGE_RISE | IC_ON );//right
	OpenCapture4( IC_SP_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER2_SRC | IC_FEDGE_RISE | IC_ON );//left

/* PINS used for the buttons */
    PORTSetPinsDigitalIn(IOPORT_D, BIT_13);
	#define CONFIG          (CN_ON | CN_IDLE_CON)
	#define INTERRUPT       (CHANGE_INT_ON | CHANGE_INT_PRI_2)
	mCNOpen(CONFIG, CN19_ENABLE, CN19_PULLUP_ENABLE);
	temp = mPORTDRead();

/* Analog input */
	CloseADC10();
	//#define PARAM1 ADC_MODULE_ON | ADC_FORMAT_INTG32 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
	//#define PARAM2 ADC_VREF_AVDD_AVSS | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_BUF_16 | ADC_ALT_INPUT_OFF
	//#define PARAM3 ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_5
	//#define PARAM4	ENABLE_AN0_ANA | ENABLE_AN1_ANA
	//#define PARAM5	SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
	//SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN0);
	//OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );
	//EnableADC10();

/* PORT D for motors */
	mPORTDSetBits(BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11); 		// Turn on PORTD on startup.
	mPORTDSetPinsDigitalOut(BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11);	// Make PORTD output.
/* PORTD for LEDs - DEBUGGING */
	mPORTDClearBits(BIT_0 | BIT_1 | BIT_2);
	mPORTDSetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2);


// Explorer-16 uses UART2 to connect to the PC.
	// This initialization assumes 36MHz Fpb clock. If it changes,
	// you will have to modify baud rate initializer.
    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
	// Configure UART2 RX Interrupt
	INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);


	
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
			counterTrigger=2; //Sends trigger signal during 1 interrupt of timer2 (50us)
			mPORTDSetBits(BIT_0); 	//DEBUGGING
			
			while(mIC5CaptureReady()) //Front echo answers
			{
			if(timesReaded==0)
			{
				risingFront=mIC5ReadCapture();
			}
			else if(timesReaded == 1)
			{
			 delayFront = mIC5ReadCapture()- risingFront;
			}
			timesReaded+=1;
			if(timesReaded==2)
			{
				//while(mIC5CaptureReady()){
				//	mIC5ReadCapture();
				//}	//vaciar FILA
				timesReaded=0;
			}
			mPORTDSetBits(BIT_1); 	//DEBUGGING
			}

			
			if(delayFront != 0){
				frontDistance= delayFront * 50 / 58; //counterTimer2*periodTimer2 (50us)/58 = cm;
				delayFront=0;
			}
			while( mIC2CaptureReady() ) //back echo answers
			{
			delayBack = mIC2ReadCapture() - delayBack;	
			}
			if(delayBack != 0)
				backDistance= delayBack * 50 / 58;

			while( mIC3CaptureReady() ) //Left echo answers
			{
			delayLeft = mIC3ReadCapture() - delayLeft;
			}
			if(delayLeft != 0)
				leftDistance= delayLeft * 50 / 58;

			while( mIC4CaptureReady() ) //Right echo answers
			{
			delayRight = mIC4ReadCapture() - delayRight;
			}
			if(delayRight != 0)
				rightDistance= delayRight * 50 / 58;
			counterDistanceMeasure=600; //measure distance again after 60ms
		}



		//Process UART command
		switch (COMMAND) {
			case 'q':
				if (MotorsON)
					MotorsON = 0;
				else
					MotorsON = 1;
				break;
			case 'w':
				M1forward = M2forward = 1;
				break;
			case 's':
				M1forward = M2forward = 0;
				break;
			case 'a': //left
				M1forward = 0;
				M2forward = 1;
				break;
			case 'd': //right
				M1forward = 1;
				M2forward = 0;
				break;
			case 'f':
				slow = 0;
				break;
			case 'g':
				slow = 1;
				break;
			
			case 0:
			default:
				break;
		}
		COMMAND = 0;
		
		if ( slow ) {
			M1_stepPeriod = 50; // value between 20 and 100 in tens of u-seconds (step period)
			M2_stepPeriod = 50;
		} else { //fast
			M1_stepPeriod = 20; // value between 20 and 100 in tens of u-seconds
			M2_stepPeriod = 20;
		}
			
		if (MotorsON) {
			/****************************
			MOTOR MAP
				M1 O-------------O M2   ON EVEN MOTORS, STEPS MUST BE INVERTED
					|	 /\		|			i.e. FORWARD IS BACKWARD
					|	/  \	|
					|	 || 	|
					|	 ||		|
				M2 O-------------O M1
			*****************************/
			if (M1_counter == 0) {
				switch (M1_state) {
					case 0: // set 0011
//		mPORTDSetBits(BIT_0);					//for debugging
						step (0x3 , 1);
						if (M1forward)
							M1_state = 1;
						else
							M1_state = 3;
						break;
					case 1: // set 0110
						step (0x6 , 1);
						if (M1forward)
							M1_state = 2;
						else
							M1_state = 0;
						break;
					case 2: // set 1100
//		mPORTDClearBits(BIT_0);					//for debugging
						step (0xC , 1);
						if (M1forward)
							M1_state = 3;
						else
							M1_state = 1;
						break;
					case 3: // set 1001
					default:
						step (0x9 , 1);
						if (M1forward)
							M1_state = 0;
						else
							M1_state = 2;
						break;	
				}
				M1_counter = M1_stepPeriod;
			}
			
			if (M2_counter == 0) {
				switch (M2_state) {
					case 0: // set 0011
						step (0x3 , 2);
						if (M2forward)
							M2_state = 1;
						else
							M2_state = 3;
						break;
					case 1: // set 1001
						step (0x9 , 2);
						if (M2forward)
							M2_state = 2;
						else
							M2_state = 0;
						break;
					case 2: // set 1100
						step (0xC , 2);
						if (M2forward)
							M2_state = 3;
						else
							M2_state = 1;
						break;
					case 3: // set 0110
					default:
						step (0x6 , 2);
						if (M2forward)
							M2_state = 0;
						else
							M2_state = 2;
						break;	
				}
				M2_counter = M2_stepPeriod;
			}

		} else {
			mPORTDSetBits(BIT_4 | BIT_5 | BIT_6 | BIT_7 |
					BIT_8 | BIT_9 | BIT_10 | BIT_11);
		}
		
		/******* SERVO CONTROL ********/
		if (auxcounter == 0) {
			if (servo_angle == 90)
				servo_angle = -90;
			else
				servo_angle = 90;
			auxcounter = 10000;		// toggle angle every 1 s.
		}

		servo_counter = (servo_angle + 90)*(18)/180 + 6; // between 600 and 2400 us

		if (servo_period == 0) {
			mPORTASetBits(BIT_2);
			servo_period = SERVOMAXPERIOD; 		/* 200 * 100us = 20000us period  */
		}
		/*******************************/
	
	} /* end of while(1)  */
		
	return 0;
}

/************** INTERRUPT HANDLERS *******************/

/* TIMER 1 Interrupt Handler - configured to 100us periods */
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

	if (M1_counter != 0)
		M1_counter--;
	if (M2_counter != 0)
		M2_counter--;
	
	if (servo_period != 0) {
		servo_period--;
		if (servo_period == (SERVOMAXPERIOD - servo_counter))
			mPORTAClearBits(BIT_2);
	}

	if (auxcounter != 0)
		auxcounter--;

	if(counterDistanceMeasure !=0)
		counterDistanceMeasure--;
}

/* TIMER 2 Interrupt Handler - configured to 50us periods */
void __ISR(_TIMER_2_VECTOR, ipl3) Timer2Handler(void)
{
    // clear the interrupt flag
    mT2ClearIntFlag();
	mPORTDSetBits(BIT_2); 	//DEBUGGING
	counterTrigger--;
	if(counterTrigger==1){
	mPORTBSetBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Sends trigger signal to the four sensors
	}
	if(counterTrigger == 0){
		mPORTBClearBits(BIT_8 | BIT_9| BIT_10 | BIT_11);	//Shut down trigger signal
		//WriteTimer2(0x0000);//Reset Timer
	}

}

/*void __ISR( _INPUT_CAPTURE_1_VECTOR, ipl3) Capture1(void)
{ 
static unsigned int con_buf[32]; // Declare an input capture buffer 
// Declare three time capture variables: 
static unsigned short int time1; // Most recent captured time 
static unsigned short int time2 = 0; // Previous time captured 
static unsigned short int time_diff = 0; // Time between captures 
 
ReadCapture1(con_buf); // Read captures into buffer 
time1 = con_buf[0]; // Save time of event 
time_diff = time1-time2; // Subtracts previous capture time to 
 // determine period. 
time2 = time1; // Save time1 as time 2 
// Compute motor speed in RPS (revolutions per second). Equation to be 
// worked out by reader 
mIC5ClearIntFlag(); // Clears interrupt flag 
// INTClearFLag(INT_IC5); // Alternate peripheral library function 
 
} */


/*void __ISR(_EXTERNAL_1_VECTOR, ipl7) INT1Interrupt() //Front sensor
{ 
   mINT1IntEnable(0);
   mINT1ClearIntFlag();
   frontDistance= delayFront*50/58; //us/58=cm
mPORTDSetBits(BIT_1); 	//DEBUGGING
}
void __ISR(_EXTERNAL_2_VECTOR, ipl7) INT2Interrupt() 
{ 
   mINT2IntEnable(0);
   mINT2ClearIntFlag();
   backDistance=delayBack*50/58; //us/58=cm
} 
void __ISR(_EXTERNAL_3_VECTOR, ipl7) INT3Interrupt() 
{ 
   mINT3IntEnable(0);
   mINT3ClearIntFlag();
   leftDistance = delayLeft * 50/58; //us/58=cm
} 
void __ISR(_EXTERNAL_4_VECTOR, ipl7) INT4Interrupt() 
{ 
   mINT4IntEnable(0);
   mINT4ClearIntFlag();
   rightDistance = delayRight * 50/58; //us/58=cm
}*/

// UART 2 interrupt handler
// it is set at priority level 2
void __ISR(_UART2_VECTOR, ipl2) IntUart2Handler(void)
{
	// Is this an RX interrupt?
	if(INTGetFlag(INT_SOURCE_UART_RX(UART2)))
	{
		unsigned char databyte;

		// Clear the RX interrupt Flag
	    INTClearFlag(INT_SOURCE_UART_RX(UART2));

		// Code to be executed on RX interrupt:
		databyte = UARTGetDataByte(UART2);
		COMMAND = databyte;
		
		// Echo what we just received.
		PutCharacter(databyte);
	}

	// We don't care about TX interrupt
	if ( INTGetFlag(INT_SOURCE_UART_TX(UART2)) )
	{
		// Clear the TX interrupt Flag
		INTClearFlag(INT_SOURCE_UART_TX(UART2));

		// Code to be executed on TX interrupt:

		
	}
}

// Configure the CN interrupt handler

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void)
{
	unsigned int temp;

    // clear the mismatch condition
    temp = mPORTDRead();

    // clear the interrupt flag
    mCNClearIntFlag();

    // .. things to do .. 
    if ( !(temp & (1<<13)) ) { //button on RD13 is pressed
		if (MotorsON == 0)
			MotorsON = 1;
		else
			MotorsON = 0;
	}
	
}
/********************************************/


/************ PROCEDURES ********************/
void step (unsigned char data, unsigned char motorNumber) {
	data = data & 0x0F; // make sure only the LS-nibble will be overwritten
	unsigned int lectura = mPORTDRead() & (~(0xF << motorNumber*4));
	
	mPORTDWrite(lectura | (data << motorNumber*4));
}

void WriteString(const char *string)
{
    while(*string != '\0')
    {
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, *string);

        string++;

        while(!UARTTransmissionHasCompleted(UART2))
            ;
    }
}

void PutCharacter(const char character)
{
        while(!UARTTransmitterIsReady(UART2))
            ;

        UARTSendDataByte(UART2, character);


        while(!UARTTransmissionHasCompleted(UART2))
            ;
}

/**************************************************/
