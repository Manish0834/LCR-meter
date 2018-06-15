//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <strings.h>
#include <math.h>
#include <inttypes.h>
#include "tm4c123gh6pm.h"
#include "wait.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define MAX_CHARS    80
#define MEAS_LR      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))
#define MEAS_C       (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))
#define LOWSIDE_R    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define HIGHSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4)))
#define INTEGRATE    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define ON           1
#define OFF          0
#define true         1
#define false        0
char Str[MAX_CHARS+1],Str2[MAX_CHARS+1],Str3[MAX_CHARS+1];
char FieldType[20];
uint32_t FieldCount,i,j;
uint8_t Offset[20];
char Value1[20],Value2[20];
char Command1[20],Command2[20];
char Str4_1[81],Str4_2[81],Str5_1[81],Str5_2[81];
bool timeMode = false;
uint32_t frequency = 0;
float time = 0;
bool timeUpdate = false;
char StrTest[10];
float res=0;
char StrRes[10];


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
	while(PUSH_BUTTON);
}

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	// Set GPIO ports to use APB (not needed since default configuration -- for clarity)
	// Note UART on port A must use APB
	SYSCTL_GPIOHBCTL_R = 0;

	// Enable all GPIO port peripherals
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC;

	// Configure LED
	GPIO_PORTF_DIR_R = 0x1E;  // bits 1, 2, 3 and 4 are outputs, other pins are inputs
	GPIO_PORTF_DR2R_R = 0x1E; // set drive Strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons

	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;     //unlocking GPIO port F for comparator output
	GPIO_PORTF_CR_R = 0x01;

	GPIO_PORTA_DIR_R = 0x24;  // bits 2 and 5 are outputs, other pins are inputs
	GPIO_PORTA_DR2R_R = 0x24; // set drive Strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTA_DEN_R = 0x24;  // enable LEDs

	GPIO_PORTB_DIR_R = 0x08;  // bits 3 is output, other pins are inputs
	GPIO_PORTB_DR2R_R = 0x08; // set drive Strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTB_DEN_R = 0x08;  // enable LEDs

	GPIO_PORTD_DIR_R = 0x40;  // bits 6 is output, other pins are inputs
	GPIO_PORTD_DR2R_R = 0x40; // set drive Strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTD_DEN_R = 0x40;  // enable LEDs

	// Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
	GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
	UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

	// Configure AN0 as an analog input
	SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
	GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
	GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
	GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3
	ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
	ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
	ADC0_SSMUX3_R = 0;                               // set first sample to AN0
	ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

	SYSCTL_RCGCACMP_R =1;
	GPIO_PORTC_DIR_R |=0x00;
	GPIO_PORTC_AFSEL_R |= 0x00;
	GPIO_PORTC_DEN_R &=~ 0xC0;         				 // disabling digital enable on pin PC6 and PC7
	GPIO_PORTC_AMSEL_R |=0xC0;						 // enabling analog mode for pin PC6 and PC7

	GPIO_PORTC_AFSEL_R |= 0x10;                      // select alternative functions for FREQ_IN pin
	GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to FREQ_IN
	GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
	GPIO_PORTC_DEN_R |= 0x10;                        // enable bit 4 for digital input

	//configuring comparator ports
	GPIO_PORTF_DIR_R |= 0x01;
	GPIO_PORTF_AFSEL_R |=0x01;
	GPIO_PORTF_DEN_R |=0x01;
	GPIO_PORTF_DR2R_R |=0x01;
	GPIO_PORTF_PCTL_R |=GPIO_PCTL_PF0_C0O;

	COMP_ACREFCTL_R |=0x20D;
	COMP_ACCTL0_R |= 0x402;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a String when the UART buffer is not full
void putsUart0(char* Str)
{
	uint8_t i;
	for (i = 0; i < strlen(Str); i++)
		putcUart0(Str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

int16_t readAdc0Ss3()
{
	ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
	while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
	return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

void setTimerMode()
{
	SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;     // turn-on timer
	WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
	WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
	WTIMER0_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
	WTIMER0_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
	WTIMER0_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
	WTIMER0_TAV_R = 0;                               // zero counter for first period
	WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
	NVIC_EN2_R |= 1 << (INT_WTIMER0A-16-64);         // turn-on interrupt 120 (WTIMER5A)
}

void WideTimer0Isr()
{
	time = WTIMER0_TAV_R;                        // read counter input
	WTIMER0_TAV_R = 0;                           // zero counter for next edge
	time /= 40;                                  // scale to us units
	timeUpdate = true;                           // set update flag
	//RED_LED ^= 1;                              // status
	WTIMER0_ICR_R = TIMER_ICR_CAECINT;
}

void getString()
{
	uint8_t count=0;
	char c;
	while(1)
	{
		c = (uint8_t)getcUart0();
		if(c==8)
		{
			if(count>0)
			{
				count--;
				continue;
			}
			else
				continue;
		}

		else
		{
			if(c==13)
			{
				Str[count]='\0';
				break;
			}
			else
			{
				if(c<' ')
					continue;

				else
					Str[count++]=c;
			}
		}
		if (count==MAX_CHARS)
		{
			Str[count]='\0';
			break;
		}
		else
			continue;
	}
}

void parseString()
{
	uint8_t DelimeterCount=0,DelimeterToAlphaNumCount=0;
	uint8_t c, c_1,cs;
	for (i=0;i<81;i++)
		Str3[i]='\0';

	for (i=0;i<strlen(Str);i++)
	{
		Str2[i]=Str[i];
	}
	Str2[strlen(Str)]='\0';
	cs=(uint8_t)Str[0];
	if (!(cs<46||cs==47||(cs>57&&cs<65)||(cs>90&&cs<95)||cs>123||cs==96))
	{
		DelimeterToAlphaNumCount++;
		Offset[0]=0;
		FieldType[0]='a';
	}
	for (i = 0; i < strlen(Str); i++)
	{
		c=(uint8_t)Str[i];
		c_1=(uint8_t)Str[i+1];
		if(c<46||c==47||(c>57&&c<65)||(c>90&&c<95)||c>123||c==96)
		{
			Str2[i]='\0';
			DelimeterCount++;
			if (c_1<46||c_1==47||(c_1>57&&c_1<65)||(c_1>90&&c_1<95)||c_1>123||c_1==96)
				continue;
			else
			{
				Offset[DelimeterToAlphaNumCount]=i+1;
				if ((c_1>64&&c_1<91)||(c_1>97&&c_1<123)||(c_1==95))
					FieldType[DelimeterToAlphaNumCount]='a';
				else
					FieldType[DelimeterToAlphaNumCount]='n';
				DelimeterToAlphaNumCount++;
				continue;
			}
		}
		else
			continue;
	}
	FieldCount=DelimeterToAlphaNumCount;
	for (j=0;j<strlen(Str);j++)
	{
		Str3[j]=Str[Offset[0]+j];
		if (!((Str3[j]>64&&Str3[j]<91)||(Str3[j]>96&&Str3[j]<123)||(Str3[j]==46)||(Str3[j]>47&&Str3[j]<58)||Str3[j]==95))
			Str3[j]='\0';
	}
}

int isCommand(char *StrCommand,uint8_t minArgCount)
{
	if ((!(strcasecmp(StrCommand,Str3))&&(FieldCount>minArgCount)&&(minArgCount==(FieldCount-1))))
		return 1;
	else
		return 0;
}

void getValue()
{
	uint8_t num=0;
	while(num<FieldCount)
	{
		if(FieldType[num]=='n')
		{
			if(num==1)
			{
				for(j=0;j<strlen(Str);j++)
				{
					Str4_1[j]=Str[Offset[num]+j];
					if (!((Str4_1[j]>64&&Str4_1[j]<91)||(Str4_1[j]>96&&Str4_1[j]<123)||(Str4_1[j]==46)||(Str4_1[j]>47&&Str4_1[j]<58)||Str4_1[j]==95))
						Str4_1[j]='\0';
				}
			}
			if(num==2)
			{
				for(j=0;j<strlen(Str);j ++)
				{
					Str4_2[j]=Str[Offset[num]+j];
					if (!((Str4_2[j]>64&&Str4_2[j]<91)||(Str4_2[j]>96&&Str4_2[j]<123)||(Str4_2[j]==46)||(Str4_2[j]>47&&Str4_2[j]<58)||Str4_2[j]==95))
						Str4_2[j]='\0';
				}
			}
		}
		num++;
	}
}


void getCommand()
{
	uint8_t num=0;
	while(num<FieldCount)
	{
		if(FieldType[num]=='a')
		{
			if(num==1)
			{
				for(j=0;j<strlen(Str);j++)
				{
					Str5_1[j]=Str[Offset[num]+j];
					if (!((Str5_1[j]>64&&Str5_1[j]<91)||(Str5_1[j]>96&&Str5_1[j]<123)||(Str5_1[j]==46)||(Str5_1[j]>47&&Str5_1[j]<58)||Str5_1[j]==95))
						Str5_1[j]='\0';
				}
			}
			if(num==2)
			{
				for(j=0;j<strlen(Str);j++)
				{
					Str5_2[j]=Str[Offset[num]+j];
					if (!((Str5_2[j]>64&&Str5_2[j]<91)||(Str5_2[j]>96&&Str5_2[j]<123)||(Str5_2[j]==46)||(Str5_2[j]>47&&Str5_2[j]<58)||Str5_2[j]==95))
						Str5_2[j]='\0';
				}
			}
		}
		num++;
	}
}

void ioCommand()
{
	if((strcasecmp(Str5_1,"MEAS_LR")==0))
	{
		if((strcasecmp(Str5_2,"ON")==0))
		{
			MEAS_C=0;
			waitMicrosecond(500000);
			MEAS_LR=1;
		}
		else
			MEAS_LR=0;
	}
	else if(!(strcasecmp(Str5_1,"MEAS_C")))
	{
		if(!(strcasecmp(Str5_2,"ON")))
		{
			MEAS_LR=0;
			waitMicrosecond(500000);
			MEAS_C=1;
		}
		else
			MEAS_C=0;
	}
	else if(!(strcasecmp(Str5_1,"LOWSIDE_R")))
	{
		if(!(strcasecmp(Str5_2,"ON")))
			LOWSIDE_R=1;
		else
			LOWSIDE_R=0;
	}
	else if(!(strcasecmp(Str5_1,"HIGHSIDE_R")))
	{
		if(!(strcasecmp(Str5_2,"ON")))
			HIGHSIDE_R=1;
		else
			HIGHSIDE_R=0;
	}
	else if(!(strcasecmp(Str5_1,"INTEGRATE")))
	{
		if(!(strcasecmp(Str5_2,"ON")))
			INTEGRATE=1;
		else
			INTEGRATE=0;
	}
	else
	{
		putsUart0("Invalid GPIO pin");
		putsUart0("\n\r");
	}
}

void voltageCommand()
{
	uint16_t raw;
	float instantVoltage, iirVoltage;
	float alpha = 0.99;
	int firstUpdate = true;
	char Str_adc[20];
	while(1)
	{
		raw = readAdc0Ss3();
		instantVoltage = ((raw / 4096.0 * 3.3)-0.01);
		if (firstUpdate)
		{
			iirVoltage = instantVoltage;
			firstUpdate = false;
		}
		else
		{
			iirVoltage = iirVoltage * alpha + instantVoltage * (1-alpha);
		}
		// display raw ADC value and Voltage
		sprintf(Str_adc, "%u", raw);
		putsUart0("Voltage == ");
		putsUart0(Str_adc);
		putsUart0("\n\r");
		sprintf(Str_adc, "%f", instantVoltage);
		putsUart0("Instant Voltage == ");
		//setGraphicsLcdTextPosition(100, 1);
		putsUart0(Str_adc);
		putsUart0("\n\r");
		sprintf(Str_adc, "%f", iirVoltage);
		putsUart0("IIR Voltage == ");
		putsUart0(Str_adc);
		putsUart0("\n\r");
		waitMicrosecond(5000);
	}
}

void alliooff()
{
	MEAS_LR=0;
	MEAS_C=0;
	LOWSIDE_R=0;
	HIGHSIDE_R=0;
	INTEGRATE=0;
}

void test()
{
	MEAS_LR=0;
	MEAS_C=0;
	INTEGRATE=1;
	LOWSIDE_R=1;
	waitMicrosecond(1000);
	LOWSIDE_R=0;
	timeUpdate=false;
	WTIMER0_TAV_R = 0;
	HIGHSIDE_R=1;
	while (1)
	{
		if (timeUpdate)
		{
			timeUpdate =false ;
			sprintf(StrTest,"%.0f", time/1.2);
			putsUart0(StrTest);
			putsUart0("\n\r");
			break;
		}
		else continue;
	}
}

void reset()
{
	alliooff();
	__asm("    .global _c_int00\n"
			"    b.w     _c_int00");
}

void resistance()
{
	HIGHSIDE_R=0;
	MEAS_C=0;
	INTEGRATE=1;
	LOWSIDE_R=1;
	waitMicrosecond(10000);
	LOWSIDE_R=0;
	timeUpdate=false;
	WTIMER0_TAV_R = 0;
	MEAS_LR=1;
	while (1)
	{
		if (timeUpdate)
		{
			timeUpdate =false ;
			sprintf(StrTest,"%f", (time/1200)+0.0132);
			putsUart0("Resistance= ");
			putsUart0(StrTest);
			putsUart0("KOhms\n\r");
			break;
		}
	}
}

void capacitance()
{
	float cap;
	MEAS_LR=0;
	MEAS_C=1;
	INTEGRATE=0;
	LOWSIDE_R=1;
	waitMicrosecond(10000);
	LOWSIDE_R=0;
	timeUpdate=false;
	WTIMER0_TAV_R = 0;
	HIGHSIDE_R=1;
	waitMicrosecond(50000);
	while (1)
	{
		if (timeUpdate)
		{
			timeUpdate =false ;
			cap=time/120000.0;
			sprintf(StrTest,"%f", cap);
			putsUart0("Capacitance= ");
			putsUart0(StrTest);
			putcUart0(230);
			putsUart0(" farads\n\r");
			break;
		}
	}
}
void inductance()
{
	MEAS_C=0;

	HIGHSIDE_R=0;
	INTEGRATE=0;
	waitMicrosecond(30000);
	LOWSIDE_R=1;
	WTIMER0_TAV_R=0;
	MEAS_LR=1;
	waitMicrosecond(2000000);
	uint16_t raw = readAdc0Ss3();
	float instantVoltage = ((raw / 4096.0 * 3.3));
	float resr=(106.52/(instantVoltage))-33;
	sprintf(StrTest,"%f", instantVoltage);
	putsUart0(StrTest);
	putsUart0("\n\r");
	sprintf(StrRes,"%f", resr);
	putsUart0(StrRes);
	putsUart0("\n\r");
	while (1)
	{
		if (timeUpdate)
		{
			timeUpdate =false ;
			sprintf(StrTest,"%f", (33+resr)*time/2.5);
			putsUart0("Inductance= ");
			putsUart0(StrTest);
			putcUart0(230);
			putsUart0(" Henry\n\r");
			break;
		}
		else continue;
	}
}

void autoCommand()
{
	uint16_t raw=0;
	while(1)
	{
		raw=0;
		raw = readAdc0Ss3();
		waitMicrosecond(10000000);
		if (raw>600)
		{
			capacitance();
			reset();
			return;
		}
		alliooff();
		raw=0;
		if (raw<600)
		{
			MEAS_C=0;
			raw=0;
			HIGHSIDE_R=0;
			INTEGRATE=0;
			waitMicrosecond(30000);
			LOWSIDE_R=1;
			WTIMER0_TAV_R=0;
			MEAS_LR=1;
			waitMicrosecond(2000000);
			raw = readAdc0Ss3();
			float instantVoltage = ((raw / 4096.0 * 3.3));
			float resr=(106.52/(instantVoltage))-33;
			if (resr>15)
			{
				resistance();
				reset();
				return;
			}
			while (1)
			{
				if (timeUpdate)
				{
					timeUpdate =false ;
					//float L=(33+resr)*time;
					//sprintf(StrTest,"%f",time);
					putsUart0("Measuring Inductance\r\n Inductor Value = ");
					sprintf(StrTest,"%f ", (33+resr)*time/2.5);
					putsUart0(StrTest);
					putcUart0(230);
					putsUart0(" Henry\n\r");
					return;
				}
			}
		}
	}
}

int main(void)
{
	// Initialize hardware
	initHw();
	setTimerMode();
	RED_LED = 1;
	waitMicrosecond(500000);
	RED_LED = 0;
	while(1)
	{
		putsUart0("Enter a String\n\r");
		getString();
		parseString();

		if(isCommand("io",2))
		{
			getCommand();
			ioCommand();
		}

		else if(isCommand("Voltage",0))
		{
			MEAS_LR=0;
			MEAS_C=1;
			LOWSIDE_R=0;
			HIGHSIDE_R=0;
			INTEGRATE=0;
			voltageCommand();
		}

		else if(isCommand("Voltageaverage",0))
		{
			INTEGRATE=1;
			MEAS_C=1;
			voltageCommand();
		}

		else if(isCommand("iooff",0))
			alliooff();

		else if(isCommand("test",0))
		{
			test();
			reset();
		}

		else if (isCommand("resistance",0))
		{
			resistance();
			reset();
		}

		else if(isCommand("capacitance",0))
		{
			capacitance();
			reset();
		}

		else if(isCommand("esr",0))
		{
			uint16_t raw;
			float instantVoltage;
			MEAS_C = 0;
			MEAS_LR=1;
			LOWSIDE_R=1;
			waitMicrosecond(10000);
			raw = readAdc0Ss3();
			instantVoltage = ((raw / 4096.0 * 3.3));
			float resr=(106.029/(instantVoltage))-33;
			sprintf(StrTest,"%f", instantVoltage);
			putsUart0(StrTest);
			putsUart0("\n\r");
			sprintf(StrRes,"%f", resr);
			putsUart0(StrRes);
			putsUart0("\n\r");
			break;
		}

		else if(isCommand("inductance",0))
		{
			inductance();
			waitMicrosecond(300000);
			reset();
		}

		else if(isCommand("auto",0))
			autoCommand();

		else if(isCommand("reset",0))
			reset();

		else
			putsUart0("enter a valid command\n\r");
	}
}
