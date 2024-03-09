/*
 * Machine6-1_code_V3.c
 *
 * Created: 9/9/2022 7:16:43 AM
 * Author : Connor
*/


/*
NOTE: The large linear drive stepper motor on Machine 6-1 has to be connected to the driver in a specific way.
While in one of the smaller stepper motors, there would be a coil connected from the red to blue wires and a coil connected from the black to green wires, with the linear drive motor it is
red to green and black to blue.

The same thing applies to the moving panel motor.
*/


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>


//Defines how many steps in a full rotation, a value of 200 is for use in full step mode on A4988. 400 steps is for use in half step mode on A4988
#define full_rot 400
//Defines the lead for the lead screw, which is equal to (pitch * 2)
//Technically this is a Double Start lead screw, meaning that its lead should be pitch * 2 (which is 3.85 in this case), but after testing the linear drive it seems that the distance
//it actually travels is half of that. This means that even though it is technically a Double Start lead screw, the actual behavior is that of a a Single Start lead screw
#define lead 1.925
//Defines the distance (in millimeters) that the panel should travel backwards starting from the home position (209.55mm-(2.33mm + 10.397mm + 2.873mm) = 196.823mm)
/*
I got 2.873mm from experimentally stretching a precursor fiber and occasionally pushing down on the middle of the fiber until it felt the same way it felt when I pushed down on another fiber that I stretched
to ~0.74 N using a spring scale. By then, I measured the moving panel's distance from the home position and I got a value of 135.45mm. It is interesting to note that (196.823mm distance) - (58.5mm offset) = 138.323, so it was
already pretty close to being stretched as far as I had programmed it to. 
*/
#define distance 196.823
//Defines the offset length that the muscle is already stretched to when the machine is at the home position. This measurement was hard to take, and the actual value is likely between 23mmm-24.5mm
//23.55 is the original offset value I measured where the tips of the Muscle Holders on each panel were nearly making contact, and then I moved the static panel 34.95mm further away from the moving panel. (23.55 + 34.95) = 58.5mm
#define offset 58.5
//exp_offset is a variable that I made to keep track of (I forgor what I was just writing)
#define exp_offset (2.873) //The second number here is found experimentally and reduces the distance the muscle is stretched by a bit to ensure it is harder for it to snap during twisting before coiling.

//Random note below from the research paper:
//1430*0.1511 = ~216.073 turns to coil 151.1 mm main length precursor fiber

//AVR millis() implementation
#define US_PER_OVF	((64 * 256) * 1000L) / (F_CPU / 1000L)
#define MILLIS_INC	(US_PER_OVF / 1000);
#define FRAC_INC	((US_PER_OVF % 1000) >> 3)
#define FRAC_MAX	(1000 >> 3)

//+++++++++++++++++++++++++++++GLOBAL VARIABLES+++++++++++++++++++++++++++++
volatile unsigned long milliseconds = 0;
volatile unsigned char frac_ms = 0;

//The array to store the time between the beginning of panel gearbox movement and the beginning of linear drive movement, as well as the time between the beginning of linear drive movement and MACHINE STOP
//My apologies if that sentence above is not very comprehensible, when writing this I was on only a few hours of sleep
unsigned long times[2];

//For use in writeByte() function
int errorCode = 0;

//Button flag
int flag = 0;

//Unused feature for speed control during the homing sequence
//uint8_t samplesFlag = 0;

//This incredibly large array will hold the all sampled potentiometer values.
//When the program is over and these need to be displayed, I will implement a compression algorithm that will enable me to more efficiently record these values off of the OLED display and input them into a larger dataset
//I made this array 3750 entries long is because (60s / 16ms) = 3750, which is roughly how many times the potentiometer would be sampled if the linear drive was moving forwards for a whole minute (which is unlikely to happen, it will probably
//run for less time than that). Either way, it should have more than enough space to record all the potentiometer samples until the linear drive is stopped
//volatile uint16_t potSamplesIndex = 0;
//volatile uint16_t potSamples[3750];

//Stores each sampled value from the potentiometer before it is entered into potSamples[]
//volatile uint16_t sampVal = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++INTERRUPT SERVICE ROUTINES++++++++++++++++++++++++
//AVR millis() implementation
ISR(TIMER0_OVF_vect)
{
	unsigned long m = milliseconds;
	//Backup the contents in SREG before performing any math operations
	unsigned char store_sreg = SREG;
	
	m += MILLIS_INC;
	frac_ms += FRAC_INC;
	if (frac_ms >= FRAC_MAX)
	{
		frac_ms -= FRAC_MAX;
		m++;
	}
	
	milliseconds = m;
	//Restore backed up version of SREG
	SREG = store_sreg;
}

//Pin 19 on Arduino Mega
//Triggers linear drive to stop moving, which means the panel is at the home position
ISR(INT2_vect)
{
	flag = 1;
}

//Pin 18 on Arduino Mega
//CONTINUE Button
ISR(INT3_vect)
{
	flag = 2;
	PORTB |= (1<<PB0);
	//_delay_ms(30);
	//PORTB &= (0<<PB0);
	//_delay_ms(30);
}

//Pin 20 on Arduino Mega
//WRAP HEATING ELEMENT Button
ISR(INT1_vect)
{
	flag = 3;
	PORTB |= (1<<PB0);
	//_delay_ms(30);
	//PORTB &= (0<<PB0);
	//_delay_ms(30);
}

/*
//Pin 3 on Arduino Mega
//STOP Button
ISR(INT5_vect)
{
	//Emergency stop button. I don't like the idea of having an infinite loop inside an ISR but that is the easiest and most efficient way to implement an E-STOP control that I could think of.
	while (1)
	{
		PORTB |= (1<<PB0);
		_delay_ms(1000);
		PORTB &= (0<<PB0);
		_delay_ms(1000);
	}
}
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void initTimer0()
{
	//Sets Timer/Counter0 to Fast PWM mode
	TCCR0A = (1<<WGM01) | (1<<WGM00);
	
	//Sets the clock prescaler to divide by 64 (clk(T0S) / 64), page 131 of ATMega 2560 datasheet (clk(T0S) is the same as the system clock, which is 16MHz)
	TCCR0B = (1<<CS01) | (1<<CS00);
	
	//Enable TimerCounter0 Overflow interrupt
	TIMSK0 |= (1<<TOIE0);
	
	sei();
}

void clearTimer0()
{
	cli();
	
	TCCR0A = 0;
	TCCR0B = 0;
	TIMSK0 = 0;
}

//AVR millis() implementation, except milliseconds is set back to 0 before returning m
unsigned long millis()
{
	unsigned long m;
	unsigned char oldSREG = SREG;
	
	clearTimer0();
	
	m = milliseconds;
	SREG = oldSREG;
	
	//Resets the millisecond counter, after calling millis() you will need to call initTimer0() again
	milliseconds = 0;
	frac_ms = 0;
	
	return m;
}

void greenBlink(int num)
{
	for (int i = 0; i < num; i++)
	{
		PORTB |= (1<<PB0);
		_delay_ms(400);
		PORTB &= (0<<PB0);
		_delay_ms(400);
	}
}

void redBlink()
{
	PORTB |= (1<<PB5);
	_delay_ms(700);
	PORTB &= (0<<PB5);
	_delay_ms(700);
}

//When flag == 0, this method will be used to print out time. When flag == 1, this method will be used to print out a formatted potentiometer sample
void displayNum(unsigned long val, uint8_t flag)
{
	//The first character extracted from m should be the last character printed on the screen, otherwise it will print the recorded time backwards and it will make me think the error is something else
	//Since character_stack is 6 elements long which means it can hold 6 characters used to represent a time up to as large as 16 minutes
	uint8_t index = 0;
	uint8_t character_stack[6];
	
	uint8_t val_temp;
	while (val > 0)
	{
		val_temp = val % 10;
		val /= 10;
		character_stack[index] = val_temp;
		index++;
	}
	
	for (int i = 0; i < 3; i++)
	{
		PORTL |= (1<<PL0);
		_delay_ms(300);
		PORTL &= (0<<PL0);
		_delay_ms(300);
	}
	
	//If printing out a potentiometer sample, turn on indicator LED to indicate that it is displaying Pot Samples
	if (flag)
	{
		PORTL |= (1<<PL0);
	}
	
	//Prints out the number starting from the MOST significant digit
	for (int i = (index - 1); i >= 0; i--) //for (int i = 0; i < index; i++)
	{
		greenBlink(character_stack[i]);
		redBlink();
	}
}

/*
void displayPotSamples()
{
	//Clear interrupts so that potentiometer samples don't keep getting taken and cause potSamplesIndex to increase, causing the program to get stuck in an infinite loop during the for loop below
	cli();
	
	//displayNum(potSamplesIndex, 0);
	//_delay_ms(1500);
	
	//If there are no samples recorded, then blink both LEDs (this means that something isn't working properly, there should always be samples recorded even if the program is running for only a few ms
	if (!potSamplesIndex)
	{
		while (1)
		{
			PORTB = (1<<PB0) | (1<<PB5);
			_delay_ms(500);
			PORTB = (0<<PB0) & (0<<PB5);
			_delay_ms(500);
		}
	}
	
	for (uint16_t i = 0; i < potSamplesIndex; i++)
	{
		displayNum(potSamples[i], 1);
	}
}
*/

//Converts time in milliseconds into character bitmaps which are written to the GDDRAM to be displayed on the OLED screen
void displayTime(unsigned long m)
{
	displayNum(m, 0);
}

/*
//Slowly accelerates the linear drive from the speed used to initiate coiling to the speed used to sustain coiling
//startVal should be 423 and endVal should be 178
void accelerate_LinearDrive(int startVal, int endVal)
{
	for (int i = startVal; i > endVal; i--)
	{
		for (int k = 0; k < 32; k++)
		{
			PORTB |= (1<<PB4);
			for (int j = 0; j < i; j++)
			{
				_delay_us(10);
			}
			PORTB &= (0<<PB4);
			for (int j = 0; j < i; j++)
			{
				_delay_us(10);
			}
		}
	}
}
*/

//Pins 8 and 9 Arduino Mega are STEP pins for the Panel Motors
//Works on Pin 8 (H5 on TCCR4A & OCR4C) and Pin 9 (H6 on TCCR2A & OCR2B) of Arduino Mega
/*
(16 * 10^6) * (1/1024) * (7/256) = 427.2460938 Hz
*/
void analogWriteV2(int val)
{
	DDRH = (1<<DDH5) | (1<<DDH6);
	
	if (val == 0)
	{
		TCCR2A &= (0<<COM2B1);
		TCCR4A &= (0<<COM4C1);
		PORTH = (0<<PH5) & (0<<PH6);
	}
	if (val == 255)
	{
		PORTH = (1<<PH5) | (1<<PH6);
	}
	else
	{
		TCCR2A |= (1<<WGM20);
		TCCR2A |= (1<<COM2B1);
		
		TCCR4A |= (1<<WGM40);
		TCCR4A |= (1<<COM4C1);
		
		//Sets the clock prescaler to divide by 1024 (clk(T2S) / 1024), page 188 of ATMega 2560 datasheet
		//TCCR2B = (1<<CS20) | (1<<CS21) | (1<<CS22);
		
		//Sets the clock prescaler to divide by 256 (clk(T2S) / 256), page 188 of ATMega 2560 datasheet
		TCCR2B = (1<<CS22) | (1<<CS21);
		//Sets the clock prescaler to divide by 256 (clk(T4S) / 256), page 162 of ATMega 2560 datasheet
		TCCR4B |= (1<<CS42);
		
		//Sets the clock prescaler to divide by 128 (clk(T2S) / 128), page 188 of ATMega 2560 datasheet
		//TCCR2B = (1<<CS20) | (1<<CS22);
		
		//Sets the clock prescaler to divide by 64 (clk(T2S) / 64), page 188 of ATMega 2560 datasheet
		//TCCR2B |= (1<<CS22);
		//CHECK THE DATASHEET pg. 162, TCCR4B DOES NOT HAVE A (clk(T4S) / 32) PRESCALER OR A (clk(T4S) / 128) PRESCALER! YOU MUST ENABLE CS41 AND CS40 TO SET IT TO A PRECSALER OF 64!
		//TCCR4B = (1<<CS41) | (1<<CS40);
		
		//Sets the clock prescaler to divide by 32 (clk(T2S) / 32), page 188 of ATMega 2560 datasheet
		//TCCR2B = (1<<CS20) | (1<<CS21);
		
		//Sets the clock prescaler to divide by 8 (clk(T2S) / 8), page 188 of ATMega 2560 datasheet
		//TCCR2B |= (1<<CS21);
		
		OCR2B = val;
		OCR4C = val;
	}
}

void waitContinue() {
	//Enable interrupts on INT3
	EIMSK |= (1<<INT3);
	
	sei();
	
	//Wait for CONTINUE button to be pressed
	flag = 0;
	while (flag != 2)
	{
		//Indicator LED
		PORTB |= (1<<PB5);
	}
	cli();
}

void wait() {
	//Enable interrupts on INT1 and INT3
	EIMSK = (1<<INT3) | (1<<INT1);
	
	sei();
	
	//Wait for either button to be pressed
	flag = 0;
	while (flag != 2 && flag != 3) {
		//Indicator LED
		PORTB |= (1<<PB5);
	}
	cli();
	
	//Disable interrupts for both buttons for a brief period
	EIMSK = (0<<INT3) & (0<<INT1);
	//INT1 will never be used again for the rest of the program, so turn off the pin to save power
	//PORTD &= (0<<PD1); //OK for some reason when I uncomment this line it seems to keep triggering the INT1 ISR because INT1 is set to trigger when PD1 is low
	
	_delay_ms(30);
	PORTB &= (0<<PB0);
}

void machineInit() {
	cli();
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Machine Setup
	
	//Set Pin 10 to output mode
	//Pin 10 Arduino Mega is STEP pin for the Linear Drive Motor
	DDRB |= (1<<DDB4);
	
	//Pin 9 Arduino Mega is STEP pin for a Panel Motor
	DDRH |= (1<<DDH6);
	
	//Pin 8 Arduino Mega is STEP pin for a Panel Motor
	DDRH |= (1<<DDH5);
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	//Pin 5 Arduino Mega is DIR pin for the Linear Drive Motor
	DDRE |= (1<<DDE3);
	
	//Pin 4 Arduino Mega is DIR pin for a Panel Motor
	DDRG |= (1<<DDG5);
	
	//Pin 14 Arduino Mega is DIR pin for a Panel Motor
	DDRJ |= (1<<DDJ1);
	
	/*
	PINS 6, 16 AND 17 ON THE ARDUINO MEGA ARE BROKEN AND DO NOT WORK, THEY STAY LOW CONSTANTLY FOR SOME REASON I DON'T KNOW WHY so use different pins instead
	*/
	//-------------------------------------------------------------------------------------------------------
	//Clockwise rotation moves panel forward & Counterclockwise rotation moves panel backwards
	//When DIR is high, linear drive motor rotates clockwise and moves panel forward
	PORTE |= (1<<PE3);
	//Keep Panel direction pins LOW (or HIGH), whichever direction has less gear slipping
	//PORTG &= (0<<PG5);
	//PORTJ &= (0<<PJ1);
	PORTG |= (1<<PG5);
	PORTJ |= (1<<PJ1);
	//-------------------------------------------------------------------------------------------------------
	//Make sure INT1/INT2/INT3 pins is in input mode with the internal pullup resistors enabled
	DDRD = (0<<DDD3) & (0<<DDD2) & (0<<DDD1);
	PORTD = (1<<PD3) | (1<<PD2) | (1<<PD1);
	
	//DDRE &= (0<<DDE5);
	//PORTE |= (1<<PE5);
	
	MCUCR &= (0<<PUD);
	//-------------------------------------------------------------------------------------------------------
	
	//Beginning sample display indicator
	DDRL |= (1<<DDL0);
	PORTL &= (0<<PL0);
	
	//Startup indicator
	for (int i = 0; i < 2; i++)
	{
		PORTL |= (1<<PL0);
		_delay_ms(300);
		PORTL &= (0<<PL0);
		_delay_ms(300);
	}
	
	
	//Button pressed indicator LED (green) will be on Pin 53 on the Arduino Mega
	DDRB |= (1<<DDB0);
	PORTB &= (0<<PB0);
	
	//Startup indicator
	for (int i = 0; i < 2; i++)
	{
		PORTB |= (1<<PB0);
		_delay_ms(300);
		PORTB &= (0<<PB0);
		_delay_ms(300);
	}
	
	
	//Startup indicator LED will be on Pin 11 on the Arduino Mega
	DDRB |= (1<<DDB5);
	PORTB &= (0<<PB5);
	
	//Startup indicator
	for (int i = 0; i < 3; i++)
	{
		PORTB |= (1<<PB5);
		_delay_ms(300);
		PORTB &= (0<<PB5);
		_delay_ms(300);
	}
	_delay_ms(1000);
	//-------------------------------------------------------------------------------------------------------
	//Low level on INT1, INT2, and INT3 each trigger an interrupt (page 76 of ATmega2560 datasheet)
	EICRA = (0<<ISC31) & (0<<ISC30) & (0<<ISC21) & (0<<ISC20) & (0<<ISC11) & (0<<ISC10);
	//EICRB = (0<<ISC51) & (0<<ISC50);
	
	//Enable interrupts on INT1 and INT2
	EIMSK = (1<<INT2) | (1<<INT1);
	//EIMSK |= (1<<INT2);
}

void home() {
	//Homing sequence
	sei();
	flag = 0;
	while (flag != 1)
	{
		PORTB |= (1<<PB4);
		_delay_ms(1.250);
		PORTB &= (0<<PB4);
		_delay_ms(1.250);
	}
	cli();
	
	
	//Disable interrupts for homing endstop
	EIMSK &= (0<<INT2);
}

void stretch() {
	//Precursor Fiber Stretching
	
	//Invert the direction of the linear drive motor so that the panel mechanism moves backwards
	PORTE &= (0<<PE3);
	
	//Wait a moment before beginning backwards movement so that it doesn't jerk the whole mechanism
	_delay_ms(500);
	
	sei();
	
	
	/*
	//Move backwards the required distance to stretch the precursor fibers to ~0.731 N of tension
	for (int i = 0; i < ((int)(full_rot * (distance / lead))); i++)
	{
		PORTB |= (1<<PB4);
		_delay_ms(1.250);
		PORTB &= (0<<PB4);
		_delay_ms(1.250);
	}
	*/
	
	
	/*
	
	IMPORTANT NOTE: Remember to connect the potentiometer legs correctly. The middle leg should be connected to the ADC pin, and the right leg with the Ohm symbol directly above it should be connected to
	ground. Finally, the left-most leg should be connected to +5V.
	
	If you connect the right leg to +5V and left leg to ground accidentally, then when you think you have turned down the potentiometer (by turning it counterclockwise) all the way, the code will automatically
	use the default homing speed and square wave period of 1250us and it will disable speed control.
	
	*/
	
	
	int scaler = 3; //1
	int steps = ((int)(full_rot * ((distance - (offset + exp_offset)) / lead)));
	int thres_slow = steps * 0.75;
	int thres_slowest = steps * 0.95;
	
	//Move backwards the required distance to stretch the precursor fibers to ~0.731 N of tension
	for (int i = 0; i < steps; i++)
	{
		PORTB |= (1<<PB4);
		for (int j = 0; j < scaler; j++)
		{
			//_delay_ms(1.250);
			_delay_us(200);
		}
		//The only reason I am putting this if statement here and not with the other one is because the comparison between i and thres_slow takes time, just like the comp. between i and thres_slowest. This might make the square wave slightly more even
		if (i >= thres_slow)
		{
			scaler = 6;
		}
		PORTB &= (0<<PB4);
		for (int j = 0; j < scaler; j++)
		{
			//_delay_ms(1.250);
			_delay_us(200);
		}
		if (i >= thres_slowest) //Slows down the stepper as it gets very close to the required distance, just to make sure it doesn't over or undershoot the distance by much
		{
			scaler = 44; //7
		}
	}
	
	
	cli();
	
	//Added delay so that one button press doesn't accidentally count as multiple and skip through phases of machine operation
	_delay_ms(500);
	
	//Set the direction of the linear drive so that the panel mechanism moves forwards towards the other panel
	PORTE |= (1<<PE3);
}

void twist() {
	//Begin Precursor Fiber Twisting
	
	waitContinue();
	
	PORTB &= (0<<PB5);
	
	_delay_ms(30);
	PORTB &= (0<<PB0);
	
	//Begin Panel Motor movement
	analogWriteV2(128);
	
	//When the clock prescalers in analogWriteV2() are set to 256, it takes 57.25 seconds for 35 full rotations of each loop gear. The muscles need to be twisted 35 times before activating the linear drive motor.
	//57.25 seconds == 57250 milliseconds, and (57250 / 250) = 229
	//You should change this for loop to a different delay time whenever you change the clock prescaler in analogWriteV2().
	for (int i = 0; i < 229; i++)
	{
		_delay_ms(250);
	}
}

void move() {
	//Begin Linear Drive Movement (with linear movement speed control)
	
	//==============================
	//Implement speed control below
	
	//Indicator LED on
	PORTB |= (1<<PB5);
	
	//initTimer0();
	
	//accelerate_LinearDrive(823, 423);
	
	//Begin linear drive movement and speed control loop 1
	//The potentiometer value to initiate coiling in this section of the code is 353, so to get the calculated delay value of 4230us that I am using here, I simply did 10(353 + 70) by replacing sampVal with 353
	//During calibration, I found that the proper time the linear drive should move at the initial speed before speeding up is 89893ms, or 89.893s. (89893 / (2 * 4.850)) = 9267.319588, or ~9267
	for (int i = 0; i < 9267; i++)
	{
		PORTB |= (1<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(485); //423
		}
		PORTB &= (0<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(485); //423
		}
	}
	
	/*
	flag = 0;
	while (flag != 2)
	{
		PORTB |= (1<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(485); //423
		}
		PORTB &= (0<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(485); //423
		}
	}
	
	//Record the first time
	times[0] = millis();
	*/
	
	
	//Allows some time for the button to release after I press it (switch de-bouncing)
	//_delay_ms(70);
	//PORTB &= (0<<PB0);
	
	//initTimer0();
	
	//Accelerates the linear drive motor to sustain coiling speed slowly and over time so that the fishing line does not snarl during operation
	//accelerate_LinearDrive(423, 200); //(423, 178)
	
	//Begin linear drive movement and speed control loop 2
	//The potentiometer value to sustain coiling in this section of the code is 108, so to get the calculated delay value of 1780us that I am using here, I simply did 10(108 + 70) by replacing sampVal with 108
	//During calibration, I found that the proper time the linear drive should move at the final speed before stopping all linear drive movement is ~64912ms, or ~64.912s. (64912 / (2 * 2.000)) = 16228
	for (int i = 0; i < 16228; i++)
	{
		PORTB |= (1<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(200); //178
		}
		PORTB &= (0<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(200); //178
		}
	}
	
	/*
	flag = 0;
	while (flag != 2)
	{
		PORTB |= (1<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(200); //178
		}
		PORTB &= (0<<PB4);
		for (uint8_t i = 0; i < 10; i++)
		{
			_delay_us(200); //178
		}
	}
	//==============================
	*/
}

void stop() {
	//Record the second time
	//times[1] = millis();
	
	//Stop panel movement, linear drive movement is already stopped
	analogWriteV2(0);
	
	//Indicator LED off
	PORTB &= (0<<PB5);
	
	//_delay_ms(30);
	//PORTB &= (0<<PB0);
	
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//Indicator
	for (int i = 0; i < 5; i++)
	{
		PORTB |= (1<<PB0);
		_delay_ms(300);
		PORTB &= (0<<PB0);
		_delay_ms(300);
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	PORTB |= (1<<PB5);
}

void wrapHeatingElement() {
	//Stretch newly formed muscles slightly before wrapping heating element
	
	//Invert the direction of the linear drive motor so that the panel mechanism moves backwards
	PORTE &= (0<<PE3);
	//Wait a moment before beginning backwards movement so that it doesn't jerk the whole mechanism
	_delay_ms(500);
	
	for (int i = 0; i < ((int)(full_rot * (18 / lead))); i++) //Stretch muscle 18 mm starting from the home position before wrapping heating element
	{
		PORTB |= (1<<PB4);
		_delay_ms(1.250);
		PORTB &= (0<<PB4);
		_delay_ms(1.250);
	}
	
	//Begin heating element wrapping
	while (1) {
		//Indicator
		PORTB |= (1<<PB0);
		
		_delay_ms(300); //Switch de-bouncing
		waitContinue();
		
		//Indicator
		PORTB &= (0<<PB0);
		
		//Reverse the direction of one of the panels
		//PORTG |= (1<<PG5);
		//PORTJ |= (1<<PJ1);
		PORTG &= (0<<PG5);
		//PORTJ &= (0<<PJ1);
		
		//Begin Panel Motor movement
		analogWriteV2(128);
		
		_delay_ms(300); //Switch de-bouncing
		waitContinue();
		
		//Finish and stop all movement
		analogWriteV2(0);
	}
}

/*
void displayData() {
	//Display recorded data
	
	//Wait for Continue button press before first time is displayed
	sei();
	flag = 0;
	while (flag != 2)
	{
		PORTB |= (1<<PB5);
	}
	cli();
	
	_delay_ms(30);
	PORTB &= (0<<PB0);
	
	displayTime(times[0]);
	
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//Indicator
	for (int i = 0; i < 5; i++)
	{
		PORTB |= (1<<PB5);
		_delay_ms(500);
		PORTB &= (0<<PB5);
		_delay_ms(500);
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	//Wait for Continue button press before second time is displayed
	sei();
	flag = 0;
	while (flag != 2)
	{
		PORTB |= (1<<PB5);
	}
	cli();
	
	displayTime(times[1]);
}
*/

int main(void)
{
	machineInit();
	home();
	
	wait();
	if (flag == 3) {
		wrapHeatingElement();
	}
	
	stretch();
	twist();
	move();
	stop();
	//wrapHeatingElement();
	
	//displayData();
}