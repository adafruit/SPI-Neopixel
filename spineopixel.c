#ifndef F_CPU
#error F_CPU is not defined, pick one using compiler options
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <util/delay.h>
#include <stdlib.h>

typedef enum {
	INPUTMODE_SPI = 0,
	INPUTMODE_UART = 1,
	INPUTMODE_I2C = 2,
}
inputmode_t;

typedef enum
{
	USI_SLAVE_CHECK_ADDRESS = 0,
	USI_SLAVE_SEND_DATA,
	USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA,
	USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA,
	USI_SLAVE_REQUEST_DATA,
	USI_SLAVE_GET_DATA_AND_SEND_ACK,
}
USI_TWI_state_t;

// pin definitions here

// input should be the USI
#define IN_CLK_PORTx	PORTB
#define IN_CLK_DDRx		DDRB
#define IN_CLK_PINx		PINB
#define IN_CLK_BIT		2
#define IN_CLK_MASK		(1 << IN_CLK_BIT)
#define IN_DATA_PORTx	PORTB
#define IN_DATA_DDRx	DDRB
#define IN_DATA_PINx	PINB
#define IN_DATA_BIT		0
#define IN_DATA_MASK	(1 << IN_DATA_BIT)
#define IN_LATCH_PORTx	PORTB
#define IN_LATCH_DDRx	DDRB
#define IN_LATCH_PINx	PINB
#define IN_LATCH_BIT	3
#define IN_LATCH_MASK	(1 << IN_LATCH_BIT)

// USI DO needs to be known
#define USI_DO_PORTx	PORTB
#define USI_DO_DDRx		DDRB
#define USI_DO_PINx		PINB
#define USI_DO_BIT		1
#define USI_DO_MASK		(1 << USI_DO_BIT)

// NeoPixel output pin
#define OUT_PORTx		PORTB
#define OUT_DDRx		DDRB
#define OUT_PINx		PINB
#define OUT_BIT			4
#define OUT_MASK		(1 << OUT_BIT)

// mode select jumper
// must have a 1 Mohm resistor to ground, we need to detect 3 states
// VCC = SPI, ->1M->GND = I2C, N/C = UART
#define JMP_PORTx		PORTB
#define JMP_DDRx		DDRB
#define JMP_PINx		PINB
#define JMP_BIT			1
#define JMP_MASK		(1 << JMP_BIT)

// speed select
#define IS_NEO_KHZ800()			(1)	// 400 KHz is deprecated

// latch macros
// drive low to indicate we are busy, no data is accepted during this time
#define LATCH_BUSY()	do { IN_LATCH_DDRx |= IN_LATCH_MASK; IN_LATCH_PORTx &= ~IN_LATCH_MASK; } while (0)
// release the pin (set as input) and activate the pull up resistor, waiting for the falling edge
#define LATCH_IDLE()	do { IN_LATCH_PORTx |= IN_LATCH_MASK; IN_LATCH_DDRx &= ~IN_LATCH_MASK; } while (0)

// private function prototypes
static void
	spi_handler(void),
	i2c_handler(void),
	uart_handler(void),
	send_to_neopixel(void);
static inputmode_t get_input_mode(void);

static volatile USI_TWI_state_t USI_TWI_Overflow_State = USI_SLAVE_CHECK_ADDRESS; // 0 means "have not been addressed"
static volatile inputmode_t input_mode;

// defines for UART input mode
#ifndef BAUDRATE
	#error Use the compiler options to define a BAUDRATE
#endif

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define TIMER_PRESCALER           1
#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER)
#define TIMER0_SEED               (256 - ( (F_CPU / BAUDRATE) / TIMER_PRESCALER ))

#if ( (( (F_CPU / BAUDRATE) / TIMER_PRESCALER ) * 3/2) > (256 - INTERRUPT_STARTUP_DELAY) )
	#define INITIAL_TIMER0_SEED       ( 256 - (( (F_CPU / BAUDRATE) / TIMER_PRESCALER ) * 1/2) )
	#define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
	#define INITIAL_TIMER0_SEED       ( 256 - (( (F_CPU / BAUDRATE) / TIMER_PRESCALER ) * 3/2) )
	#define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
#endif

static volatile char is_rxing = 0;

// global vars
#define BUFFER_SIZE		((RAMEND - RAMSTART) - (32 * 3)) // if the buffer is too big then we might get a stack collision
static volatile uint8_t* buffer;
static volatile uint16_t buffer_idx = 0;

int main(void)
{
	// we drive the latch low at startup to indicate that we are busy
	IN_LATCH_DDRx  |=  IN_LATCH_MASK;
	IN_LATCH_PORTx &= ~IN_LATCH_MASK;

	buffer = (uint8_t*)malloc(BUFFER_SIZE + 1); // compile time allocation doesn't seem to work for some reason... so here we do run time allocation

	// setup pin directions
	IN_CLK_DDRx &= ~IN_CLK_MASK;
	IN_DATA_DDRx &= ~IN_DATA_MASK;
	OUT_DDRx |= OUT_MASK;
	// no pull-up resistors
	IN_CLK_PORTx &= ~IN_CLK_MASK;
	IN_DATA_PORTx &= ~IN_DATA_MASK;
	// default low
	OUT_PORTx &= ~OUT_MASK;

	// For testing, call a handler directly:
	//spi_handler();
	//i2c_handler();
	//uart_handler();

	while (1)
	{
		input_mode = get_input_mode();
		if (input_mode == INPUTMODE_SPI) {
			spi_handler();
		}
		else if (input_mode == INPUTMODE_I2C) {
			i2c_handler();
		}
		else if (input_mode == INPUTMODE_UART) {
			uart_handler();
		}
	}

	// should never be able to reach here
	return 0;
}

static void spi_handler(void)
{
	uint8_t cur_latch, last_latch;

	input_mode = INPUTMODE_SPI;

	while (1)
	{
		// USI setup for slave 
		USICR = _BV(USIWM0) | _BV(USICS1); // 3 wire mode, positive edge triggered
		USISR = _BV(USIOIF); // clears interrupt and resets counter

		LATCH_IDLE();

		// reset falling edge detection
		last_latch = IN_LATCH_MASK;
		cur_latch = IN_LATCH_MASK;

		while (1)
		{
			if ((last_latch & IN_LATCH_MASK) != 0 && (cur_latch & IN_LATCH_MASK) == 0) // falling edge
			{
				LATCH_BUSY();

				if (buffer_idx > 0) // only if at least one byte is RX'ed
				{
					// shutdown the USI
					USICR = 0;
					USISR = 0;

					send_to_neopixel();
					_delay_us(25); // at 500 KHz, it is possible to send 3 bytes in under 50 us, a delay here is needed to ensure the NeoPixels latch
				}

				break; // exit out of the loop reset USI
			}

			if (bit_is_set(USISR, USIOIF)) // new data arrived
			{
				if (buffer_idx < BUFFER_SIZE) buffer[buffer_idx++] = USIDR; // take input w/o overflow
				USISR |= _BV(USIOIF); // clear the flag
			}

			// falling edge detection
			last_latch = cur_latch;
			cur_latch = IN_LATCH_PINx;
		}
	}
}

static void uart_handler(void)
{
	uint8_t cur_latch, last_latch;

	input_mode = INPUTMODE_UART;

	while (1)
	{
		USICR  =  0;			// Disable USI, will be enabled by PCINT
		GIFR   =  (1<<PCIF);	// Clear pin change interrupt flag
		GIMSK |=  (1<<PCIE);	// Enable pin change interrupt
		PCMSK |=  (1<<PCINT0);	// Use DI pin
		is_rxing = 0;
		sei();

		LATCH_IDLE();

		// reset falling edge detection
		last_latch = IN_LATCH_MASK;
		cur_latch = IN_LATCH_MASK;

		while (1)
		{
			if ((last_latch & IN_LATCH_MASK) != 0 && (cur_latch & IN_LATCH_MASK) == 0) // falling edge
			{
				LATCH_BUSY();

				if (buffer_idx > 0) // only if at least one byte is RX'ed
				{
					// shutdown the USI
					USICR = 0;
					USISR = 0;
					// shutdown UART timer
					TCCR0B = 0;

					send_to_neopixel();
					// no delay needed because there's no way to send 3 bytes of data in under 50 us (NeoPixel's latch timeout)
				}

				break; // exit out of the loop reset USI
			}

			// falling edge detection
			last_latch = cur_latch;
			cur_latch = IN_LATCH_PINx;
		}
	}
}

static void i2c_handler(void)
{
	uint8_t cur_latch, last_latch;
	char has_trigger;

	input_mode = INPUTMODE_I2C;

	LATCH_IDLE();

	while (1)
	{
		USICR =	(1<<USISIE)|(0<<USIOIE)|				// Enable Start Condition Interrupt. Disable Overflow Interrupt.
				(1<<USIWM1)|(1<<USIWM0)|				// Set USI in Two-wire mode. No USI Counter overflow prior
														// to first Start Condition (potentail failure)
				(1<<USICS1)|(0<<USICS0)|(0<<USICLK)|	// Shift Register Clock Source = External, positive edge
				(0<<USITC);
		USISR = 0xF0;									// Clear all flags and reset overflow counter
		IN_CLK_DDRx &= ~IN_CLK_MASK;
		IN_DATA_DDRx &= ~IN_DATA_MASK;
		sei();

		// reset rising edge detection
		last_latch = 0;
		cur_latch = 0;
		has_trigger = 0;

		while (1)
		{
			if (bit_is_set(USISR, USIPF)) {
				is_rxing = 0;
				// clear the stop condition flag if set
				USISR |= _BV(USIPF);
			}

			if ((last_latch & IN_LATCH_MASK) == 0 && (cur_latch & IN_LATCH_MASK) != 0) // rising edge
			{
				// or if the latch pin is disconnected, this if statement will also be triggered
				has_trigger = 1;
			}

			if (has_trigger != 0 && is_rxing == 0)
			{
				has_trigger = 0;

				if (buffer_idx > 0)
				{
					// shutdown the USI
					USICR = 0;
					USISR = 0;

					// occupy the bus to disallow incoming communication, indicate that it is busy
					IN_DATA_PORTx &= ~IN_DATA_MASK;
					IN_DATA_DDRx  |=  IN_DATA_MASK;
					IN_CLK_DDRx   |=  IN_CLK_MASK;
					IN_CLK_PORTx  &= ~IN_CLK_MASK;

					send_to_neopixel();
					// no delay needed because there's no way to send 3 bytes of data in under 50 us (NeoPixel's latch timeout)

					break; // exit out of the loop reset USI
				}
			}

			// falling edge detection
			last_latch = cur_latch;
			cur_latch = IN_LATCH_PINx;
		}
	}
}

static void send_to_neopixel(void)
{
	cli(); // timing critical code below, no interrupts allowed

	// code below is copied from Adafruit_NeoPixel.cpp

	// Global variable buffer_idx is altered by this function.  It will
	// always equal zero on return, indicating an empty buffer.

	volatile uint8_t next, bit;
	volatile uint8_t
	*ptr	= &buffer[0],	// Pointer to next byte
	b		= *ptr++,		// Current byte value
	hi,						// PORT w/output bit set high
	lo;						// PORT w/output bit set low

	volatile uint8_t* port = &OUT_PORTx;
	hi = OUT_PORTx |  OUT_MASK;
	lo = OUT_PORTx & ~OUT_MASK;

	// Hand-tuned assembly code issues data to the LED drivers at a specific
	// rate.  There's separate code for different CPU speeds (8, 12, 16 MHz)
	// for both the WS2811 (400 KHz) and WS2812 (800 KHz) drivers.  The
	// datastream timing for the LED drivers allows a little wiggle room each
	// way (listed in the datasheets), so the conditions for compiling each
	// case are set up for a range of frequencies rather than just the exact
	// 8, 12 or 16 MHz values, permitting use with some close-but-not-spot-on
	// devices (e.g. 16.5 MHz DigiSpark).  The ranges were arrived at based
	// on the datasheet figures and have not been extensively tested outside
	// the canonical 8/12/16 MHz speeds; there's no guarantee these will work
	// close to the extremes (or possibly they could be pushed further).
	// Keep in mind only one CPU speed case actually gets compiled; the
	// resulting program isn't as massive as it might look from source here.

// 8 MHz(ish) AVR ---------------------------------------------------------
#if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)

	if (IS_NEO_KHZ800())
	{ // 800 KHz bitstream

		volatile uint8_t n1, n2 = 0;  // First, next bits out

		// Squeezing an 800 KHz stream out of an 8 MHz chip requires code
		// specific to each PORT register.
		// this covers all the pins on the Adafruit Flora and the bulk of
		// digital pins on the Arduino Pro 8 MHz (keep in mind, this code
		// doesn't even get compiled for 16 MHz boards like the Uno, Mega,
		// Leonardo, etc., so don't bother extending this out of hand).
		// Additional PORTs could be added if you really need them, just
		// duplicate the else and loop and change the PORT.  Each add'l
		// PORT will require about 150(ish) bytes of program space.

		// 10 instruction clocks per bit: HHxxxxxLLL
		// OUT instructions:              ^ ^    ^   (T=0,2,7)

		n1 = lo;
		if(b & 0x80) n1 = hi;

		// Dirty trick: RJMPs proceeding to the next instruction are used
		// to delay two clock cycles in one instruction word (rather than
		// using two NOPs).  This was necessary in order to squeeze the
		// loop down to exactly 64 words -- the maximum possible for a
		// relative branch.

		asm volatile(
			"headB:"                  "\n\t"	// Clk  Pseudocode
			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n2]   , %[lo]"    "\n\t"	// 1    n2   = lo
			"out  %[port] , %[n1]"    "\n\t"	// 1    PORT = n1
			"rjmp .+0"                "\n\t"	// 2    nop nop
			"sbrc %[byte] , 6"        "\n\t"	// 1-2  if(b & 0x40)
			"mov %[n2]   , %[hi]"     "\n\t"	// 0-1    n2 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"rjmp .+0"                "\n\t"	// 2    nop nop

			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n1]   , %[lo]"    "\n\t"	// 1    n1   = lo
			"out  %[port] , %[n2]"    "\n\t"	// 1    PORT = n2
			"rjmp .+0"                "\n\t"	// 2    nop nop
			"sbrc %[byte] , 5"        "\n\t"	// 1-2  if(b & 0x20)
			"mov %[n1]   , %[hi]"     "\n\t"	// 0-1    n1 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"rjmp .+0"                "\n\t"	// 2    nop nop

			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n2]   , %[lo]"    "\n\t"	// 1    n2   = lo
			"out  %[port] , %[n1]"    "\n\t"	// 1    PORT = n1
			"rjmp .+0"                "\n\t"	// 2    nop nop
			"sbrc %[byte] , 4"        "\n\t"	// 1-2  if(b & 0x10)
			"mov %[n2]   , %[hi]"     "\n\t"	// 0-1    n2 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"rjmp .+0"                "\n\t"	// 2    nop nop

			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n1]   , %[lo]"    "\n\t"	// 1    n1   = lo
			"out  %[port] , %[n2]"    "\n\t"	// 1    PORT = n2
			"rjmp .+0"                "\n\t"	// 2    nop nop
			"sbrc %[byte] , 3"        "\n\t"	// 1-2  if(b & 0x08)
			"mov %[n1]   , %[hi]"     "\n\t"	// 0-1    n1 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"rjmp .+0"                "\n\t"	// 2    nop nop

			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n2]   , %[lo]"    "\n\t"	// 1    n2   = lo
			"out  %[port] , %[n1]"    "\n\t"	// 1    PORT = n1
			"rjmp .+0"                "\n\t"	// 2    nop nop
			"sbrc %[byte] , 2"        "\n\t"	// 1-2  if(b & 0x04)
			"mov %[n2]   , %[hi]"     "\n\t"	// 0-1    n2 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"rjmp .+0"                "\n\t"	// 2    nop nop

			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n1]   , %[lo]"    "\n\t"	// 1    n1   = lo
			"out  %[port] , %[n2]"    "\n\t"	// 1    PORT = n2
			"rjmp .+0"                "\n\t"	// 2    nop nop
			"sbrc %[byte] , 1"        "\n\t"	// 1-2  if(b & 0x02)
			"mov %[n1]   , %[hi]"     "\n\t"	// 0-1    n1 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"rjmp .+0"                "\n\t"	// 2    nop nop

			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n2]   , %[lo]"    "\n\t"	// 1    n2   = lo
			"out  %[port] , %[n1]"    "\n\t"	// 1    PORT = n1
			"rjmp .+0"                "\n\t"	// 2    nop nop
			"sbrc %[byte] , 0"        "\n\t"	// 1-2  if(b & 0x01)
			"mov %[n2]   , %[hi]"     "\n\t"	// 0-1    n2 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"sbiw %[count], 1"        "\n\t"	// 2    i--  (dec. but don't act on zero flag yet)

			"out  %[port] , %[hi]"    "\n\t"	// 1    PORT = hi
			"mov  %[n1]   , %[lo]"    "\n\t"	// 1    n1   = lo
			"out  %[port] , %[n2]"    "\n\t"	// 1    PORT = n2
			"ld   %[byte] , %a[ptr]+" "\n\t"	// 2    b = *ptr++
			"sbrc %[byte] , 7"        "\n\t"	// 1-2  if(b & 0x80)
			"mov %[n1]   , %[hi]"     "\n\t"	// 0-1    n1 = hi
			"out  %[port] , %[lo]"    "\n\t"	// 1    PORT = lo
			"brne headB"              "\n"		// 2    while(i) (zero flag determined above)         
			:
			[count] "+w" (buffer_idx),
			[n1]    "+r" (n1),
			[n2]    "+r" (n2),
			[byte]  "+r" (b)
			:
			[ptr]    "e" (ptr),
			[port]   "I" (_SFR_IO_ADDR(OUT_PORTx)),
			[hi]     "r" (hi),
			[lo]     "r" (lo)
		); // end asm
	}
	else
	{ // end 800 KHz, do 400 KHz

		// Timing is more relaxed; unrolling the inner loop for each bit is
		// not necessary.  Still using the peculiar RJMPs as 2X NOPs, not out
		// of need but just to trim the code size down a little.
		// This 400-KHz-datastream-on-8-MHz-CPU code is not quite identical
		// to the 800-on-16 code later -- the hi/lo timing between WS2811 and
		// WS2812 is not simply a 2:1 scale!

		// 20 inst. clocks per bit: HHHHxxxxxxLLLLLLLLLL
		// ST instructions:         ^   ^     ^          (T=0,4,10)

		next = lo;
		bit  = 8;

		asm volatile(
			"head20:\n\t"         // Clk  Pseudocode    (T =  0)
			"st   %a0, %1\n\t"    // 2    PORT = hi     (T =  2)
			"sbrc %2, 7\n\t"      // 1-2  if(b & 128)
			"mov  %4, %1\n\t"     // 0-1   next = hi    (T =  4)
			"st   %a0, %4\n\t"    // 2    PORT = next   (T =  6)
			"mov  %4, %5\n\t"     // 1    next = lo     (T =  7)
			"dec  %3\n\t"         // 1    bit--         (T =  8)
			"breq nextbyte20\n\t" // 1-2  if(bit == 0)
			"rol  %2\n\t"         // 1    b <<= 1       (T = 10)
			"st   %a0, %5\n\t"    // 2    PORT = lo     (T = 12)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 14)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 16)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 18)
			"rjmp head20\n\t"     // 2    -> head20 (next bit out)
			"nextbyte20:\n\t"     //                    (T = 10)
			"st   %a0, %5\n\t"    // 2    PORT = lo     (T = 12)
			"nop\n\t"             // 1    nop           (T = 13)
			"ldi  %3, 8\n\t"      // 1    bit = 8       (T = 14)
			"ld   %2, %a6+\n\t"   // 2    b = *ptr++    (T = 16)
			"sbiw %7, 1\n\t"      // 2    i--           (T = 18)
			"brne head20\n\t"     // 2    if(i != 0) -> head20 (next byte)
		::
			"e" (port),          // %a0
			"r" (hi),            // %1
			"r" (b),             // %2
			"r" (bit),           // %3
			"r" (next),          // %4
			"r" (lo),            // %5
			"e" (ptr),           // %a6
			"w" (buffer_idx)     // %7
		); // end asm
	}
// 12 MHz(ish) AVR --------------------------------------------------------
#elif (F_CPU >= 11100000UL) && (F_CPU <= 14300000UL)

	if (IS_NEO_KHZ800())
	{
		// In the 12 MHz case, an optimized 800 KHz datastream (no dead time
		// between bytes) requires a PORT-specific loop similar to the 8 MHz
		// code (but a little more relaxed in this case).

		// 15 instruction clocks per bit: HHHHxxxxxxLLLLL
		// OUT instructions:              ^   ^     ^     (T=0,4,10)

		next = lo;
		if (b & 0x80) next = hi;

		// Don't "optimize" the OUT calls into the bitTime subroutine;
		// we're exploiting the RCALL and RET as 3- and 4-cycle NOPs!
		asm volatile(
			"headB:"                   "\n\t"	//        (T =  0)
			"out   %[port], %[hi]"     "\n\t"	//        (T =  1)
			"rcall bitTimeB"           "\n\t"	// Bit 7  (T = 15)
			"out   %[port], %[hi]"     "\n\t"	
			"rcall bitTimeB"           "\n\t"	// Bit 6
			"out   %[port], %[hi]"     "\n\t"	
			"rcall bitTimeB"           "\n\t"	// Bit 5
			"out   %[port], %[hi]"     "\n\t"	
			"rcall bitTimeB"           "\n\t"	// Bit 4
			"out   %[port], %[hi]"     "\n\t"	
			"rcall bitTimeB"           "\n\t"	// Bit 3
			"out   %[port], %[hi]"     "\n\t"	
			"rcall bitTimeB"           "\n\t"	// Bit 2
			"out   %[port], %[hi]"     "\n\t"	
			"rcall bitTimeB"           "\n\t"	// Bit 1
												// Bit 0:
			"out   %[port] , %[hi]"    "\n\t"	// 1    PORT = hi    (T =  1)
			"rjmp  .+0"                "\n\t"	// 2    nop nop      (T =  3)
			"ld    %[byte] , %a[ptr]+" "\n\t"	// 2    b = *ptr++   (T =  5)
			"out   %[port] , %[next]"  "\n\t"	// 1    PORT = next  (T =  6)
			"mov   %[next] , %[lo]"    "\n\t"	// 1    next = lo    (T =  7)
			"sbrc  %[byte] , 7"        "\n\t"	// 1-2  if(b & 0x80) (T =  8)
			"mov   %[next] , %[hi]"    "\n\t"	// 0-1    next = hi  (T =  9)
			"nop"                      "\n\t"	// 1                 (T = 10)
			"out   %[port] , %[lo]"    "\n\t"	// 1    PORT = lo    (T = 11)
			"sbiw  %[count], 1"        "\n\t"	// 2    i--          (T = 13)
			"brne  headB"              "\n\t"	// 2    if(i != 0) -> headD (next byte)
			"rjmp doneB"               "\n\t"	
			"bitTimeB:"                "\n\t"	//      nop nop nop     (T =  4)
			"out   %[port] , %[next]"  "\n\t"	// 1    PORT = next     (T =  5)
			"mov   %[next] , %[lo]"    "\n\t"	// 1    next = lo       (T =  6)
			"rol   %[byte]"            "\n\t"	// 1    b <<= 1         (T =  7)
			"sbrc  %[byte] , 7"        "\n\t"	// 1-2  if(b & 0x80)    (T =  8)
			"mov   %[next] , %[hi]"    "\n\t"	// 0-1   next = hi      (T =  9)
			"nop"                      "\n\t"	// 1                    (T = 10)
			"out   %[port] , %[lo]"    "\n\t"	// 1    PORT = lo       (T = 11)
			"ret"                      "\n\t"	// 4    nop nop nop nop (T = 15)            
			"doneB:"                   "\n"
			:
			[count] "+w" (buffer_idx),
			[next]  "+r" (next),
			[byte]  "+r" (b)
			:
			[ptr]    "e" (ptr),
			[port]   "I" (_SFR_IO_ADDR(OUT_PORTx)),
			[hi]     "r" (hi),
			[lo]     "r" (lo)
		); // end asm
	}
	else
	{ // 400 KHz

		// 30 instruction clocks per bit: HHHHHHxxxxxxxxxLLLLLLLLLLLLLLL
		// ST instructions:               ^     ^        ^    (T=0,6,15)

		next = lo;
		bit  = 8;

		asm volatile(
			"head30:\n\t"         // Clk  Pseudocode    (T =  0)
			"st   %a0, %1\n\t"    // 2    PORT = hi     (T =  2)
			"sbrc %2, 7\n\t"      // 1-2  if(b & 128)
			"mov  %4, %1\n\t"     // 0-1   next = hi    (T =  4)
			"rjmp .+0\n\t"        // 2    nop nop       (T =  6)
			"st   %a0, %4\n\t"    // 2    PORT = next   (T =  8)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 10)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 12)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 14)
			"nop\n\t"             // 1    nop           (T = 15)
			"st   %a0, %5\n\t"    // 2    PORT = lo     (T = 17)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 19)
			"dec  %3\n\t"         // 1    bit--         (T = 20)
			"breq nextbyte30\n\t" // 1-2  if(bit == 0)
			"rol  %2\n\t"         // 1    b <<= 1       (T = 22)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 24)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 26)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 28)
			"rjmp head30\n\t"     // 2    -> head30 (next bit out)
			"nextbyte30:\n\t"     //                    (T = 22)
			"nop\n\t"             // 1    nop           (T = 23)
			"ldi  %3, 8\n\t"      // 1    bit = 8       (T = 24)
			"ld   %2, %a6+\n\t"   // 2    b = *ptr++    (T = 26)
			"sbiw %7, 1\n\t"      // 2    i--           (T = 28)
			"brne head30\n\t"     // 1-2  if(i != 0) -> head30 (next byte)
		::
			"e" (port),          // %a0
			"r" (hi),            // %1
			"r" (b),             // %2
			"r" (bit),           // %3
			"r" (next),          // %4
			"r" (lo),            // %5
			"e" (ptr),           // %a6
			"w" (buffer_idx)     // %7
		); // end asm
	}
// 16 MHz(ish) AVR --------------------------------------------------------
#elif (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)

	if (IS_NEO_KHZ800())
	{

		// WS2811 and WS2812 have different hi/lo duty cycles; this is
		// similar but NOT an exact copy of the prior 400-on-8 code.

		// 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
		// ST instructions:         ^   ^        ^       (T=0,5,13)

		volatile uint8_t next, bit;

		next = lo;
		bit  = 8;

		asm volatile(
			"head20:"                      "\n\t" // Clk  Pseudocode    (T =  0)
				"st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
				"sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
				"mov  %[next], %[hi]"      "\n\t" // 0-1   next = hi    (T =  4)
				"dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
				"st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
				"mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
				"breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
				"rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
				"rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
				"nop"                      "\n\t" // 1    nop           (T = 13)
				"st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
				"nop"                      "\n\t" // 1    nop           (T = 16)
				"rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
				"rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
			"nextbyte20:"                  "\n\t" //                    (T = 10)
				"ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
				"ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
				"st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
				"nop"                      "\n\t" // 1    nop           (T = 16)
				"sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
				"brne head20"              "\n"   // 2    if(i != 0) -> (next byte)
			:
				[port]  "+e" (port),
				[byte]  "+r" (b),
				[bit]   "+r" (bit),
				[next]  "+r" (next),
				[count] "+w" (buffer_idx)
			:
				[ptr]    "e" (ptr),
				[hi]     "r" (hi),
				[lo]     "r" (lo)
		); // end asm

	}
	else
	{ // 400 KHz

		// The 400 KHz clock on 16 MHz MCU is the most 'relaxed' version.

		// 40 inst. clocks per bit: HHHHHHHHxxxxxxxxxxxxLLLLLLLLLLLLLLLLLLLL
		// ST instructions:         ^       ^           ^         (T=0,8,20)

		next = lo;
		bit  = 8;

		asm volatile(
			"head40:\n\t"         // Clk  Pseudocode    (T =  0)
			"st   %a0, %1\n\t"    // 2    PORT = hi     (T =  2)
			"sbrc %2, 7\n\t"      // 1-2  if(b & 128)
			"mov  %4, %1\n\t"     // 0-1   next = hi    (T =  4)
			"rjmp .+0\n\t"        // 2    nop nop       (T =  6)
			"rjmp .+0\n\t"        // 2    nop nop       (T =  8)
			"st   %a0, %4\n\t"    // 2    PORT = next   (T = 10)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 12)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 14)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 16)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 18)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 20)
			"st   %a0, %5\n\t"    // 2    PORT = lo     (T = 22)
			"nop\n\t"             // 1    nop           (T = 23)
			"mov  %4, %5\n\t"     // 1    next = lo     (T = 24)
			"dec  %3\n\t"         // 1    bit--         (T = 25)
			"breq nextbyte40\n\t" // 1-2  if(bit == 0)
			"rol  %2\n\t"         // 1    b <<= 1       (T = 27)
			"nop\n\t"             // 1    nop           (T = 28)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 30)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 32)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 34)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 36)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 38)
			"rjmp head40\n\t"     // 2    -> head40 (next bit out)
			"nextbyte40:\n\t"     //                    (T = 27)
			"ldi  %3, 8\n\t"      // 1    bit = 8       (T = 28)
			"ld   %2, %a6+\n\t"   // 2    b = *ptr++    (T = 30)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 32)
			"st   %a0, %5\n\t"    // 2    PORT = lo     (T = 34)
			"rjmp .+0\n\t"        // 2    nop nop       (T = 36)
			"sbiw %7, 1\n\t"      // 2    i--           (T = 38)
			"brne head40\n\t"     // 1-2  if(i != 0) -> head40 (next byte)
		::
			"e" (port),          // %a0
			"r" (hi),            // %1
			"r" (b),             // %2
			"r" (bit),           // %3
			"r" (next),          // %4
			"r" (lo),            // %5
			"e" (ptr),           // %a6
			"w" (buffer_idx)     // %7
		); // end asm
	}
#else
	#error "CPU SPEED NOT SUPPORTED"
#endif
}

static inputmode_t get_input_mode()
{
	USICR = 0; // turn off USI so it doesn't drive the pins
	JMP_DDRx &= ~JMP_MASK; // pin is input
	JMP_PORTx &= ~JMP_MASK; // turn off internal pull-up resistor
	asm volatile ("rjmp .+0\n\t"); // small delay
	if ((JMP_PINx & JMP_MASK) != 0) {
		// reading logic high even though there's an externally implemented pull-down resistor?
		// jumper is tied to VCC
		return INPUTMODE_SPI;
	}
	JMP_PORTx |= JMP_MASK; // activate internal pull-up resistor
	asm volatile ("rjmp .+0\n\t"); // small delay
	if ((JMP_PINx & JMP_MASK) == 0) {
		// reading logic low even though there's a pull-up resistor?
		// jumper is tied to ground
		// note: the external pull-down resistor must have magnitudes higher resistance than the internal pull-up resistor
		return INPUTMODE_I2C;
	}
	// if it's not the above two, then jumper is floating
	JMP_PORTx &= ~JMP_MASK; // turn off resistor
	return INPUTMODE_UART;
}

// code to support UART input mode

static inline uint8_t reverse_bits(uint8_t x)
{
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x; 
}

static void usi_ovf_vect_uart()
{
	uint8_t tmp = USIDR; // take input
	if (buffer_idx < BUFFER_SIZE) buffer[buffer_idx++] = reverse_bits(tmp);	// increment w/o overflow
	USIDR  = 0;									// make sure we don't accidentally send stuff to neopixel
	TCCR0B = (0<<CS02)|(0<<CS01)|(0<<CS00);		// Stop Timer0.
	USICR  =  0;								// Disable USI.
	GIFR   =  (1<<PCIF);						// Clear pin change interrupt flag.
	GIMSK |=  (1<<PCIE);						// Enable pin change interrupt for PB3:0.
	is_rxing = 0;
}

ISR (
	#if defined(TIM0_OVF_vect)
	TIM0_OVF_vect
	#elif defined(TIMER0_OVF_vect)
	TIMER0_OVF_vect
	#elif defined(TIMER0_OVF0_vect)
	TIMER0_OVF0_vect
	#else
		#error Timer0 Overflow Vector Not Defined
	#endif
)
{
							// Timer0 Overflow interrupt is used to trigger the sampling of signals on the USI ports.
	TCNT0 += TIMER0_SEED;	// Reload the timer,
							// current count is added for timing correction.
}

ISR (PCINT0_vect)
{
	if ((IN_DATA_PINx & IN_DATA_MASK) == 0)
	{
		TCNT0  = INTERRUPT_STARTUP_DELAY + INITIAL_TIMER0_SEED;   // Plant TIMER0 seed to match baudrate (incl interrupt start up time.).
		TCCR0B = (1<<PSR0)|(0<<CS02)|(0<<CS01)|(1<<CS00);         // Reset the prescaler and start Timer0.
		TIFR   = (1<<TOV0);                                       // Clear Timer0 OVF interrupt flag.
		TIMSK |= (1<<TOIE0);                                      // Enable Timer0 OVF interrupt.
		USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
				 (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
				 (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 OVER as USI Clock source.
				 (0<<USITC);
																  // Note that enabling the USI will also disable the pin change interrupt
		USISR  = 0xF0 |                                           // Clear all USI interrupt flags
				 USI_COUNTER_SEED_RECEIVE;                        // Preload the USI counter to generate interrupt
		GIMSK &=  ~(1<<PCIE);                                     // Disable pin change interrupt
		is_rxing = 1;
	}
}

// code to support I2C input mode

//! Functions implemented as macros
#define SET_USI_TO_SEND_ACK()                                                                             \
{                                                                                                         \
	USIDR    =  0;                                              /* Prepare ACK                         */ \
	IN_DATA_DDRx |=  IN_DATA_MASK;                              /* Set SDA as output                   */ \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|  /* Clear all flags, except Start Cond  */ \
				(0x0E<<USICNT0);                                /* set USI counter to shift 1 bit.     */ \
}

#define SET_USI_TO_READ_ACK()                                                                             \
{                                                                                                         \
	IN_DATA_DDRx &= ~IN_DATA_MASK;                              /* Set SDA as intput  */                  \
	USIDR    =  0;                                              /* Prepare ACK        */                  \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|  /* Clear all flags, except Start Cond  */ \
				(0x0E<<USICNT0);                                /* set USI counter to shift 1 bit.     */ \
}

#define SET_USI_TO_TWI_START_CONDITION_MODE()                                                                                   \
{                                                                                                                               \
	USICR = (1<<USISIE)|(0<<USIOIE)|                        /* Enable Start Condition Interrupt. Disable Overflow Interrupt.*/  \
		    (1<<USIWM1)|(0<<USIWM0)|                        /* Set USI in Two-wire mode. No USI Counter overflow hold.      */  \
		    (1<<USICS1)|(0<<USICS0)|(0<<USICLK)|            /* Shift Register Clock Source = External, positive edge        */  \
		    (0<<USITC);                                                                                                         \
	USISR = (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|  /* Clear all flags, except Start Cond                           */  \
		    (0x0<<USICNT0);                                                                                                     \
}

#define SET_USI_TO_SEND_DATA()                                                                               \
{                                                                                                            \
	IN_DATA_DDRx |=  IN_DATA_MASK;                                  /* Set SDA as output                  */ \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      /* Clear all flags, except Start Cond */ \
                (0x0<<USICNT0);                                     /* set USI to shift out 8 bits        */ \
}

#define SET_USI_TO_READ_DATA()                                                                               \
{                                                                                                            \
	IN_DATA_DDRx &= ~IN_DATA_MASK;                                  /* Set SDA as input                   */ \
	USISR    =  (0<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      /* Clear all flags, except Start Cond */ \
                (0x0<<USICNT0);                                     /* set USI to shift in 8 bits         */ \
}

ISR (
	#if defined(USI_START_vect)
	USI_START_vect
	#elif defined(USI_STRT_vect)
	USI_STRT_vect
	#elif defined(USI_STR_vect)
	USI_STR_vect
	#else
		#error USI Start Condition Vector Not Defined
	#endif
	, ISR_NAKED)
{
	IN_CLK_DDRx  |=  IN_CLK_MASK; // clock stretch
	IN_CLK_PORTx &= ~IN_CLK_MASK; // by driving low
	IN_DATA_DDRx  &= ~IN_DATA_MASK; // Set SDA as input

	asm volatile ("\n\t"
		"push r1\n\t"
		"push r0\n\t"
		"in r0, %0\n\t" // save SREG
		"push r0\n\t"
		"eor r1, r1\n\t" // make 0
		"push r2\n\t" "push r3\n\t" "push r4\n\t" "push r5\n\t" "push r6\n\t"
		"push r7\n\t" "push r8\n\t" "push r9\n\t" "push r10\n\t" "push r11\n\t"
		"push r12\n\t" "push r13\n\t" "push r14\n\t" "push r15\n\t" "push r16\n\t"
		"push r17\n\t" "push r18\n\t" "push r19\n\t" "push r20\n\t" "push r21\n\t"
		"push r22\n\t" "push r23\n\t" "push r24\n\t" "push r25\n\t" "push r26\n\t"
		"push r27\n\t" "push r28\n\t" "push r29\n\t" "push r30\n\t" "push r31\n\t"
		::
		"I" (_SFR_IO_ADDR(SREG))
	);

	// Set default starting conditions for new TWI package
	USI_TWI_Overflow_State = USI_SLAVE_CHECK_ADDRESS;

	USICR =		(1<<USISIE)|(1<<USIOIE)|									// Enable Overflow and Start Condition Interrupt. (Keep StartCondInt to detect RESTART)
				(1<<USIWM1)|(1<<USIWM0)|									// Set USI in Two-wire mode.
				(1<<USICS1)|(0<<USICS0)|(0<<USICLK)|						// Shift Register Clock Source = External, positive edge
				(0<<USITC);
	USISR =		(1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|				// Clear flags
				(0x0<<USICNT0);
	is_rxing = 1;

	asm volatile ("\n\t"
		"pop r31\n\t" "pop r30\n\t" "pop r29\n\t" "pop r28\n\t" "pop r27\n\t"
		"pop r26\n\t" "pop r25\n\t" "pop r24\n\t" "pop r23\n\t" "pop r22\n\t"
		"pop r21\n\t" "pop r20\n\t" "pop r19\n\t" "pop r18\n\t" "pop r17\n\t"
		"pop r16\n\t" "pop r15\n\t" "pop r14\n\t" "pop r13\n\t" "pop r12\n\t"
		"pop r11\n\t" "pop r10\n\t" "pop r9\n\t" "pop r8\n\t" "pop r7\n\t"
		"pop r6\n\t" "pop r5\n\t" "pop r4\n\t" "pop r3\n\t" "pop r2\n\t"
		"pop r0\n\t"
		"out %0, r0\n\t" // restore SREG
		"pop r0\n\t"
		"pop r1\n\t"
		::
		"I" (_SFR_IO_ADDR(SREG))
	);

	IN_CLK_DDRx &= ~IN_CLK_MASK; // release clock
	asm volatile ("reti\n\t");
}

void usi_ovf_vect_i2c()
{
	unsigned char tmpUSIDR;

	is_rxing = 1;

	switch (USI_TWI_Overflow_State)
	{
		// ---------- Address mode ----------
		// Check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK, else reset USI.
		case USI_SLAVE_CHECK_ADDRESS:
			tmpUSIDR = USIDR;
			if (tmpUSIDR == 0 ||
				( tmpUSIDR>>1 ) == (I2C_ADDR>>1))
			{
				if ( tmpUSIDR & TW_READ )
					USI_TWI_Overflow_State = USI_SLAVE_SEND_DATA;
				else
					USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
				SET_USI_TO_SEND_ACK();
			}
			else
			{
				SET_USI_TO_TWI_START_CONDITION_MODE();
			}
		break;

		// ----- Master write data mode ------
		// Check reply and goto USI_SLAVE_SEND_DATA if OK, else reset USI.
		case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
			if ( USIDR ) // If NACK, the master does not want more data.
			{
				SET_USI_TO_TWI_START_CONDITION_MODE();
				return;
			}
		// From here we just drop straight into USI_SLAVE_SEND_DATA if the master sent an ACK

		// Copy data from buffer to USIDR and set USI to shift byte. Next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
		case USI_SLAVE_SEND_DATA:
			// data read is unsupported!
			USIDR = 0;
			USI_TWI_Overflow_State = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
			SET_USI_TO_SEND_DATA();
		break;

		// Set USI to sample reply from master. Next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
		case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
			USI_TWI_Overflow_State = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
			SET_USI_TO_READ_ACK();
		break;

		// ----- Master read data mode ------
		// Set USI to sample data from master. Next USI_SLAVE_GET_DATA_AND_SEND_ACK.
		case USI_SLAVE_REQUEST_DATA:
			USI_TWI_Overflow_State = USI_SLAVE_GET_DATA_AND_SEND_ACK;
			SET_USI_TO_READ_DATA();
		break;

		// Copy data from USIDR and send ACK. Next USI_SLAVE_REQUEST_DATA
		case USI_SLAVE_GET_DATA_AND_SEND_ACK:
			// Put data into Buffer
			tmpUSIDR = USIDR;             // Not necessary, but prevents warnings

			if (buffer_idx < BUFFER_SIZE) buffer[buffer_idx++] = tmpUSIDR; // take input w/o overflow

			USI_TWI_Overflow_State = USI_SLAVE_REQUEST_DATA;
			SET_USI_TO_SEND_ACK();
		break;
	}

	USISR |= _BV(USIOIF); // clear the flag
}

// overflow vector needs to make the decision about which mode

ISR (
	#if defined(USI_OVF_vect)
	USI_OVF_vect
	#elif defined(USI_OVERFLOW_vect)
	USI_OVERFLOW_vect
	#else
		#error USI overflow Vector Not Defined
	#endif
	, ISR_NAKED)
{
	IN_CLK_DDRx  |=  IN_CLK_MASK; // clock stretch
	IN_CLK_PORTx &= ~IN_CLK_MASK; // by driving low

	asm volatile ("\n\t"
		"push r1\n\t"
		"push r0\n\t"
		"in r0, %0\n\t" // save SREG
		"push r0\n\t"
		"eor r1, r1\n\t" // make 0
		"push r2\n\t" "push r3\n\t" "push r4\n\t" "push r5\n\t" "push r6\n\t"
		"push r7\n\t" "push r8\n\t" "push r9\n\t" "push r10\n\t" "push r11\n\t"
		"push r12\n\t" "push r13\n\t" "push r14\n\t" "push r15\n\t" "push r16\n\t"
		"push r17\n\t" "push r18\n\t" "push r19\n\t" "push r20\n\t" "push r21\n\t"
		"push r22\n\t" "push r23\n\t" "push r24\n\t" "push r25\n\t" "push r26\n\t"
		"push r27\n\t" "push r28\n\t" "push r29\n\t" "push r30\n\t" "push r31\n\t"
		::
		"I" (_SFR_IO_ADDR(SREG))
	);

	if (input_mode == INPUTMODE_UART) usi_ovf_vect_uart();
	else if (input_mode == INPUTMODE_I2C) usi_ovf_vect_i2c();

	asm volatile ("\n\t"
		"pop r31\n\t" "pop r30\n\t" "pop r29\n\t" "pop r28\n\t" "pop r27\n\t"
		"pop r26\n\t" "pop r25\n\t" "pop r24\n\t" "pop r23\n\t" "pop r22\n\t"
		"pop r21\n\t" "pop r20\n\t" "pop r19\n\t" "pop r18\n\t" "pop r17\n\t"
		"pop r16\n\t" "pop r15\n\t" "pop r14\n\t" "pop r13\n\t" "pop r12\n\t"
		"pop r11\n\t" "pop r10\n\t" "pop r9\n\t" "pop r8\n\t" "pop r7\n\t"
		"pop r6\n\t" "pop r5\n\t" "pop r4\n\t" "pop r3\n\t" "pop r2\n\t"
		"pop r0\n\t"
		"out %0, r0\n\t" // restore SREG
		"pop r0\n\t"
		"pop r1\n\t"
		::
		"I" (_SFR_IO_ADDR(SREG))
	);

	IN_CLK_DDRx &= ~IN_CLK_MASK; // release clock
	asm volatile ("reti\n\t");
}