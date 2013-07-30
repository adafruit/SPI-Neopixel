// Experiemtnal alternate ATtiny85 NeoPixel bridge code based on
// spineopixel.c and caffeine.  No interrupts (all polling-based),
// Handles 4 MHz SPI on 16 MHz MCU.  No global state machine stuff,
// max space for MOAR PIXELS.

#if !defined(F_CPU)
 #error F_CPU not defined
#endif
#if !defined(BAUDRATE)
 #error BAUDRATE not defined
#endif
#if !defined(I2C_ADDR) || (I2C_ADDR < 1) || (I2C_ADDR > 127)
 #error I2C_ADDR not defined, or invalid range (must be 1-127)
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "reverse_bits.h"

// Clock/data/latch input pins
#define IN_CLK_PORTx   PORTB
#define IN_CLK_DDRx    DDRB
#define IN_CLK_PINx    PINB
#define IN_CLK_BIT     2
#define IN_CLK_MASK   (1 << IN_CLK_BIT)
#define IN_DATA_PORTx  PORTB
#define IN_DATA_DDRx   DDRB
#define IN_DATA_PINx   PINB
#define IN_DATA_BIT    0
#define IN_DATA_MASK  (1 << IN_DATA_BIT)
#define IN_LATCH_PORTx PORTB
#define IN_LATCH_DDRx  DDRB
#define IN_LATCH_PINx  PINB
#define IN_LATCH_BIT   3
#define IN_LATCH_MASK (1 << IN_LATCH_BIT)
// NeoPixel output pin
#define OUT_PORTx      PORTB
#define OUT_DDRx       DDRB
#define OUT_PINx       PINB
#define OUT_BIT        4
#define OUT_MASK      (1 << OUT_BIT)
// Mode select jumper: ->VCC = SPI, ->1M->GND = I2C, N/C = UART
#define JMP_PORTx      PORTB
#define JMP_DDRx       DDRB
#define JMP_PINx       PINB
#define JMP_MASK      (1 << 1)

// Drive latch low to indicate busy state, release for idle state
#define LATCH_BUSY                  \
  IN_LATCH_DDRx  |=  IN_LATCH_MASK; \
  IN_LATCH_PORTx &= ~IN_LATCH_MASK; // Output mode, set low
#define LATCH_IDLE                  \
  IN_LATCH_DDRx  &= ~IN_LATCH_MASK; \
  IN_LATCH_PORTx |=  IN_LATCH_MASK; // Input mode w/internal pullup

static void
  spi_handler(void) ,
  i2c_handler(void) ,
  uart_handler(void),
  send_to_neopixel(void);

#define MAX_LEDS         150
#define BUFFERSIZE      (MAX_LEDS * 3)
static uint8_t           buffer[BUFFERSIZE];
static volatile uint16_t byteCount;

int main(void) {

	LATCH_BUSY
	cli();                  // No interrupts EVAR

	OUT_DDRx  |=  OUT_MASK; // NeoPixel output
	OUT_PORTx &= ~OUT_MASK; // Default low

	// Set all pixels to initial 'off' state
	byteCount = sizeof(buffer);
	memset(buffer, 0, byteCount);
	send_to_neopixel();

	// For testing, call a handler directly:
	// spi_handler();
	// i2c_handler();
	// uart_handler();

	// Determine input mode from jumper, call specific handler.
	// This allows for tighter control loops for each peripheral
	// type rather than a unified state thing (and associated
	// variables).  Code space is more prevalent than RAM.

	USICR      = 0;            // USI off
	JMP_DDRx  &= ~JMP_MASK;    // Jumper pin = input
	JMP_PORTx &= ~JMP_MASK;    // No pullup
	asm volatile ("rjmp .+0"); // Small delay
	// If reading logic high even though there's an external pull-down
	// resistor, pin is tied to VCC.  Jump to SPI-specific code.
	if(JMP_PINx & JMP_MASK) spi_handler();

	JMP_PORTx |= JMP_MASK;     // Activate pullup
	asm volatile ("rjmp .+0"); // Small delay
	// If reading logic low even though there's a pull-up resistor,
	// pin is tied to ground.  Jump to I2C-specific code.
	// Note: external pull-down resistor MUST have magnitudes
	// higher resistance than the internal pull-up resistor.
	if(!(JMP_PINx & JMP_MASK)) i2c_handler();

	// Pin is floating.  Jump to UART-specific code.
	JMP_PORTx &= ~JMP_MASK; // Disable pullup
	uart_handler();

	return 0;
}

static void inline __attribute__ ((always_inline)) spi_handler(void) {

	IN_CLK_DDRx   &= ~IN_CLK_MASK;  // Clock, data inputs
	IN_DATA_DDRx  &= ~IN_DATA_MASK;
	IN_CLK_PORTx  &= ~IN_CLK_MASK;  // No pullups on clock, data
	IN_DATA_PORTx &= ~IN_DATA_MASK;

	USICR = _BV(USIWM0) | _BV(USICS1); // 3 wire, ext clock, pos edge

	for(;;) {                                    // Forever
	  LATCH_IDLE
	  USISR = _BV(USIOIF);                       // Reset frame, important

	  while(IN_LATCH_PINx & IN_LATCH_MASK);      // Latch is high, wait
	  do {                                       // else latch low...
	    if(USISR & _BV(USIOIF)) {                // New data ready?
	      USISR = _BV(USIOIF);                   // Clear flag & counter
	      if(byteCount < sizeof(buffer))         // If room avail
	        buffer[byteCount++] = USIDR;         //  store next byte
	    }
	  } while(!(IN_LATCH_PINx & IN_LATCH_MASK)); // While low

	  if(byteCount) send_to_neopixel();
	}
}

static void inline __attribute__ ((always_inline)) i2c_ack(void) {
	IN_DATA_DDRx |=  IN_DATA_MASK; // SDA output (pulls SDA low = ACK)
	// Clear overflow bit (also releases clock stretch),
	// set counter to shift one bit out.
	USISR         = _BV(USIOIF) | (0xE << USICNT0);
	while(!(USISR & _BV(USIOIF))); // Wait for bit
	IN_DATA_DDRx &= ~IN_DATA_MASK; // Back to input
	USISR         = _BV(USIOIF);   // Reset counter & overflow flag again
}

#define ADDR_8BIT (I2C_ADDR << 1) // Actual 8-bit I2C addr on the wire

// Important change: I2C stop condition does NOT latch.  If using an
// Arduino as the I2C master, that platform's Wire library only supports
// up to 32 bytes per transfer, while we can receive up to 450.  Using
// the latch pin allows smaller packets to be coalesced.  It's not "pure"
// I2C, but a necessary evil.

static void inline __attribute__ ((always_inline)) i2c_handler(void) {

	uint8_t x;

	IN_DATA_DDRx  &= ~IN_DATA_MASK; // SDA input
	IN_CLK_DDRx   &= ~IN_CLK_MASK;  // SCL input
	IN_DATA_PORTx &= ~IN_CLK_MASK;  // Pullups disabled in TWI mode,
	IN_CLK_PORTx  &= ~IN_DATA_MASK; // external pullups required

	// TWI mode, external clock, negative edge
	USICR = _BV(USIWM1) | _BV(USIWM0) | _BV(USICS1) | _BV(USICS0);
	// Setting USIWM0 holds SCL low when a counter overflow occurs --
	// basically, automatic clock-stretching.  It's released when
	// the overflow flag is reset.

	for(;;) {                                       // Forever
	  LATCH_IDLE

	  while(IN_LATCH_PINx & IN_LATCH_MASK);         // Latch is high, wait
	  do {                                          // else latch low...

	    USISR = 0xF0;                               // Reset flags, counter

	    while((!(USISR         & _BV(USISIF))) &&   // Wait for start cond.
	          (!(IN_LATCH_PINx & IN_LATCH_MASK)));  // or latch high

	    if(IN_DATA_PINx & IN_DATA_MASK) break;      // Latch high

#if(0)
	    while( (IN_CLK_PINx   & IN_CLK_MASK  )  &&  // Ensure start
	         (!(IN_DATA_PINx  & IN_DATA_MASK )) &&  // condition completes
	         (!(IN_LATCH_PINx & IN_LATCH_MASK)));
#else
	    // Condition is simplified when they're on the same PINx register:
	    while((IN_CLK_PINx &
	      (IN_CLK_MASK | IN_DATA_MASK | IN_LATCH_MASK)) == IN_CLK_MASK);
#endif

	    if(IN_LATCH_PINx & IN_LATCH_MASK) break;    // Latch high
	    if(IN_DATA_PINx  & IN_DATA_MASK ) continue; // Stop condition

	    // Start condition detected

	    USISR = _BV(USIOIF); // Reset overflow flag, clear counter

	    while((!(USISR & (_BV(USIOIF) |             // Wait for data
                              _BV(USIPF )))) &&         // or stop condition
	          (!(IN_LATCH_PINx & IN_LATCH_MASK)));  // or latch high

	    if(IN_LATCH_PINx & IN_LATCH_MASK) break;    // Latch high
	    if(USISR         & _BV(USIPF)   ) continue; // Stop condition

	    if(USIDR != ADDR_8BIT) continue;            // Not my address
	    i2c_ack();                                  // It's a me!

	    while((!(USISR         & _BV(USIPF)   )) && // Wait for stop cond
	          (!(IN_LATCH_PINx & IN_LATCH_MASK))) { // or latch high
	      if(USISR & _BV(USIOIF)) {                 // New data ready?
	        x            = USIDR;                   // Read byte in
	        i2c_ack();                              // Thanks
	        if(byteCount < sizeof(buffer))          // If room avail
	          buffer[byteCount++] = x;              //  store next byte
	      }
	    }

	  } while(!(IN_LATCH_PINx & IN_LATCH_MASK)); // While latch low

	  if(byteCount) send_to_neopixel();
	}
}

// Determine valid Timer0 prescale setting for F_CPU/BAUDRATE combo
#if   ((F_CPU      / BAUDRATE) < 256)
 #define PRESCALE       1
 #define PRESCALE_MASK _BV(CS00)
#elif ((F_CPU / 8  / BAUDRATE) < 256)
 #define PRESCALE       8
 #define PRESCALE_MASK _BV(CS01)
#elif ((F_CPU / 64 / BAUDRATE) < 256)
 #define PRESCALE       64
 #define PRESCALE_MASK (_BV(CS01) | _BV(CS00))
#else
 // Could escalate with more prescales if needed, but for now...
 #error F_CPU, BAUDRATE combo incompatible with supported Timer0 prescales
#endif
#define BITTIME ((F_CPU / PRESCALE) / BAUDRATE)

static void inline __attribute__ ((always_inline)) uart_handler(void) {

	IN_DATA_DDRx  &= ~IN_DATA_MASK; // Data input (clock is unused)
	IN_DATA_PORTx &= ~IN_DATA_MASK; // No pullup

	// WGM Mode 7: Fast PWM, OC0A/B disconnected
	TCCR0A = _BV(WGM01) | _BV(WGM00);
	TCCR0B = _BV(WGM02) | PRESCALE_MASK;
	OCR0A  = BITTIME;

	// Three-wire mode w/Timer0 compare match = ersatz UART
	USICR  = _BV(USIWM0) | _BV(USICS0);

	for(;;) {                                    // Forever
	  LATCH_IDLE

	  while(IN_LATCH_PINx & IN_LATCH_MASK);      // Latch is high, wait
	  do {                                       // else latch low...

	    while((IN_DATA_PINx  & IN_DATA_MASK ) && // Wait for start bit
	         !(IN_LATCH_PINx & IN_LATCH_MASK));  // or latch high

	    if(IN_LATCH_PINx & IN_LATCH_MASK) break; // Latch went high

	    // Now at beginning of start bit.  Need to start sampling
	    // at middle of bit...set TCNT0 halfway to overflow and
	    // USI counter to read 1 bit.
	    TCNT0 = BITTIME / 2;
	    USISR = _BV(USIOIF) | (0xF << USICNT0);
	    while(!(USISR & _BV(USIOIF)));           // Wait for bit

	    // Now at middle of bit start bit.
	    // Reset overflow and set counter for next 8 bits.
	    USISR = _BV(USIOIF) | (0x8 << USICNT0);
	    while(!(USISR & _BV(USIOIF)));           // Wait for byte
	    if(byteCount < sizeof(buffer))           // Store if not full
	      buffer[byteCount++] = pgm_read_byte(&reverse_bits[USIDR]);

	    // Now in middle of last bit.  Idle for one more (stop bit).
	    USISR = _BV(USIOIF) | (0xF << USICNT0);
	    while(!(USISR & _BV(USIOIF)));

	    // Now in middle of stop bit (logic high).
	    // Next iteration resumes watching for falling edge.

	  } while(!(IN_LATCH_PINx & IN_LATCH_MASK)); // While latch low

	  if(byteCount) send_to_neopixel();
	}
}

// ------------------------------------------------------------------------

// Code below is adapted from Adafruit_NeoPixel.cpp, with minor tweaks
// for this project (and comments stripped for brevity).  The full PORT
// writes with hi/lo/next vars is an artifact of that library's runtime
// pin selection...while fixed bit set/clear instructions would suffice
// for this project, the timing is extremely critical and it was easier
// to use the more flexible code than to write all new stuff here just
// to maybe save a handful of bytes.

// Global variable byteCount is altered by this function.  It will
// always equal zero on return, indicating an empty buffer.

static void __attribute__ ((noinline)) send_to_neopixel(void) {

	LATCH_BUSY

	volatile uint8_t
	  *ptr =  buffer,                // Pointer to next byte
	   b   = *ptr++;                 // Current byte value
	uint8_t
	   hi  =  OUT_PORTx |  OUT_MASK, // PORT w/output bit set high
	   lo  =  OUT_PORTx & ~OUT_MASK; // PORT w/output bit set low

// 8 MHz(ish) AVR ---------------------------------------------------------
#if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)

	volatile uint8_t n2 = 0, n1 = lo;
	if(b & 0x80) n1 = hi;

	asm volatile(
	 "headB:"                   "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n2]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n1]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "sbrc %[byte] , 6"        "\n\t"
	   "mov %[n2]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n1]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n2]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "sbrc %[byte] , 5"        "\n\t"
	   "mov %[n1]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n2]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n1]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "sbrc %[byte] , 4"        "\n\t"
	   "mov %[n2]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n1]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n2]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "sbrc %[byte] , 3"        "\n\t"
	   "mov %[n1]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n2]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n1]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "sbrc %[byte] , 2"        "\n\t"
	   "mov %[n2]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n1]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n2]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "sbrc %[byte] , 1"        "\n\t"
	   "mov %[n1]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n2]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n1]"    "\n\t"
	  "rjmp .+0"                "\n\t"
	  "sbrc %[byte] , 0"        "\n\t"
	   "mov %[n2]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "sbiw %[count], 1"        "\n\t"
	  "out  %[port] , %[hi]"    "\n\t"
	  "mov  %[n1]   , %[lo]"    "\n\t"
	  "out  %[port] , %[n2]"    "\n\t"
	  "ld   %[byte] , %a[ptr]+" "\n\t"
	  "sbrc %[byte] , 7"        "\n\t"
	   "mov %[n1]   , %[hi]"    "\n\t"
	  "out  %[port] , %[lo]"    "\n\t"
	  "brne headB"              "\n"
	  : [count] "+w" (byteCount),
	    [n1]    "+r" (n1),
	    [n2]    "+r" (n2),
	    [byte]  "+r" (b)
	  :
	    [ptr]    "e" (ptr),
	    [port]   "I" (_SFR_IO_ADDR(OUT_PORTx)),
	    [hi]     "r" (hi),
	    [lo]     "r" (lo));

// 12 MHz(ish) AVR --------------------------------------------------------
#elif (F_CPU >= 11100000UL) && (F_CPU <= 14300000UL)

	volatile uint8_t next = lo;
	if(b & 0x80) next = hi;

	asm volatile(
	 "headB:"                    "\n\t"
	  "out   %[port], %[hi]"     "\n\t"
	  "rcall bitTimeB"           "\n\t"
	  "out   %[port], %[hi]"     "\n\t"
	  "rcall bitTimeB"           "\n\t"
	  "out   %[port], %[hi]"     "\n\t"
	  "rcall bitTimeB"           "\n\t"
	  "out   %[port], %[hi]"     "\n\t"
	  "rcall bitTimeB"           "\n\t"
	  "out   %[port], %[hi]"     "\n\t"
	  "rcall bitTimeB"           "\n\t"
	  "out   %[port], %[hi]"     "\n\t"
	  "rcall bitTimeB"           "\n\t"
	  "out   %[port], %[hi]"     "\n\t"
	  "rcall bitTimeB"           "\n\t"
	  "out   %[port] , %[hi]"    "\n\t"
	  "rjmp  .+0"                "\n\t"
	  "ld    %[byte] , %a[ptr]+" "\n\t"
	  "out   %[port] , %[next]"  "\n\t"
	  "mov   %[next] , %[lo]"    "\n\t"
	  "sbrc  %[byte] , 7"        "\n\t"
	   "mov  %[next] , %[hi]"    "\n\t"
	  "nop"                      "\n\t"
	  "out   %[port] , %[lo]"    "\n\t"
	  "sbiw  %[count], 1"        "\n\t"
	  "brne  headB"              "\n\t"
	  "rjmp doneB"               "\n\t"
	 "bitTimeB:"                 "\n\t"
	  "out   %[port] , %[next]"  "\n\t"
	  "mov   %[next] , %[lo]"    "\n\t"
	  "rol   %[byte]"            "\n\t"
	  "sbrc  %[byte] , 7"        "\n\t"
	   "mov  %[next] , %[hi]"    "\n\t"
	  "nop"                      "\n\t"
	  "out   %[port] , %[lo]"    "\n\t"
	  "ret"                      "\n\t"
	 "doneB:"                    "\n"
	  : [count] "+w" (byteCount),
	    [next]  "+r" (next),
	    [byte]  "+r" (b)
	  : [ptr]    "e" (ptr),
	    [port]   "I" (_SFR_IO_ADDR(OUT_PORTx)),
	    [hi]     "r" (hi),
	    [lo]     "r" (lo));

// 16 MHz(ish) AVR --------------------------------------------------------
#elif (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)

	volatile uint8_t next = lo, bit = 8;

	asm volatile(
	 "head20:"                  "\n\t"
	  "out  %[port],  %[hi]"    "\n\t"
	  "sbrc %[byte],  7"        "\n\t"
	   "mov  %[next], %[hi]"    "\n\t"
	  "dec  %[bit]"             "\n\t"
	  "out  %[port],  %[next]"  "\n\t"
	  "mov  %[next],  %[lo]"    "\n\t"
	  "breq nextbyte20"         "\n\t"
	  "rol  %[byte]"            "\n\t"
	  "rjmp .+0"                "\n\t"
	  "nop"                     "\n\t"
	  "out  %[port],  %[lo]"    "\n\t"
	  "nop"                     "\n\t"
	  "rjmp .+0"                "\n\t"
	  "rjmp head20"             "\n\t"
	 "nextbyte20:"              "\n\t"
	  "ldi  %[bit] ,  8"        "\n\t"
	  "ld   %[byte],  %a[ptr]+" "\n\t"
	  "out  %[port],  %[lo]"    "\n\t"
	  "nop"                     "\n\t"
	  "sbiw %[count], 1"        "\n\t"
	   "brne head20"            "\n"
	  : [count] "+w" (byteCount),
	    [next]  "+r" (next),
	    [byte]  "+r" (b),
	    [bit]   "+r" (bit)
	  : [ptr]    "e" (ptr),
	    [port]   "I" (_SFR_IO_ADDR(OUT_PORTx)),
	    [hi]     "r" (hi),
	    [lo]     "r" (lo));

#else
 #error "CPU SPEED NOT SUPPORTED"
#endif

	// ~ 50 uS delay
	volatile uint16_t x = (F_CPU / 1000000L * 50L) / 6;
	while(x--); // 6 cycles per iteration
}

