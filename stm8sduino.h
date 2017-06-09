//Arduino port for STM8S family
//software environment: IAR EMSTM8 + STM8S.h from ST Standard Peripheral Library
//
//version v0.15 6/9/2017
//rewrote to enhance portability
//
//version v0.13 6/3/2017
//added I2C support
//
//version v0.12a 6/3/2017
//added support for UART1 as well
//added support for user-configurability of hardware modules -> to help manage code size on smaller chips
//
//version v0.12 @ 6/2/2017
//added serial support off uart2: serial_begin(), serial_println(), serial_print(), serial_write(), serial_read() and serila_available()
//added hardware spi support: SPIshiftOut(), and SPIshiftIn()
//
//version: v0.11a @ 5/29/2017
//added support for attachInterrupt() and detachInterrupt()
//
//version: v0.11 @ 5/28/2017
//added support for analogWrite() and analogRead()
//added support for tone() and noTone()
//added support for pulseIn()
//
//version: v0.10 @ 5/26/2017
//initiaial release, support for STM8S chips
//
//Supported functions:
//GPIO: pinMode(), digitalWrite(), digitalRead()
//Time: millis(), micros(), delay(), delayMicroseconds()
//Math: min(), max(), abs(), constrain(), map(), pow(), sqrt()
//Trigonometry: sin(), cos(), tan()
//Characters: isAlphaNumeric(), isAlpha(), isAscii(), isWhitespace(), isControl(), isDigit(), isGraph(), isLowerCase(), isPrintable, isPunct(), isSpace(), isUpperCase(), isHexadecimalDigit()
//Random Numbers: randomSeed(), random(max). random(min, max) ported to random2(min, max)
//Bits and Bytes: lowByte(), highByte(), bitRead(), bitWrite(), bitSet(), bitClear(), bit()
//Analog IO: analogRead(), analogWrite(), analogReference()
//Advanced IO: tone(), noTone(), shiftOut(), shiftIn(), pulseIn()
//External Interrupts: attachInterrupt(), detachInterrupt()
//Serial: Serial_begin(), Serial_write(), Serial_read(), Serial_print(), Serial_println(), Serial_available()
//
//STM8Sduino extensions:
//DAC: DAC1Write(), DAC2Write()
//SPI: SPIshiftOut(), SPIshiftIn()
//I2C: I2C_start(), I2C_writeaddr(), I2C_write(), I2C_read(), I2C_stop()
//Serial1: Serial1_begin(), Serial1_write(), Serial1_read(), Serial1_print(), Serial1_println(), Serial1_available()
//Serial2: Serial2_begin(), Serial2_write(), Serial2_read(), Serial2_print(), Serial2_println(), Serial2_available()

#ifndef _STM8Sduino_H
#define _STM8Sduino_H

#include <stdlib.h>							//we use rand()
#include <ctype.h>							//we use char-functions
#include "stm8s.h"							//we use spl's register definitions

//STM8Sduino module configuration
//comment out the follow macros to disable a hardware functionality
//default: use all
//Peripherals always used: GPIO, TIM4
#define USE_PWM1							//comment out if not used - pwm using tim1
#define USE_PWM2							//comment out if not used - pwm using tim2
#define USE_PWM3							//comment out if not used - pwm using tim3
#define USE_EXTI							//comment out if not used
#define USE_ADC1							//comment out if not used
//#define USE_ADC2							//comment out if not used - not present on all chips
#define USE_DAC								//comment out if not used
#define USE_SPI								//comment out if not used
#define USE_SPI								//comment out if not used
#define USE_I2C								//comment out if not used
#define USE_UART1							//comment out if not used - present on 003, 103, 207, 208
#define USE_UART2							//comment out if not used - present on 005, 105
//end STM8Sduino module configuration

//STM8Sduino hardware configuration
#define PWMOUT_BITs		12					//in bits. PWM output / analogWrite() resolution. 8-14 suggested -> default = 12bits
#define PWMOUT_PS		1					//3-bit PMWOUT prescaler. 0x00->1:1, ..., 0x07->128:1, for timer1/2/3
#define ADC1_PS			2					//3-bit ADC prescaler. 0x00->2:1, ...0x07 = 18:1
#define TIM4_PS			7					//tim4 (time base for micros() and millis()) 3-bit prescaler, valid values: [0..7]. 2^TIM4_PS, 128x max
#define SPI_PS			7					//3-bit SPI prescaler. [0..7]: 0->2:1, 7->256:1
//#define I2C_FASTMODE						//uncomment for standard mode (100Khz).
#define F_TONE			1000				//beep frequency, in Hz. 500 - 8000. 
//end STM8Sduino hardware configuration

//global defines
//gpio enums - needs to match GPIO_PinDef[]
typedef enum {
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, //PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,			//GPIOA pin defs
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, //PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,			//GPIOB pin defs
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, //PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,			//GPIOC pin defs
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, //PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,			//GPIOD pin defs
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, //PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,			//GPIOE pin defs
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, //PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,			//GPIOF pin defs
#if defined(GPIOG)
	PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, //PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,			//GPIOG pin defs
#endif
#if defined(GPIOH)
	PH0, PH1, PH2, PH3, PH4, PH5, PH6, PH7, //PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15,			//GPIOH pin defs
#endif
#if defined(GPIOI)
	PI0, PI1, PI2, PI3, PI4, PI5, PI6, PI7, //PI8, PI9, PI10, PI11, PI12, PI13, PI14, PI15,			//GPIOI pin defs
#endif
	PINMAX,																							//max pin. for error checking
} PIN_TypeDef;

//analog input enum for analogRead() - not all channells are present on all chips/packages
typedef enum {
	A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15,
	//AVREFINT,								//A16=Vrefint. nominal 1.20v
	//ATEMP,								//A17=temperature sensor
	AINMAX,									//for error checking
} AIN_TypeDef;

//port/gpio oriented macros
#define GIO_SET(port, pins)					port->ODR |= (pins)				//set bits on port
#define GIO_CLR(port, pins)					port->ODR &=~(pins)				//clear bits on port
#define GIO_FLP(port, pins)					port->ODR ^= (pins)				//flip bits on port
#define GIO_GET(port, pins)					((port->IDR) & (pins))			//return bits on port
//set a pin to output/input
#define GIO_OUTPP(port, pins)				do {port->DDR |= (pins); port->CR1 |= (pins); port->CR2 &=~(pins);} while (0)	//push-pull mode (CR1 set, CR2 cleared)	//IO_OUTPP(GPIOx, GPIO_Pins).
#define GIO_OUTOD(port, pins)				do {port->DDR |= (pins); port->CR1 &=~(pins); port->CR2 &=~(pins);} while (0)	//open drain mode (cr1 + cr2 cleared)	//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_OD_LOW_FAST)
#define GIO_OUT(port, pins)					GIO_OUTPP(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_OUT_PP_LOW_FAST)
#define GIO_INFL(port, pins)				do {port->DDR &=~(pins); port->CR1 &=~(pins); port->CR2 &=~(pins);} while (0)	//floating input, no interrupt			//IO_INFL(GPIOx, GPIO_Pins)
#define GIO_INPU(port, pins)				do {port->DDR &=~(pins); port->CR1 |= (pins); port->CR2 &=~(pins);} while (0)	//pull-up, no interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define GIO_INFLINT(port, pins)				do {port->DDR &=~(pins); port->CR1 &=~(pins); port->CR2 |= (pins);} while (0)	//floating input, with interrupt			//IO_INFL(GPIOx, GPIO_Pins)
#define GIO_INPUINT(port, pins)				do {port->DDR &=~(pins); port->CR1 |= (pins); port->CR2 |= (pins);} while (0)	//pull-up, with interrupt					//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_PU_NO_IT)
#define GIO_IN(port, pins)					GIO_INFL(port, pins)					//IO_IN(port, pins)				//_GPIO_Init(GPIOx, GPIO_Pins, GPIO_MODE_IN_FL_NO_IT)

//extension of pinMode, starting with 4
#define OUTPUT_OD				(INPUT_PULLUP+1)

#define NOP()         			nop()						//__no_operation()			//asm("nop")            		//nop
#define NOP2()					{NOP(); NOP();}
#define NOP4()					{NOP2(); NOP2();}
#define NOP8()					{NOP4(); NOP4();}
#define NOP16()					{NOP8(); NOP8();}
#define NOP16()					{NOP8(); NOP8();}
#define NOP24()					{NOP16(); NOP8();}
#define NOP32()					{NOP16(); NOP16();}
#define NOP40()					{NOP32(); NOP8();}
#define NOP64()					{NOP32(); NOP32();}

#ifndef ei
	#define ei()				enableInterrupts()			//asm("rim")					//enable all interrupts
#endif

#ifndef di
	#define di()				disableInterrupts()			//asm("sim")					//disable all interrupts
#endif

#define F_CPU					(SystemCoreClock + 0*16000000ul / 1)			//default fcpu @ 2Mhz = IntRC@16Mhz / 8. or use SystemCoreClock -> more code space (+200 bytes)

//clock configuration
#define CLK_PRESCALER_CPUDIV1			0x00					//cpu / 1
#define CLK_PRESCALER_CPUDIV2			0x01					//cpu / 2
#define CLK_PRESCALER_CPUDIV4			0x02					//cpu / 4
#define CLK_PRESCALER_CPUDIV8			0x03					//cpu / 8
#define CLK_PRESCALER_CPUDIV16			0x04					//cpu / 16
#define CLK_PRESCALER_CPUDIV32			0x05					//cpu / 32
#define CLK_PRESCALER_CPUDIV64			0x06					//cpu / 64
#define CLK_PRESCALER_CPUDIV128			0x07					//cpu / 128
#define CLK_PRESCALER_HSIDIV1			(0<<3)					//hsi divider 1:1
#define CLK_PRESCALER_HSIDIV2			(1<<3)					//hsi divider 2:1
#define CLK_PRESCALER_HSIDIV4			(2<<3)					//hsi divider 4:1
#define CLK_PRESCALER_HSIDIV8			(3<<3)					//hsi divider 8:1

//#define PWM_MAXDC						0x3fff					//max duty cycle for tim1/12-bit

//use hsi oscillator
void SystemCoreClockHSI(uint8_t CLK_HSIDIV);

//oscillator macros for HSI + dividers
#define SystemCoreClockHSI_16MHz()	SystemCoreClockHSI(CLK_PRESCALER_HSIDIV1)
#define SystemCoreClockHSI_8MHz()	SystemCoreClockHSI(CLK_PRESCALER_HSIDIV2)
#define SystemCoreClockHSI_4MHz()	SystemCoreClockHSI(CLK_PRESCALER_HSIDIV4)
#define SystemCoreClockHSI_2MHz()	SystemCoreClockHSI(CLK_PRESCALER_HSIDIV8)

//arduino-specific defs
#define INPUT				0
#define OUTPUT				1									//(!INPUT)
#define INPUT_PULLUP		2									//input with pull-up
//device specific defitions exist

#define LOW					0
#define HIGH				1									//(!LOW)

#define PI 					3.1415926535897932384626433832795
#define HALF_PI 			(PI / 2)							//1.5707963267948966192313216916398
#define TWO_PI 				(PI + PI)							//6.283185307179586476925286766559
#define DEG_TO_RAD 			(TWO_PI / 360)						//0.017453292519943295769236907684886
#define RAD_TO_DEG 			(360 / TWO_PI)						//57.295779513082320876798154814105
#define EULER 				2.718281828459045235360287471352	//Euler's number

#define SERIAL  			0x0
#define DISPLAY 			0x1

#define LSBFIRST 			0
#define MSBFIRST 			1									//(!LSBFIRST)							//1

#define CHANGE 				1
#define FALLING 			2
#define RISING 				3

#define min(a,b) 			((a)<(b)?(a):(b))
#define max(a,b) 			((a)>(b)?(a):(b))
#define abs(x) 				((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     		((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 		((deg)*DEG_TO_RAD)
#define degrees(rad) 		((rad)*RAD_TO_DEG)
#define sq(x) 				((x)*(x))

#define interrupts() 		ei()
#define noInterrupts() 		di()

#define clockCyclesPerMicrosecond() 	( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) 	( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) 	( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) 			((uint8_t) ((w) & 0xff))
#define highByte(w) 		((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) 	((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(n)				(1ul<<(n))

#define false				0
#define true				(!false)

//characters
#define isAlphaNumeric(c)	isalnum(c)
#define isAlpha(c)			isalpha(c)
#define isAscii(c)			isascii(c)
#define isWhitespace(c)		isblank(c)
#define isControl(c)		iscntrl(c)
#define isDigit(c)			isdigit(c)
#define isGraph(c)			isgraph(c)
#define isLowerCase(c)		islower(c)
#define isPrintable(c)		isprint(c)
#define isPunct(c)			ispunct(c)
#define isSpace(c)			isspace(c)
#define isUpperCase(c)		isupper(c)
#define isHexadecimalDigit(c)	isxdigit(c)

//random number
#define randomSeed(seed)	srand(seed)
#define random(max)			random2(0, max)
#define random2(min, max)	((min) + (int32_t) ((max) - (min)) * rand() / 32768)

//random() will need manual porting

//external setup/loop - defined by user
extern void setup(void);
extern void loop(void);

//gpio
void pinMode(PIN_TypeDef pin, uint8_t mode);
void digitalWrite(PIN_TypeDef pin, uint8_t mode);
int digitalRead(PIN_TypeDef pin);

//time base
uint32_t millis(void);
uint32_t micros(void);
uint32_t ticks(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

//advanced io
//shiftin/out: bitOrder = MSBFIRST or LSBFIRST
void tone(void);							//tone frequency specified by F_TONE in STM8Sduino.h
void noTone(void);
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder);
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val);
//wait for a pulse and return timing
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state);

//pwm output
//dc = 0x00..0x0fff for pwm2/3/4/5, 0x00..0xffff for pwm1
//RP4=PWM1, RP12=PWM2, RP13=PWM3, RP14=PWM4, RP15=PWM5
void analogWrite(PIN_TypeDef pin, uint16_t dc);

//analog read on ADC1
//read DRL first for right aligned results
uint16_t analogRead(AIN_TypeDef ain);

//analog reference - not applicable for STM8S
#define analogReference(x)					

//external interrupt 
void attachInterrupt(PIN_TypeDef pin, void (*isrptr)(void), uint8_t mode);
void detachInterrupt(PIN_TypeDef pin);

//stm8s extensions
//spi
uint8_t SPIWrite(uint8_t order, uint8_t dat);
uint8_t SPIWrites(uint8_t order, uint8_t *dat, uint16_t length);
uint8_t SPIRead(uint8_t order);

//i2c
void I2CStart(void);
void I2CStop();
void I2CWriteaddr(uint8_t addr);
void I2CWrite(uint8_t dat);
uint8_t I2CRead(uint8_t ack);
#define I2CRestart()			do {I2C_stop(); I2C_start();} while (0)

//return time base ticks
uint32_t ticks(void);

//uart1
void serial1Begin(uint32_t baudrate);
void serial1Print(unsigned char *str);
void serial1Println(unsigned char *str);
void serial1Write(unsigned char ch);
unsigned char serial1Read(void);
uint8_t serial2Available(void);
//uart2
void serial2Begin(uint32_t baudrate);
void serial2Print(unsigned char *str);
void serial2Println(unsigned char *str);
void serial2Write(unsigned char ch);
unsigned char serial2Read(void);
uint8_t serial2Available(void);
//serial defaults to uart2
#if defined(UART1)
#define serialBegin				serial1Begin
#define serialPrint				serial1Print
#define serialPrintln			serial1Println
#define serialWrite				serial1Write
#define serialRead				serial1Read
#define serialAvailable			serial1Available
#else
#define serialBegin			serial2Begin
#define serialPrint			serial2Print
#define serialPrintln			serial2Println
#define serialWrite			serial2Write
#define serialRead				serial2Read
#define serialAvailable		serial2Available
#endif
//end stm8s extensions

//interrupt from iostm8s105c6.h - don't change
/*-------------------------------------------------------------------------
 *      Interrupt vector numbers
 *-----------------------------------------------------------------------*/
#define AWU_vector                           0x03
#define CLK_CSS_vector                       0x04
#define CLK_SWITCH_vector                    0x04
#define EXTI0_vector                         0x05
#define EXTI1_vector                         0x06
#define EXTI2_vector                         0x07
#define EXTI3_vector                         0x08
#define EXTI4_vector                         0x09
#define SPI_CRCERR_vector                    0x0C
#define SPI_MODF_vector                      0x0C
#define SPI_OVR_vector                       0x0C
#define SPI_RXNE_vector                      0x0C
#define SPI_TXE_vector                       0x0C
#define SPI_WKUP_vector                      0x0C
#define TIM1_OVR_BIF_vector                  0x0D
#define TIM1_OVR_TIF_vector                  0x0D
#define TIM1_OVR_UIF_vector                  0x0D
#define TIM1_CAPCOM_CC1IF_vector             0x0E
#define TIM1_CAPCOM_CC2IF_vector             0x0E
#define TIM1_CAPCOM_CC3IF_vector             0x0E
#define TIM1_CAPCOM_CC4IF_vector             0x0E
#define TIM1_CAPCOM_COMIF_vector             0x0E
#define TIM2_OVR_UIF_vector                  0x0F
#define TIM3_OVR_UIF_vector                  0x11
#define TIM2_CAPCOM_CC1IF_vector             0x10
#define TIM2_CAPCOM_CC2IF_vector             0x10
#define TIM2_CAPCOM_CC3IF_vector             0x10
#define TIM2_CAPCOM_TIF_vector               0x10
#define TIM3_CAPCOM_CC1IF_vector             0x12
#define TIM3_CAPCOM_CC2IF_vector             0x12
#define TIM3_CAPCOM_CC3IF_vector             0x12
#define TIM3_CAPCOM_TIF_vector               0x12
#define I2C_ADD10_vector                     0x15
#define I2C_ADDR_vector                      0x15
#define I2C_AF_vector                        0x15
#define I2C_ARLO_vector                      0x15
#define I2C_BERR_vector                      0x15
#define I2C_BTF_vector                       0x15
#define I2C_OVR_vector                       0x15
#define I2C_RXNE_vector                      0x15
#define I2C_SB_vector                        0x15
#define I2C_STOPF_vector                     0x15
#define I2C_TXE_vector                       0x15
#define I2C_WUFH_vector                      0x15
#define UART2_T_TC_vector                    0x16
#define UART2_T_TXE_vector                   0x16
#define UART2_R_IDLE_vector                  0x17
#define UART2_R_LBDF_vector                  0x17
#define UART2_R_OR_vector                    0x17
#define UART2_R_PE_vector                    0x17
#define UART2_R_RXNE_vector                  0x17
#define ADC1_AWDG_vector                     0x18
#define ADC1_AWS0_vector                     0x18
#define ADC1_AWS1_vector                     0x18
#define ADC1_AWS2_vector                     0x18
#define ADC1_AWS3_vector                     0x18
#define ADC1_AWS4_vector                     0x18
#define ADC1_AWS5_vector                     0x18
#define ADC1_AWS6_vector                     0x18
#define ADC1_AWS7_vector                     0x18
#define ADC1_AWS8_vector                     0x18
#define ADC1_AWS9_vector                     0x18
#define ADC1_EOC_vector                      0x18
#define TIM4_OVR_UIF_vector                  0x19
#define FLASH_EOP_vector                     0x1A
#define FLASH_WR_PG_DIS_vector               0x1A

#endif
