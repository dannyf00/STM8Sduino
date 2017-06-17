#include "stm8sduino.h"					//we use aruidno port for stm8s

//global definitions
//struct used to map a pin to GPIO+mask
typedef struct {
	volatile GPIO_TypeDef *gpio;					//gpio for a pin
	uint8_t mask;						//pin mask - 8bit
} PIN2GPIO;

#define PWM_PR			((1ul<<PWMOUT_BITs) - 1)			//period for 16-bit pwm

//void empty handler for exti
void empty_handler(void) {
	//do nothing
}

#if defined(USE_EXTI)
//exti handlers
static void (*_isrptr_extia) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extib) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extic) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extid) (void)=empty_handler;					//tim isr handler ptr
static void (*_isrptr_extie) (void)=empty_handler;					//tim isr handler ptr

//exti handlers
//#pragma vector = EXTI0_vector					//exti 0 interrupt
//__interrupt void extia_isr(void) {
INTERRUPT_HANDLER(extia_isr, EXTI0_vector) {
	//clear the flag
	_isrptr_extia();							//execute the isr handler
}

//#pragma vector = EXTI1_vector					//exti 0 interrupt
//__interrupt void extib_isr(void) {
INTERRUPT_HANDLER(extib_isr, EXTI1_vector) {
	//clear the flag
	_isrptr_extib();							//execute the isr handler
}

//#pragma vector = EXTI2_vector					//exti 0 interrupt
//__interrupt void extic_isr(void) {
INTERRUPT_HANDLER(extic_isr, EXTI2_vector) {
	//clear the flag
	_isrptr_extic();							//execute the isr handler
}

//#pragma vector = EXTI3_vector					//exti 0 interrupt
//__interrupt void extid_isr(void) {
INTERRUPT_HANDLER(extid_isr, EXTI3_vector) {
	//clear the flag
	_isrptr_extid();							//execute the isr handler
}

//#pragma vector = EXTI4_vector					//exti 0 interrupt
//__interrupt void extie_isr(void) {
INTERRUPT_HANDLER(extie_isr, EXTI4_vector) {
	//clear the flag
	_isrptr_extie();							//execute the isr handler
}
#endif											//EXTI

//global variables
//for time base off TIM4 @ 256:1 prescaler (TIM4 is 8-bit)
//volatile uint32_t timer1_millis = 0;
volatile uint32_t timer_ticks = 0;							//msw for time base on TIM4
//static uint16_t timer1_fract = 0;
volatile uint32_t SystemCoreClock=16000000ul/8;				//systemcoreclock. defaults to HSI (@16Mhz / 8)

//TIM4 overflow isr
//tim4 isr handler
//#pragma vector = TIM4_OVR_UIF_vector		//tim4 overflow interrupt
//__interrupt void tim4_ovr_isr(void) {
INTERRUPT_HANDLER(tim4_ovr_isr, TIM4_OVR_UIF_vector) {
	TIM4->SR1 &=~(1<<0);					//TIM4_SR_UIF=0;							//clear the flag
	timer_ticks+=0x100ul;					//increase timer_ticks = 8-bit timer/counter + 1:1 prescaler
}

//user-supplied code prototype
extern void setup(void);			//user set-up
extern void loop(void);				//user loop

//define your chips here
//declare pins based on chip packaging

//STM8S pin defs for all packaging
//F: TSSOP20pin - 003F, 103f, 903f
//K: LQFP32pin - 003k, 005k, 103k, 105k, 207k
//S: LQFP44pin - 105s, 207s
//C: LQFP48pin - 005c, 007c, 105c, 207c, 208c
//R: LQFP64pin - 207r, 208r
//M: LQFP80pin - 207m
//
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR PACKAGE
//Pin  0.. 7 -> GPIOA
//Pin  8..15 -> GPIOB
//Pin 16..23 -> GPIOC
//Pin 24..31 -> GPIOD
//Pin 32..39 -> GPIOE
//Pin 40..47 -> GPIOF
//Pin 48..55 -> GPIOG
//Pin 56..63 -> GPIOH
//Pin 64..71 -> GPIOI
const PIN2GPIO GPIO_PinDef[]={
	{GPIOA, 1<<0},						//STM8Sduino Pin  0 = PA0
	{GPIOA, 1<<1},						//GPIOA1 = OSCI
	{GPIOA, 1<<2}, 						//GPIOA2 = OSCO
	{GPIOA, 1<<3},						//STM8Sduino Pin  3 = PA3 TIM2/CH3
	{GPIOA, 1<<4},						//STM8Sduino Pin  4 = PA4
	{GPIOA, 1<<5},						//STM8Sduino Pin  5 = PA5
	{GPIOA, 1<<6},						//STM8Sduino Pin  6 = PA6
	{GPIOA, 1<<7},						//STM8Sduino Pin  7 = PA7
	
	{GPIOB, 1<<0},						//STM8Sduino Pin  8 = PB0 AIN0
	{GPIOB, 1<<1},						//STM8Sduino Pin  9 = PB1 AIN1
	{GPIOB, 1<<2},						//STM8Sduino Pin 10 = PB2 AIN2
	{GPIOB, 1<<3},						//STM8Sduino Pin 11 = PB3 AIN3
	{GPIOB, 1<<4},						//STM8Sduino Pin 12 = PB4 AIN4
	{GPIOB, 1<<5},						//STM8Sduino Pin 13 = PB5 AIN5
	{GPIOB, 1<<6},						//STM8Sduino Pin 14 = PB6 AIN6
	{GPIOB, 1<<7},						//STM8Sduino Pin 15 = PB7 AIN7
	
	{GPIOC, 1<<0},						//STM8Sduino Pin 16 = PC0
	{GPIOC, 1<<1},						//STM8Sduino Pin 17 = PC1 TIM1/CH1
	{GPIOC, 1<<2},						//STM8Sduino Pin 18 = PC2 TIM1/CH2
	{GPIOC, 1<<3},						//STM8Sduino Pin 19 = PC3 TIM1/CH3
	{GPIOC, 1<<4},						//STM8Sduino Pin 20 = PC4 TIM1/CH4
	{GPIOC, 1<<5},						//STM8Sduino Pin 21 = PC5
	{GPIOC, 1<<6},						//STM8Sduino Pin 22 = PC6
	{GPIOC, 1<<7},						//STM8Sduino Pin 23 = PC7
	
	{GPIOD, 1<<0},						//STM8Sduino Pin 24 = PD0 TIM3/CH2
	{GPIOD, 1<<1},						//STM8Sduino Pin 25 = PD1
	{GPIOD, 1<<2},						//STM8Sduino Pin 26 = PD2 TIM3/CH1
	{GPIOD, 1<<3},						//STM8Sduino Pin 27 = PD3 TIM2/CH2
	{GPIOD, 1<<4},						//STM8Sduino Pin 28 = PD4 TIM2/CH1
	{GPIOD, 1<<5},						//STM8Sduino Pin 29 = PD5 
	{GPIOD, 1<<6},						//STM8Sduino Pin 30 = PD6 
	{GPIOD, 1<<7},						//STM8Sduino Pin 31 = PD7
	
	{GPIOE, 1<<0},						//STM8Sduino Pin 32 = PE0
	{GPIOE, 1<<1},						//STM8Sduino Pin 33 = PE1
	{GPIOE, 1<<2},						//STM8Sduino Pin 34 = PE2
	{GPIOE, 1<<3},						//STM8Sduino Pin 35 = PE3
	{GPIOE, 1<<4},						//STM8Sduino Pin 36 = PE4
	{GPIOE, 1<<5},						//STM8Sduino Pin 37 = PE5
	{GPIOE, 1<<6},						//STM8Sduino Pin 38 = PE6 AIN9
	{GPIOE, 1<<7},						//STM8Sduino Pin 39 = PE7 AIN8
	
	{GPIOF, 1<<0},						//STM8Sduino Pin 40 = PF0
	{GPIOF, 1<<1},						//STM8Sduino Pin 41 = PF1
	{GPIOF, 1<<2},						//STM8Sduino Pin 42 = PF2
	{GPIOF, 1<<3},						//STM8Sduino Pin 43 = PF3
	{GPIOF, 1<<4},						//STM8Sduino Pin 44 = PF4 AIN12
	{GPIOF, 1<<5},						//STM8Sduino Pin 45 = PF5
	{GPIOF, 1<<6},						//STM8Sduino Pin 46 = PF6
	{GPIOF, 1<<7},						//STM8Sduino Pin 47 = PF7

#if defined(GPIOG)						//GPIOG not present on all chips
	{GPIOG, 1<<0},						//STM8Sduino Pin 48 = PG0
	{GPIOG, 1<<1},						//STM8Sduino Pin 49 = PG1
	{GPIOG, 1<<2},						//STM8Sduino Pin 50 = PG2
	{GPIOG, 1<<3},						//STM8Sduino Pin 51 = PG3
	{GPIOG, 1<<4},						//STM8Sduino Pin 52 = PG4
	{GPIOG, 1<<5},						//STM8Sduino Pin 53 = PG5
	{GPIOG, 1<<6},						//STM8Sduino Pin 54 = PG6
	{GPIOG, 1<<7},						//STM8Sduino Pin 55 = PG7
#endif

#if defined(GPIOH)						//GPIOH not present on all chips
	{GPIOH, 1<<0},						//STM8Sduino Pin 56 = PH0
	{GPIOH, 1<<1},						//STM8Sduino Pin 57 = PH1
	{GPIOH, 1<<2},						//STM8Sduino Pin 58 = PH2
	{GPIOH, 1<<3},						//STM8Sduino Pin 59 = PH3
	{GPIOH, 1<<4},						//STM8Sduino Pin 60 = PH4
	{GPIOH, 1<<5},						//STM8Sduino Pin 61 = PH5
	{GPIOH, 1<<6},						//STM8Sduino Pin 62 = PH6
	{GPIOH, 1<<7},						//STM8Sduino Pin 63 = PH7
#endif

#if defined(GPIOI)						//GPIOI not present on all chips
	{GPIOI, 1<<0},						//STM8Sduino Pin 64 = PI0
	{GPIOI, 1<<1},						//STM8Sduino Pin 65 = PI1
	{GPIOI, 1<<2},						//STM8Sduino Pin 66 = PI2
	{GPIOI, 1<<3},						//STM8Sduino Pin 67 = PI3
	{GPIOI, 1<<4},						//STM8Sduino Pin 68 = PI4
	{GPIOI, 1<<5},						//STM8Sduino Pin 69 = PI5
	{GPIOI, 1<<6},						//STM8Sduino Pin 70 = PI6
	{GPIOI, 1<<7},						//STM8Sduino Pin 71 = PI7
#endif
};

//configure clock
//use hsi oscillator
void SystemCoreClockHSI(uint8_t CLK_HSIDIV) {
   /* check the parameters */
    //assert_param(IS_CLK_PRESCALER_OK(CLK_HSIDIV));
	
	//CLK_DeInit();
	CLK->ICKR |= CLK_ICKR_HSIEN;					//enable HSI
	while ((CLK->ICKR & CLK_ICKR_HSIRDY) == 0) continue;	//wait until HSI is ready
	CLK->SWCR|= CLK_SWCR_SWEN;						//start the switch
	CLK->SWR = 0xe1;								//0xe1->HSI, 0xd2->LSI, 0xb4->HSE	//CLK_HSICmd(ENABLE);
	while ((CLK->SWCR & CLK_SWCR_SWBSY) == 1) continue;	//wait until the busy signal is no more	//while(SET != CLK_GetFlagStatus(CLK_FLAG_HSIRDY)) continue;	//make sure hsi is ready
	CLK->CKDIVR = (CLK->CKDIVR &~CLK_CKDIVR_HSIDIV) | (CLK_HSIDIV & CLK_CKDIVR_HSIDIV);	//set hsi divider
	CLK->CKDIVR = (CLK->CKDIVR &~CLK_CKDIVR_CPUDIV) | (CLK_PRESCALER_CPUDIV1& CLK_CKDIVR_CPUDIV);	//set cpu divier -> default to 1:1
	//update SystemCoreClock
	switch (CLK_HSIDIV) {
		case CLK_PRESCALER_HSIDIV1: SystemCoreClock = HSI_VALUE / 1; break;
		case CLK_PRESCALER_HSIDIV2: SystemCoreClock = HSI_VALUE / 2; break;
		case CLK_PRESCALER_HSIDIV4: SystemCoreClock = HSI_VALUE / 4; break;
		case CLK_PRESCALER_HSIDIV8: SystemCoreClock = HSI_VALUE / 8; break;
		default: SystemCoreClock = HSI_VALUE / 8; break;
	}
}

//use hse oscillator
void SystemCoreClockHSE(void) {
   /* check the parameters */
    //assert_param(IS_CLK_PRESCALER_OK(CLK_HSIDIV));
	
	//CLK_DeInit();
	CLK->ECKR |= CLK_ECKR_HSEEN;					//enable HSE
	while ((CLK->ECKR & CLK_ECKR_HSERDY) == 0) continue;	//wait until HSE is ready
	CLK->SWCR|= CLK_SWCR_SWEN;						//start the switch
	CLK->SWR = 0xb4;								//0xe1->HSI, 0xd2->LSI, 0xb4->HSE	//CLK_HSICmd(ENABLE);
	while ((CLK->SWCR & CLK_SWCR_SWBSY) == 1) continue;	//wait until the busy signal is no more	//while(SET != CLK_GetFlagStatus(CLK_FLAG_HSIRDY)) continue;	//make sure hsi is ready
	//update SystemCoreClock
	SystemCoreClock = HSE_VALUE;
}

//Arduino Functions: GPIO
//set a pin mode to INPUT, INPUT_PULLUP, INPUT_PULLDN or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	//if (mode==INPUT) GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	//else GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	switch (mode) {
	case OUTPUT_OD: 	GIO_OUTOD(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
	case OUTPUT: 		GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
	case INPUT_PULLUP: 	GIO_INPU(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
	case INPUT:
	default: 			GIO_INFL(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
	}
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (GIO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

//Arduino Functions: Time
//return time base ticks
uint32_t ticks(void) {
	uint32_t m;
	uint8_t f;
	
	//use double reads
	do {
		m = timer_ticks;
		f = TIM4->CNTR;
	} while (m != timer_ticks);
		
	//return ((((uint32_t) m << 16) | (f << 8)) >> 10) / 2 / clockCyclesPerMicrosecond();		//128:1 prescaler
	return (m | f) << TIM4_PS;
}

//return microseconds
uint32_t micros(void) {
	uint32_t m;					//stores overflow count
	uint8_t f;					//return the fractions / TIM4 8-bit value
	
	//use double reads
	do {
		m = timer_ticks;
		f = TIM4->CNTR;
	} while (m != timer_ticks);
	//now m and f are consistent
	return ((m | f) / clockCyclesPerMicrosecond()) << TIM4_PS;						//2^TIM4_PS:1 prescaler
}
	
//return milliseconds
uint32_t millis(void) {
	uint32_t m;
	uint8_t f;
	
	//use double reads
	do {
		m = timer_ticks;
		f = TIM4->CNTR;
	} while (m != timer_ticks);

	return (((m | f) / clockCyclesPerMicrosecond()) << TIM4_PS) / 1000;
}

//delay millisseconds
void delay(uint32_t ms) {
#if 0												//using millis()
	uint32_t start_time = millis();

	while (millis() - start_time < ms) continue;
#else												//using ticks()
	uint32_t start_time = ticks();
	
	ms *= (SystemCoreClock / 1000);					//convert ms to ticks
	while (ticks() - start_time < ms) continue;
#endif
}

//delay micros seconds
void delayMicroseconds(uint32_t us) {
#if 0												//using micros()
	uint32_t start_time = micros();
	
	while (micros() - start_time < us) continue;
#else												//using ticks()
	uint32_t start_time = ticks();
	
	us *= (SystemCoreClock / 1000000ul);			//convert us to ticks
	while (ticks() - start_time < us) continue;
#endif
}
//end Time


//Arduino Functions: Advanced IO
//shift in - from arduino code base / not optimized
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder) {
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST)
			value |= digitalRead(dataPin) << i;
		else
			value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}

//shift out - from arduino code base / not optimized
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
			digitalWrite(dataPin, !!(val & (1 << i)));
		else	
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);		
	}
}

//wait for a pulse and return timing
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state) {
	uint32_t tmp;
	state = (state == LOW)?LOW:HIGH;				
	while (digitalRead(pin) == state) continue;		//wait for the pin to opposite
	//now pin is _state
	tmp = micros();
	state = (state == LOW)?HIGH:LOW;				//calculate the state to end the wait
	while (digitalRead(pin) == state) continue;		//wait for the pin to go back to its original state
	tmp = micros() - tmp;							//calculate the pulse width
	return tmp;
}
//end Advanced IO

//pwm output - not implemented yet
//top at PWMOUT_PR -> max 16-bit pwm output
void analogWrite(PIN_TypeDef pin, uint16_t dc) {
	//bound dc range
	dc = (dc > PWM_PR)?PWM_PR:dc;				//limit the range
	switch (pin) {
#if defined(TIM1) && defined(USE_PWM1)
	//tim1 ch1/2/3/4
	case PC1/*17*/:								//pin 17 -> PC1/TIM1CH1
		//load the duty cycle
		TIM1->CCR1H = dc>>8;
		TIM1->CCR1L = dc;
		TIM1->CCER1|= 1<<0;						//1->enable OCi output, 0->disable OCi output
		break;
	case PC2/*18*/:								//pin 18 -> PC2/TIM1CH2
		//load the duty cycle
		TIM1->CCR2H = dc>>8;
		TIM1->CCR2L = dc;
		TIM1->CCER1|= 1<<4;						//1->enable OCi output, 0->disable OCi output
		break;
	case PC3/*19*/:								//pin 19 -> PC3/TIM1CH3
		//load the duty cycle
		TIM1->CCR3H = dc>>8;
		TIM1->CCR3L = dc;
		TIM1->CCER2|= 1<<0;						//1->enable OCi output, 0->disable OCi output
		break;
	case PC4/*20*/:								//pin 20 -> PC4/TIM1CH4
		//load the duty cycle
		TIM1->CCR4H = dc>>8;
		TIM1->CCR4L = dc;
		TIM1->CCER2|= 1<<4;						//1->enable OCi output, 0->disable OCi output
		break;
#endif
#if defined(TIM2) && defined(USE_PWM2)			//3x pwm
	//TIM2 ch1/2/3
	case PD4/*28*/: 							//pin 28 -> PD4/TIM2CH1
		//load the duty cycle
		TIM2->CCR1H = dc>>8;
		TIM2->CCR1L = dc;
		TIM2->CCER1|= 1<<0;						//1->enable OCi output, 0->disable OCi output
		break;
	case PD3/*27*/: 							//pin 27 -> PD3/TIM2CH2
		//load the duty cycle
		TIM2->CCR2H = dc>>8;
		TIM2->CCR2L = dc;
		TIM2->CCER1|= 1<<4;						//1->enable OCi output, 0->disable OCi output
		break;
	case PA3/*3*/:								//pin  3 -> PA3/TIM2CH3
		//load the duty cycle
		TIM2->CCR3H = dc>>8;
		TIM2->CCR3L = dc;
		TIM2->CCER2|= 1<<0;						//1->enable OCi output, 0->disable OCi output
		break;
#endif
#if defined(TIM3) && defined(USE_PWM3)			//2ch pwm
	//TIM3 ch1/2
	case PD2/*26*/:								//pin 26 -> PD2/TIM3CH1
		//load the duty cycle
		TIM3->CCR1H = dc>>8;
		TIM3->CCR1L = dc;
		TIM3->CCER1|= 1<<0;						//1->enable OCi output, 0->disable OCi output
		break;
	case PD0/*24*/: 							//pin 24 -> PD0/TIM3CH2
		//load the duty cycle
		TIM3->CCR2H = dc>>8;
		TIM3->CCR2L = dc;
		TIM3->CCER1|= 1<<4;						//1->enable OCi output, 0->disable OCi output
		break;
#endif
	default: 	break; 							//do nothing
	}
}

#if defined(USE_ADC1)
//analog read on ADC1
//read DRL first for right aligned results
//user responsible for configuring ain to a gpio pin
uint16_t analogRead(AIN_TypeDef ain) {
	uint16_t tmp;
	ADC1->CSR = (ADC1->CSR &~0x0f) | (ain & 0x0f);	//set the channel
	//ADC1->CR1|= (1<<0);							//start the conversion
	ADC1->CR1|= (1<<0);								//start the conversion
	while ((ADC1->CSR & (1<<7)) == 0) continue;		//wait for the previous conversion to end (EOC = 1 if conversion is complete)
	//read the adc results
	tmp = ADC1->DRL;								//read DRL first for right aligned results
	tmp|= ADC1->DRH << 8;
	ADC1->CSR &=~(1<<7);							//clear the EOC
	//ADC1->CR1&=~(1<<0);							//power down adc
	return tmp;
}

//change analog reference - not applicable on STM8S
#define analogReference(x)
#endif												//ADC1

//beep functions
//user needs to configure tone pin as output - see datasheet
void tone(void) {
	BEEP->CSR |= (1<<5);							//'1'->turn on tone, '0'->turn off tone
}

void noTone(void) {
	BEEP->CSR &=~(1<<5);							//'1'->turn on tone, '0'->turn off tone
}

#if defined(USE_EXTI)
//activate interrupt
//mode: RISING, FALLING and CHANGE
void attachInterrupt(PIN_TypeDef pin, void (*isrptr)(void), uint8_t mode){
	//convert arduino modes to EXTI modes
	switch (mode) {
	case RISING: 	mode = 0x01; break;			//rising edge triggered
	case CHANGE: 	mode = 0x03; break;			//change triggered
	case FALLING: 	
	default: 		mode = 0x02; break;			//default is falling edge triggered
	}
	
	//disable global interrupt first before changing EXTI->CRn registers
	noInterrupts();
	
	//configure EXTI_CR1/CR2 registers
	if (GPIO_PinDef[pin].gpio==GPIOA) {			//configure interrupt on porta
		_isrptr_extia = isrptr;					//install user handler
		EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PAIS) | ((mode << 0) & EXTI_CR1_PAIS);
	}

	if (GPIO_PinDef[pin].gpio==GPIOB) {			//configure interrupt on portb
		_isrptr_extib = isrptr;					//install user handler
		EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PBIS) | ((mode << 2) & EXTI_CR1_PBIS);
	}
	
	if (GPIO_PinDef[pin].gpio==GPIOC) {			//configure interrupt on portc
		_isrptr_extic = isrptr;					//install user handler
		EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PCIS) | ((mode << 4) & EXTI_CR1_PCIS);
	}
	
	if (GPIO_PinDef[pin].gpio==GPIOD) {			//configure interrupt on portd
		_isrptr_extid = isrptr;					//install user handler
		EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PDIS) | ((mode << 6) & EXTI_CR1_PDIS);
	}
	
	if (GPIO_PinDef[pin].gpio==GPIOE) {			//configure interrupt on porte
		_isrptr_extie = isrptr;					//install user handler
		EXTI->CR2 = (EXTI->CR2 &~EXTI_CR2_PEIS) | ((mode << 0) & EXTI_CR2_PEIS);
	}

	//enable global interrupt
	interrupts();	

	//configure gpio pin, with interrupt capability by setting CR2
	GPIO_PinDef[pin].gpio->CR2 |= GPIO_PinDef[pin].mask;
	
}

//deactivate interrupt
//pin revert back to input, floating mode
void detachInterrupt(PIN_TypeDef pin){
	uint8_t mode = 0x00;						//falling and low -> default state
	void (*isrptr)(void) = empty_handler;		//reset handler to empty_handler. for ease of coding
	
	//convert arduino modes to EXTI modes
	//switch (mode) {
	//case RISING: 	mode = 0x01; break;			//rising edge triggered
	//case CHANGE: 	mode = 0x03; break;			//change triggered
	//case FALLING: 
	//default: 		mode = 0x02; break;			//default is falling edge triggered
	//}
	
	//configure EXTI_CR1/CR2 registers
	if (GPIO_PinDef[pin].gpio==GPIOA) {			//configure interrupt on porta
		_isrptr_extia = isrptr;					//install user handler
		EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PAIS) | ((mode << 0) & EXTI_CR1_PAIS);
	}

	if (GPIO_PinDef[pin].gpio==GPIOB) {			//configure interrupt on portb
		_isrptr_extib = isrptr;					//install user handler
		//EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PBIS) | ((mode << 2) & EXTI_CR1_PBIS);
	}
	
	if (GPIO_PinDef[pin].gpio==GPIOC) {			//configure interrupt on portc
		_isrptr_extic = isrptr;					//install user handler
		//EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PCIS) | ((mode << 4) & EXTI_CR1_PCIS);
	}
	
	if (GPIO_PinDef[pin].gpio==GPIOD) {			//configure interrupt on portd
		_isrptr_extid = isrptr;					//install user handler
		//EXTI->CR1 = (EXTI->CR1 &~EXTI_CR1_PDIS) | ((mode << 6) & EXTI_CR1_PDIS);
	}
	
	if (GPIO_PinDef[pin].gpio==GPIOE) {			//configure interrupt on porte
		_isrptr_extie = isrptr;					//install user handler
		//EXTI->CR2 = (EXTI->CR2 &~EXTI_CR2_PEIS) | ((mode << 0) & EXTI_CR2_PEIS);
	}

	//disable pin interrupt capability by clear CR2
	GPIO_PinDef[pin].gpio->CR2 &=~GPIO_PinDef[pin].mask;
}
#endif										//exti

//uart1 routines
#if defined(UART1) && defined(USE_UART1)
//configure uart1 -> similar configurations among UART1/2/3/4
//UART1: TX on PA2, RX on PA3
void serial1Begin(uint32_t BaudRate) {
	//route clock to uart2
	CLK->PCKENR1 |= CLK_PCKENR1_UART1;		//'1'=clock enabled, '0'=clock disabled

  	//user to configure TXpin (PD5) as output through serialBegin()
	//user to configure RXpin (PD6) as input through serialBegin()
	//configure UART2 RX (on PD6)
	
	//configure UART2 TX: 8-bit, 1-stop bit, no parity, syncmode disable
	UART1->CR1 |= (1<<5);					//'1'->disable uart, '0'->enable uart transmitter

	uint16_t tmp = SystemCoreClock / BaudRate;			//max of 16-bit
	//set BRR2 first, per the datasheet
	UART1->BRR2 = ((tmp >> 8) & 0xf0) | (tmp & 0x0f);	//BRR2 is the top 4 + bottom 4 digits
	UART1->BRR1 = tmp >> 4;								//BRR1 is the middle 4 digits
	
	UART1->CR5 = 0;							//default value
	UART1->CR4 = 	(0<<6) |				//'0'->LIN break detection interrupt disabled, '1'->enabled
					(0<<5) |				//'0'->10-bit line detection, '1'->11-bit line detection
					(0<<4) |				//'0'->line break not detected, '1'->line break detected
					(0<<0) |				//4-bit Address of the UART note
					0x00;
	UART1->CR3 = 	(0<<6) |				//'0'->disable LIN, '1'->enable LIN
					(0<<4) |				//0->1 stop bit, 1->reserved, 2->2 stop bit, 3->1.5 stop bit
					(0<<3) |				//'0'->disable SCK, '1'->enable SCK
					(0<<2) |				//'0'->SCK idles low, '1'->SCK idles high
					(0<<1) |				//'0'->capture data on first clock transition, '1'->capture data on 2nd clock transition
					(0<<0) |				//'0'->last clock pulse is not output, '1'->last clock pulse is on output
					0x00;
	UART1->CR2 =	(0<<7) |				//'0'->tx interrupt disabled, '1'->tx interrupt enabled
					(0<<6) |				//'0'->tx completion interrupt diabled, '1'->tx completion interrupt enabled
					(0<<5) |				//'0'->rx interrupt disabled, '1'->rx interrupt enabled
					(0<<4) |				//'0'->idle line interrupt disabled, '1'->idle line interrupt enabled
					(1<<3) |				//'0'->tx disabled, '1'->tx enabled
					(1<<2) |				//'0'->rx disabled, '1'->rx enabled
					(0<<1) |				//'0'->receiver in active mode, '1'->receiver in mute mode
					(0<<0) |				//'0'->don't send break; '1'->send break;
					0x00;
	UART1->CR1 = 	(0<<7) |				//9th bit for receiving in 9-bit mode (when M=1)
					(0<<6) |				//9th bit for transmission in 9-bit mode (when M=1)
					(0<<5) |				//'0'->uart enabled, '1'->uart disabled
					(0<<4) |				//'0'->1start bit, 8 data bits, n stop bits; '1'->1 start bit, 9 data bits, 1 stop bit
					(0<<3) |				//'0'->idle line, '1'->address line wakes up uart
					(0<<2) |				//'0'->parity disabled, '1'->parity enabled
					(0<<1) |				//'0'->even partity, '1'->odd parity
					(0<<0) |				//'0'->parity interrupt disabled, '1'->parity interrupt enabled
					0x00;
	//configure interrupt -> if uart2 to be interrupt-driven
	
	//enable UART1
	UART1->CR1 &=~(1<<5);					//'0'->enable uart, '1'->disable uart
	
}

//uart1 send a char
void serial1Write(unsigned char dat) {
    UART1->DR = dat;                        //load the data buffer
    while (!(UART1->SR & UART1_SR_TC));    	//wait for the transmission to complete
}

//uart1 returns a char
unsigned char serial1Read(void) {
    while (!(UART1->SR & UART1_SR_RXNE));  	//wait fo the receipt to terminate
    return UART1->DR;                       //save the transmission buffer
}

//uart1 print a string
void serial1Print(unsigned char *str) {
	do {
		while (!(UART1->SR & UART1_SR_TXE));	//wait for the transmission buffer to empty
		UART1->DR = *str++;					//load the data into transmission buffer
	} while (*str);
}

//uart1 print a string + return
void serial1Println(unsigned char *str) {
	serial1Print(str);						//print the string
	serial1Print("\n\r");					//print the return
}

//test if uart1 is available
//return true if transmission on uart1 has completed
uint8_t serial1Available(void) {
	return (UART1->SR & UART1_SR_TC)?true:false;
}
#endif											//use_uart1

#if defined(USE_UART2) && defined(UART2)		//005, 105
//configure uart2 -> similar configurations among UART1/2/3/4
//UART2: TX on PD5, RX on PD6
void serial2Begin(uint32_t BaudRate) {
	uint16_t tmp;
	//route clock to uart2
	CLK->PCKENR1 |= CLK_PCKENR1_UART2;		//'1'=clock enabled, '0'=clock disabled

  	//user to configure TXpin (PD5) as output through serialBegin()
	//user to configure RXpin (PD6) as input through serialBegin()
	//configure UART2 RX (on PD6)
	
	//configure UART2 TX: 8-bit, 1-stop bit, no parity, syncmode disable
	UART2->CR1 |= (1<<5);					//'1'->disable uart, '0'->enable uart transmitter

	/*uint16_t */tmp = SystemCoreClock / BaudRate;			//max of 16-bit
	//set BRR2 first, per the datasheet
	UART2->BRR2 = ((tmp >> 8) & 0xf0) | (tmp & 0x0f);	//BRR2 is the top 4 + bottom 4 digits
	UART2->BRR1 = tmp >> 4;								//BRR1 is the middle 4 digits
	
	UART2->CR1 = 	(0<<7) |				//9th bit for receiving in 9-bit mode (when M=1)
					(0<<6) |				//9th bit for transmission in 9-bit mode (when M=1)
					(0<<5) |				//'0'->uart enabled, '1'->uart disabled
					(0<<4) |				//'0'->1start bit, 8 data bits, n stop bits; '1'->1 start bit, 9 data bits, 1 stop bit
					(0<<3) |				//'0'->idle line, '1'->address line wakes up uart
					(0<<2) |				//'0'->parity disabled, '1'->parity enabled
					(0<<1) |				//'0'->even partity, '1'->odd parity
					(0<<0) |				//'0'->parity interrupt disabled, '1'->parity interrupt enabled
					0x00;
	UART2->CR2 =	(0<<7) |				//'0'->tx interrupt disabled, '1'->tx interrupt enabled
					(0<<6) |				//'0'->tx completion interrupt diabled, '1'->tx completion interrupt enabled
					(0<<5) |				//'0'->rx interrupt disabled, '1'->rx interrupt enabled
					(0<<4) |				//'0'->idle line interrupt disabled, '1'->idle line interrupt enabled
					(1<<3) |				//'0'->tx disabled, '1'->tx enabled
					(1<<2) |				//'0'->rx disabled, '1'->rx enabled
					(0<<1) |				//'0'->receiver in active mode, '1'->receiver in mute mode
					(0<<0) |				//'0'->don't send break; '1'->send break;
					0x00;
	UART2->CR3 = 	(0<<6) |				//'0'->disable LIN, '1'->enable LIN
					(0<<4) |				//0->1 stop bit, 1->reserved, 2->2 stop bit, 3->1.5 stop bit
					(0<<3) |				//'0'->disable SCK, '1'->enable SCK
					(0<<2) |				//'0'->SCK idles low, '1'->SCK idles high
					(0<<1) |				//'0'->capture data on first clock transition, '1'->capture data on 2nd clock transition
					(0<<0) |				//'0'->last clock pulse is not output, '1'->last clock pulse is on output
					0x00;
	UART2->CR4 = 	(0<<6) |				//'0'->LIN break detection interrupt disabled, '1'->enabled
					(0<<5) |				//'0'->10-bit line detection, '1'->11-bit line detection
					(0<<4) |				//'0'->line break not detected, '1'->line break detected
					(0<<0) |				//4-bit Address of the UART note
					0x00;
	UART2->CR5 = 0;							//default value
	//configure interrupt -> if uart2 to be interrupt-driven
	
	//enable UART2
	UART2->CR1 &=~(1<<5);					//'0'->enable uart, '1'->disable uart
	
}

//uart2 send a char
void serial2Write(unsigned char dat) {
    UART2->DR = dat;                        //load the data buffer
    while (!(UART2->SR & UART2_SR_TC));    	//wait for the transmission to complete
}

//uart2 returns a char
unsigned char serial2Read(void) {
    while (!(UART2->SR & UART2_SR_RXNE));  	//wait fo the receipt to terminate
    return UART2->DR;                       //save the transmission buffer
}

//uart2 print a string
void serial2Print(unsigned char *str) {
	do {
		while (!(UART2->SR & UART2_SR_TXE));	//wait for the transmission buffer to empty
		UART2->DR = *str++;					//load the data into transmission buffer
	} while (*str);
}

//uart2 print a string + return
void serial2Println(unsigned char *str) {
	serial2Print(str);						//print the string
	serial2Print("\n\r");					//print the return
}

//test if uart2 is available
//return true if transmission on uart2 has completed
uint8_t serial2Available(void) {
	return (UART2->SR & UART2_SR_TC)?true:false;
}
#endif											//use_uart2

#if defined(USE_SPI)
//send char via hardware spi
//MISO on PC7, MOSI on PC6, SCK on PC5
//Order = LSBFIRST or MSBFIRST
uint8_t SPIWrite(uint8_t order, uint8_t dat) {
    if (order == LSBFIRST) SPI->CR1 |= (1<<7);  //'1'->LSB first
    else SPI->CR1 &=~(1<<7);                    //'0'->MSB first
    while ((SPI->SR & (1<<1))==0) continue;      //wait for transmit buffer to be empty (bit 1 goes to 1)
    SPI->DR = dat;                          	//load the transmission buffer -> transmission starts. this approach can be risk for reads
    //consider test busy signal
    while (SPI->SR & SPI_SR_BSY) continue;      //'1'->SPI is buy, '0'->SPI is not busy
    //comment the above line for higher throughput if you don't care about the read-back
    return SPI->DR;                         	//return spi data buffer
}

//buffer write of a string
uint8_t SPIWrites(uint8_t order, uint8_t *dat, uint16_t length) {
    if (order == LSBFIRST) SPI->CR1 |= (1<<7);  //'1'->LSB first
    else SPI->CR1 &=~(1<<7);                    //'0'->MSB first
	while (length--) {
		while ((SPI->SR & (1<<1))==0) continue;      //wait for transmit buffer to be empty (bit 1 goes to 1)
    	SPI->DR = *dat++;                          	//load the transmission buffer -> transmission starts. this approach can be risk for reads
	}
    //consider test busy signal
    while (SPI->SR & SPI_SR_BSY) continue;      //'1'->SPI is buy, '0'->SPI is not busy
    //comment the above line for higher throughput if you don't care about the read-back
    return SPI->DR;                         	//return spi data buffer
}

//read spi
//MISO on PC7, MOSI on PC6, SCK on PC5
//Order = LSBFIRST or MSBFIRST
uint8_t SPIRead(uint8_t order) {
    //uint8_t tmp;
    if (order == LSBFIRST) SPI->CR1 |= (1<<7);  //'1'->LSB first
    else SPI->CR1 &=~(1<<7);                    //'0'->MSB first
    while ((SPI->SR & (1<<1))==0) continue;      //wait for transmit buffer to be empty (bit 1 goes to 1)
    SPI->DR = /*dat*/0x00;                      //load the transmission buffer -> transmission starts. this approach can be risk for reads
    //consider test busy signal
    while (SPI->SR & SPI_SR_BSY) continue;      //'1'->SPI is buy, '0'->SPI is not busy
    //comment the above line for higher throughput if you don't care about the read-back
    return SPI->DR;                         	//return spi data buffer
}
#endif											//spi

#if defined(USE_I2C) 
//send a start condition
void I2CStart(void) {
    I2C->CR2 |= (1<<0);							//'1'->send a start condition
    while (!(I2C->SR1 & (1<<0)));				//'1'->start bit generated
	(void)I2C->SR1;								//clear the start condition by reading I2C->SR1 followed by a write to I2C->DR
}
//send a stop condition
void I2CStop() {
    I2C->CR2 |= (1<<1);							//'1'->send a stop condition
    while (!(I2C->SR3 & (1<<0)));				//'1'->stop condition is detected on the bus
}

//write address
void I2CWriteaddr(uint8_t addr) {
    I2C->DR = addr;
    while (!(I2C->SR1 & (1<<1)));				//'1'->address sent, '0'->address not yet sent
    (void) I2C->SR1;							//clear EV6 -> see datasheet
	(void) I2C->SR3;
    I2C->CR2 |= (1<<2);							//send the ack
}

//send data
void I2CWrite(uint8_t dat) {
    I2C->DR = dat;								//load data
    while (!(I2C->SR1 & (1<<7)));				//'1' when transmission buffer is empty. not set in address phase
}

//read i2c
uint8_t I2CRead(uint8_t ack) {
    if (ack)
        I2C->CR2 |= (1<<2);						//'1'->send ack
    else
        I2C->CR2 &= ~(1<<2);						//'0'->don't send ack
    while (!(I2C->SR1 & (1<<6)));				//'0'->receiver buffer is empty, '1'->receiver buffer is not empty
    return I2C->DR;
}	

#endif											//i2c

//initialize the mcu
//reset the mcu
void mcu_init(void){
	//configure clock sources
	//SystemCoreClockHSE();								//set clock to hse, @ HSE_VALUE
	//SystemCoreClockHSI_16MHz();							//set clock to hsi, _2MHz, _4MHz, _8MHz, _16MHz
	SystemCoreClockHSI_2MHz();						//default setting: set clock to hsi, _2MHz, _4MHz, _8MHz, _16MHz

	//disable all peripherals by default
	CLK->PCKENR1 = CLK->PCKENR2 = 0x00;		//'0'->disable clock to a peripheral, '1'->enable clock
	
	//configure time base for micros/millis on TIM4 (or TIM2 if TIME4 isn't available)
	timer_ticks = 0;						//reset timer ticks
	//enable the clock to peripheral
	//TIM4 running at 2^TIM4_PS prescaler
	CLK->PCKENR1 |= CLK_PCKENR1_TIM4;					//'1'=clock enabled, '0'=clock disabled
	//set up the time base
	//stop the timer
	//TIM4->CR1 &=~(1<<0);					//stop the timer
	//set up the timer
	TIM4->CR1 = (1<<7) |					//'1'->enable auto reload buffer
	  			(0<<5) |					//'0'->edge aligned. 1..3->center aligned
				(0<<4) |					//'0'->up counter, '1' downcounter
				(0<<3) |					//'0'->continuous mode, '1'->one pulse mode
				(0<<2) |					//'0'-> update enable source
				(0<<1) |					//'0'-> update enabled
				(0<<0);						//counter disabled
	//TIMx->CR2 = 0;							//default value
	//TIMx->SMCR = 0;							//default value
	//TIMx->ETR = 0;							//'0'->external trigger not inverted
	TIM4->PSCR = (TIM4_PS) & 0x07;			//3-bit prescaler = 2^0x00 = 1:1
	TIM4->CNTR = 0; 						//TIMx->CNTRL = 0;			//reset the counter
	TIM4->ARR = 0xff;						//top at 256				//pr = pr - 1; TIMx->ARR = pr;			//load up the auto reload register - msb first
	TIM4->SR1&=~(1<<0);						//clear UIF
	TIM4->IER|= (1<<0);						//'1'->enable overflow interrupt, '0'->disable interrupt
	//re-enable the counter
	TIM4->CR1 |= (1<<0);	
	
	//configure pwm time base - not yet implemented
	//enable the clock to TIM1
	CLK->PCKENR1 |= CLK_PCKENR1_TIM1;		//'1'=clock enabled, '0'=clock disabled
	TIM1->CR1 = (1<<7) |					//'1'->enable auto reload buffer
	  			(1<<5) |					//'0'->edge aligned. 1..3->center aligned
				(0<<4) |					//'0'->up counter, '1' downcounter
				(0<<3) |					//'0'->continuous mode, '1'->one pulse mode
				(0<<2) |					//'0'-> update enable source
				(0<<1) |					//'0'-> update enabled
				(0<<0);						//counter disabled
	TIM1->CR2 = 0;							//reset value
	TIM1->PSCRH = (1ul<<(PWMOUT_PS)-1)>>8;
	TIM1->PSCRL = (1ul<<(PWMOUT_PS)-1);		//prescaler = 1:1. map'd to 3-bit prescaler to be compatible with tim2/3
	TIM1->CNTRH = TIM1->CNTRL = 0;			//reset the counter
	TIM1->ARRH = (PWM_PR)>>8; 
	TIM1->ARRL = (PWM_PR);					//top at 0x0fff
	TIM1->SR1 &=~(1<<0);					//clear uif
	TIM1->CR1 |= (1<<0);					//start the timer

#if defined(TIM1) && defined(USE_PWM1)
	//configure TIM1/OC1/2/3/4 -> output disabled
	TIM1->CCMR1 = 	(7<<4) |				//0b110->PWM mode 1, 0b111->PWM mode 2
					(0<<3) |				//'0'->OCiPE disabled, '1'->OCiPE enabled
					(0<<0);					//0b00->OCi as output
	TIM1->CCMR4 = TIM1->CCMR3 = TIM1->CCMR2 = TIM1->CCMR1;
	TIM1->CCER1 = 	(0<<5) |				//OC2: 0->active high, 1->active low
					(0<<4) |				//OC2: 1->enable OC2, 0->disable OC2
					(0<<1) |				//OC1: 0->active high, 1->active low
					(0<<0) |				//OC1: 1->enable OC2, 0->disable OC2
					0x00;
	//check CCER2 for OC3 settings
	TIM1->CCER2 = 	(0<<5) |				//OC4: 0->active high, 1->active low
					(0<<4) |				//OC4: 1->enable OC2, 0->disable OC2
					(0<<1) |				//OC3: 0->active high, 1->active low
					(0<<0) |				//OC3: 1->enable OC2, 0->disable OC2
					0x00;
#endif										//tim1

#if defined(TIM2) && defined(USE_PWM2)
	//enable the clock to TIM2
	CLK->PCKENR1 |= CLK_PCKENR1_TIM2;					//'1'=clock enabled, '0'=clock disabled
	TIM2->CR1 = (1<<7) |					//'1'->enable auto reload buffer
	  			//(1<<5) |					//'0'->edge aligned. 1..3->center aligned
				//(0<<4) |					//'0'->up counter, '1' downcounter
				(0<<3) |					//'0'->continuous mode, '1'->one pulse mode
				(0<<2) |					//'0'-> update enable source
				(0<<1) |					//'0'-> update enabled
				(0<<0);						//counter disabled
	TIM2->PSCR = (PWMOUT_PS) & 0x0f;			//4-bit prescaler, valid range [0..15], 2^PSCR divider, 0 = 1:1.
	TIM2->CNTRH = TIM1->CNTRL = 0;			//reset the counter
	TIM2->ARRH = (PWM_PR)>>8; 
	TIM2->ARRL = (PWM_PR);					//top at 0x0fff
	TIM2->SR1 &=~(1<<0);					//clear uif
	TIM2->CR1 |= (1<<0);					//start the timer
	
	//configure TIM2/OC1/2/3 -> output disabled
	TIM2->CCMR1 = 	(7<<4) |				//0b110->PWM mode 1, 0b111->PWM mode 2
					(0<<3) |				//'0'->OCiPE disabled, '1'->OCiPE enabled
					(0<<0);					//0b00->OCi as output
	TIM2->CCMR3 = TIM2->CCMR2 = TIM2->CCMR1;
	TIM2->CCER1 = 	(0<<5) |				//OC2: 0->active high, 1->active low
					(0<<4) |				//OC2: 1->enable OC2, 0->disable OC2
					(0<<1) |				//OC1: 0->active high, 1->active low
					(0<<0) |				//OC1: 1->enable OC2, 0->disable OC2
					0x00;
	//check CCER2 for OC3 settings
	TIM2->CCER2 = 	(0<<1) |				//OC3: 0->active high, 1->active low
					(0<<0) |				//OC3: 1->enable OC2, 0->disable OC2
					0x00;
#endif										//tim2
	
#if defined(TIM3) && defined(USE_PWM3)
	//enable the clock to TIM3
	CLK->PCKENR1 |= CLK_PCKENR1_TIM3;					//'1'=clock enabled, '0'=clock disabled
	TIM3->CR1 = (1<<7) |					//'1'->enable auto reload buffer
	  			//(1<<5) |					//'0'->edge aligned. 1..3->center aligned
				//(0<<4) |					//'0'->up counter, '1' downcounter
				(0<<3) |					//'0'->continuous mode, '1'->one pulse mode
				(0<<2) |					//'0'-> update enable source
				(0<<1) |					//'0'-> update enabled
				(0<<0);						//counter disabled
	TIM3->PSCR = (PWMOUT_PS) & 0x0f;		//4-bit prescaler, valid range [0..15], 2^PSCR divider, 0 = 1:1.
	TIM3->CNTRH = TIM1->CNTRL = 0;			//reset the counter
	TIM3->ARRH = (PWM_PR)>>8; 
	TIM3->ARRL = (PWM_PR);					//top at 0x0fff
	TIM3->SR1 &=~(1<<0);					//clear uif
	TIM3->CR1 |= (1<<0);					//start the timer
	
	//configure TIM3/OC1/2 -> output disabled
	TIM3->CCMR1 = 	(7<<4) |				//0b110->PWM mode 1, 0b111->PWM mode 2
					(0<<3) |				//'0'->OCiPE disabled, '1'->OCiPE enabled
					(0<<0);					//0b00->OCi as output
	TIM3->CCMR2 = TIM3->CCMR1;
	TIM3->CCER1 = 	(0<<5) |				//OC2: 0->active high, 1->active low
					(0<<4) |				//OC2: 1->enable OC2, 0->disable OC2
					(0<<1) |				//OC1: 0->active high, 1->active low
					(0<<0) |				//OC1: 1->enable OC2, 0->disable OC2
					0x00;
	//OC3 not present on TIM3
	
#endif										//tim3


#if defined(USE_ADC1)
	//route clock to ADC
	CLK->PCKENR2 |= CLK_PCKENR2_ADC;		//'1'=clock enabled, '0'=clock disabled
	//configure the adc
	ADC1->CR1 = ((ADC1_PS)<<4) |			//3-bit adc prescaler: 7 = Fmaster / 18, 6 = Fmaster / 12, ...
				0x00;
	ADC1->CR2 = (0<<6) |					//0->conversion on external trigger disabled
				(0<<4) |					//0->external event -> TIM1 TRGO
				(1<<3) |					//'1'->right aligned, '0'->left aligned
				(0<<1) |					//'0'->scan disabled, '1'->scan enabled
				0x00;
	ADC1->CR3 = (0<<7);						//'0'->data buffer disabled, read from DRH..DRL
	ADC1->CR1 |= (1<<0);					//turn on the adc
#endif										//ADC1
	
	//configure beep / tone generator
	BEEP->CSR = (0x00 << 6) |					//set divider: 0b00->Fls/8, 0b01->Fls/4, 0b1x->Fls/2. default: 8x
				(0<<5) |						//0->disable beep, 1->enable beep
				(((128000ul/ F_TONE / 8 > 2)?(128000ul / F_TONE / 8 - 2):0) & 0x1f);					//set beep divider. 0x00->2:1, 0x01->3:1, 0x1e->32:1
	//tone off at start-up
	
#if defined(USE_EXTI)
	//configure EXTI
	//use default / boot-up EXTI values
#endif										//exti
	
#if defined(USE_SPI)
	//configure SPI
	//route clock to SPI
	CLK->PCKENR1 |= CLK_PCKENR1_SPI;		//'1'=clock enabled, '0'=clock disabled
    //stop spi
    //SPI->CR1 &=~(1<<6);                     //'1'->enable spi, '0'->disable spi
    //set up the clock ...
    SPI->CR1 =  (0<<7) |                     //'0'->MSB first, '1'->LSB first
                (0<<6) |                    //'0'->spi disabled, '1'->spi enabled
                (((SPI_PS) & 0x07) << 3) |  //set 3-bit spi prescaler, 0..7 -> 2:1 ... 256:1
                (1<<2) |                    //'1'->master, '0'->slave
                (0<<1) |                    //'0'->SCK idles low, '1'->SCK idles high
                (0<<0) |                    //'0'->the first clock transition is the first data capture edge, '1'->the second clock transition is the first data capture edge
                0x00;
    SPI->CR2 =  (0<<7) |                    //'0'->2-line unidirectional data mode, '1'->1-line bidirectional data mode
                (0<<6) |                    //'0'->input enabled (receive-only mode), '1'->output enabled (transmit only mode)
                (0<<5) |                    //'0'->CRC calculation disabled, '1'->CRC calculation enabled
                (0<<4) |                    //'0'->next transmit value is from Tx buffer, '1'->next transmit value is from Tx CRC register
                (0<<2) |                    //'0'->full duplex (transmit and receive), '1'->output disabled (receive inly)
                (0<<1) |                    //'0'->software slave management disabled, '1'->software slave management enabled
                (0<<0) |                    //'0'->slave mode, '1'->Master mode. This bit has effect only when SSM bit (bit 1 of CR2) is set
                0x00;
    SPI->ICR =  (0<<7) |                     //'0'->disable TX interrupt
                (0<<6) |                    //'0'->disable RX interrupt
                (0<<5) |                    //'0'->disable spi error interrupt
                (0<<4) |                    //'0'->disable spi wakeup interrupt
				0x00;
    SPI->SR = 0;                            //'0'->clear all error status
    SPI->DR = 0;                            //clear spi data register
    SPI->CR1|= (1<<6);                      //'0'->disable spi, '1'->enable spi
    //spi is now enabled    
#endif										//spi

#if defined(USE_I2C)
	//configure i2c
	//route clock to i2c
	CLK->PCKENR1 |= CLK_PCKENR1_I2C;		//'1'=clock enabled, '0'=clock disabled
    //configure i2c
	I2C->CR1 &=~(1<<0);						//'0'->disable i2c, '1'->enable i2c
	I2C->CR2 = 	(0<<7) |					//'0'->software reset disabled, '1'->software reset enabled
				(0<<3) |					//'0'->ACK bit controls acknowledge, '1'->ACK bit controls the next byte
				(0<<2) |					//'0'->no ACK returned, '1'->ACK returned. can only be set after PE = 1
				(0<<1) |					//'0'->don't generate stop condition, '1'->generate a stop condition
				(0<<0) |					//'0'->don't generate a start condition, '1'->generate a start condition
				0x00;
	I2C->FREQR = clockCyclesPerMicrosecond();
	I2C->FREQR = (I2C->FREQR>=24)?24:((I2C->FREQR==0)?1:I2C->FREQR);	//top at 24Mhz, and bottom at 1Mhz
	I2C->CCRL = (clockCyclesPerMicrosecond() * 10);
	I2C->CCRH = (clockCyclesPerMicrosecond() * 10) >> 8;
#if defined(I2C_FASTMODE)
	I2C->CCRH = (I2C->CCRH & 0x0f) | 		//lowest 4 bits effective
				(1<<7) |					//'1'->fast mode, '0'->standard mode
				(0<<6) |					//'1'->duty cycle 9/(16+9), '0'->duty cycle 2/(3+2)
				0x00;
#else
	I2C->CCRH = (I2C->CCRH & 0x0f) | 		//lowest 4 bits effective
				(0<<7) |					//'1'->fast mode, '0'->standard mode
				(0<<6) |					//'1'->duty cycle 9/(16+9), '0'->duty cycle 2/(3+2)
				0x00;
#endif
    I2C->OARH = (0<<7) |					//'0'-> 7-bit addressing, '1'->10-bit addressing
				(1<<6) |					//ADDCONF bit ***must always be 1***
				(0x00<<1) |					//bit 9..8 of 10-bit own address
				0x00;
	I2C->OARL = 0x00;						//own address, bit 0 don't care
    //I2C->TRISER must be configured before the I2C is turned on
	//1000ns for standard mode and 300ns for fast mode
	if (I2C->CR2 & (1<<7)) {				//fast mode
		I2C->TRISER = I2C->FREQR + 1;		//simple math to configure I2C rise time to 1000ns for standard mode
	} else {								//slow mode
		I2C->TRISER = I2C->FREQR * 3 / 10 + 1;		//simple math to configure I2C rise time to 300ns for fast mode
	}
	//clear interrupt flags and then enable interrupt
	
	//enable i2c
	I2C->CR1 |= (1<<0);						//enable i2c
	//enable ACK
	I2C->CR2 |= (1<<2);						//'1'->send ACK,'0'->don't send ACK - can only be set after PE = 1
#endif										//i2c
	
	//always enable interrupts - for micros/millis
	interrupts();
}

//templated code for main()
int main(void) {
	mcu_init();							//reset the chip
	setup();							//user-set up the code
	while (1) {
		loop();							//user specified loop
	}
	return 0;
}

