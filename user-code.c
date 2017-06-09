#include "stm8sduino.h"							//include stm8suduino defs

#define LED			PD0							//LED attached to pin 24 (=PD0 on STM8SDiscovery)
#define LED_DLY		100							//waste some time, in ms
#define AIN			0							//ADC ch0
#define BTN			PC5							//(8*2+5)						//pin 21 = PC5
#define LOOP_CNT	1

//flip led
void led_flp(void) {
	if (digitalRead(BTN)==HIGH) digitalWrite(LED, HIGH);
	else digitalWrite(LED, LOW);
	//digitalWrite(LED, !digitalRead(LED));
}

//user setup
void setup(void) {
	pinMode(LED, OUTPUT); digitalWrite(LED, LOW);						//set LED as output
	pinMode(BTN, INPUT_PULLUP); 
	//attachInterrupt(BTN, led_flp, CHANGE);			//test exti
	//DAC1Write(100);										//test dac
	//serialBegin(9600);
}

volatile uint32_t time0, time1;
	
//user loop
void loop(void) {
	uint16_t i;

#if 0
	//test adc
	time0=ticks();
	for (i=0; i<ADC_CNT; i++) analogRead(A0);
	time1=ticks()-time0;
	if (time1>10) NOP();
#endif

	//test GPIO
	time0=ticks();
	for(i=0; i<LOOP_CNT; i++) digitalWrite(LED, !digitalRead(LED));		//flip the led
	time1=ticks() - time0;
	if (time1>10) NOP();
	
	//test timing
	time0=ticks();
	delay(LED_DLY);														//waste sometime
	time1=ticks() - time0;
	if (time1>10) NOP();
	
#if 0
	//test Uart2
	time0=micros();
	Serial_print("0123456789012345678901234567890123456789");
	time1=micros() - time0;
	if (time1>100) NOP();
#endif
	
#if 0
	//test spi
	time0=micros();
	for (i=0; i<1000; i++) SPIshiftOut(MSBFIRST, 0x55);
	time1=micros() - time0;
	if (time1>100) NOP();
#endif

#if 0
	//test i2c
	time0=micros();
	for (i=0; i<1000; i++) I2C_write(0x55);
	time1=micros() - time0;
	if (time1>100) NOP();
#endif	
}
