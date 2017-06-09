#include "stm8sduino.h"								//include stm8suduino defs

#define LED		PD0							//LED attached to pin 24 (=PD0 on STM8SDiscovery)
#define LED_DLY		100							//waste some time, in ms

//user setup
void setup(void) {
	pinMode(LED, OUTPUT); digitalWrite(LED, LOW);				//set LED as output
}

//user loop
void loop(void) {
	digitalWrite(LED, !digitalRead(LED));					//flip the led
	delay(LED_DLY);								//waste sometime
}
