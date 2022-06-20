/*
 * utils.c
 *
 *  Created on: 18 Jun. 2022
 *      Author: paul
 */



void rebootme(int why) {
	volatile unsigned int i;

	while (1) {
#ifdef HARDWARE_WATCHDOG
		__disable_irq();			// mask all interrupts
		err_leds(why);
#else
		err_leds(why);
		osDelay(6000);
		__NVIC_SystemReset();   // reboot
#endif
	}
}
