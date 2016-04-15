// GPIO.c
// Runs on LM4F120/TM4C123
// Initialize four GPIO pins as outputs.  Continually generate output to
// drive simulated stepper motor.
// Daniel Valvano
// May 28, 2014

/* This example accompanies the books
  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
  Volume 1 Program 4.5

"Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Volume 2 Example 2.2, Program 2.8, Figure 2.32

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// PD3 is an output to LED3, negative logic
// PD2 is an output to LED2, negative logic
// PD1 is an output to LED1, negative logic
// PD0 is an output to LED0, negative logic
#include <stdio.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "UART.h" 


#define PF4 (*((volatile uint32_t *) 0x4005D040))

void Output_Init(void);
void GPIO_Init(void){  
  SYSCTL_RCGCGPIO_R |= 0x08;        // 1) activate port D
  while((SYSCTL_PRGPIO_R&0x08)==0){};   // allow time for clock to stabilize
                                    // 2) no need to unlock PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO
  GPIO_PORTD_DIR_R |= 0x00;         // 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~0x0F;      // 6) regular port function 
  GPIO_PORTD_DEN_R |= 0x0F;         // 7) enable digital I/O on PD3-0
}
void PortD_Output(uint32_t data){ 
	GPIO_PORTD_DATA_R = data;
}	
uint32_t PortD_Input(void){
	return (GPIO_PORTD_DATA_R);
}	
void GPIOF_Init(void){  
  SYSCTL_RCGCGPIO_R |= 0x00000020;        // 1) activate port D
  while((SYSCTL_PRGPIO_R&0x00000020)==0){};   // allow time for clock to stabilize
  GPIO_PORTF_LOCK_R = 0x4C4F434B;                                  // 2) no need to unlock PD3-0
	GPIO_PORTF_CR_R = 0x10;
  GPIO_PORTF_DIR_R |= 0xFF;         // 5) make PD3-0 out 
  GPIO_PORTF_PUR_R = 0x00;					// Inputs get PUR
  GPIO_PORTF_DEN_R |= 0xFF;         // 7) enable digital I/O on PD3-0
}

uint32_t PortF_Input(void) {
	return (GPIO_PORTF_DATA_R&0x10);
}
void PortF_Output(uint32_t data) {
	GPIO_PORTF_DATA_R = data;
}	

/***************
	SENSOR TRIGGER 
	***************/

void sensorTrig(void){
	uint32_t trig;
	
	trig = 80;
	
	PortF_Output(0x10);
	while(trig>0){
		trig--;
	}
	
	PortF_Output(0x00);
	
}
/***************
	SENSOR ECHO
	***************/

uint32_t distance(void){
	uint32_t distance;
	uint32_t echo, cycles, wait;
	
	cycles=0;
	wait = 2500;
	
	while(wait >0){
		echo = PortD_Input();
		
		while (echo ==1){
			echo= PortD_Input();
			cycles++;
		}
		
		wait--;
	}
	
	distance = cycles/580;
	if ((distance > 150)||(distance == 0)){
		distance = 100;
	}
	
	return distance;
}


void delay_loop(uint32_t del){
	while (del>0){
		del--;
	}
}


void objectDetect(uint32_t range){
	
	if (range <18){
		PortF_Output(0x02);
		//PortF_Output(0x04);
		delay_loop(500000);
	} else if ((range<40) &&(range >18)){
		PortF_Output(0x08);
		
	} else {
		PortF_Output(0x00);
		
	}
}

int main(void){
	Output_Init();
	printf("*****BSNS*******\nBlind Spot Notification System\n");
  GPIO_Init();
  GPIOF_Init();
	while(1){
		uint32_t trial;
		sensorTrig();
		trial = distance();
		objectDetect(trial);
		printf("distance = %d \n", trial);
		delay_loop(8000);
	}

}