/*
 * File:    iofuncs.h
 * Author:  Santiago Dominguez Vidal
 * Date:    November 2018
 */

#ifndef __IOFUNCS_H__
#define __IOFUNCS_H__

#define MSEC 40000
#define MAX_CYCLES 4500 //(2400us)(1875 cycles_per_ms)
#define MIN_CYCLES 1020 //(544us)(1875 cycles_per_ms)

int digitalRead(char port, uint8_t pin){
    switch(port){
    case 'A':   return GPIOPinRead(GPIO_PORTA_BASE, 1<<pin);
    case 'B':   return GPIOPinRead(GPIO_PORTB_BASE, 1<<pin);
    case 'C':   return GPIOPinRead(GPIO_PORTC_BASE, 1<<pin);
    case 'D':   return GPIOPinRead(GPIO_PORTD_BASE, 1<<pin);
    case 'E':   return GPIOPinRead(GPIO_PORTE_BASE, 1<<pin);
    case 'F':   return GPIOPinRead(GPIO_PORTF_BASE, 1<<pin);
    case 'G':   return GPIOPinRead(GPIO_PORTG_BASE, 1<<pin);
    case 'H':   return GPIOPinRead(GPIO_PORTH_BASE, 1<<pin);
    case 'J':   return GPIOPinRead(GPIO_PORTJ_BASE, 1<<pin);
    case 'K':   return GPIOPinRead(GPIO_PORTK_BASE, 1<<pin);
    case 'L':   return GPIOPinRead(GPIO_PORTL_BASE, 1<<pin);
    case 'M':   return GPIOPinRead(GPIO_PORTM_BASE, 1<<pin);
    case 'N':   return GPIOPinRead(GPIO_PORTN_BASE, 1<<pin);
    case 'P':   return GPIOPinRead(GPIO_PORTP_BASE, 1<<pin);
    case 'Q':   return GPIOPinRead(GPIO_PORTQ_BASE, 1<<pin);
    default: return -1;
    }
}

void debounce(){
    SysCtlDelay(20*MSEC);
}

void longdebounce(){
	SysCtlDelay(40*MSEC);
}

bool button_pushed(uint8_t pin){
    if(!digitalRead('J',pin)){
        debounce();
        while(!digitalRead('J',pin));
        debounce();
        return true;
    }
    return false;
}

int32_t map(int32_t val,int32_t in_min,int32_t in_max,int32_t out_min,int32_t out_max){
    return (val-in_min)*(out_max-out_min)/(in_max-in_min) + out_min;
}

float fmap(float val,float in_min,float in_max,float out_min,float out_max){
    return (val-in_min)*(out_max-out_min)/(in_max-in_min) + out_min;
}

uint32_t deg_to_cycles(uint32_t angle){
    return map(angle,0,180,MIN_CYCLES,MAX_CYCLES);
}

#endif
