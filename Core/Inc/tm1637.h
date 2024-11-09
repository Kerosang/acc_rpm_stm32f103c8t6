/*
 * TM1637.h
 *
 *  Created on: 20-Oct-2018
 *      Author: Venki
 */

#ifndef TM1637_H_
#define TM1637_H_
#include "main.h"
#include "stdio.h"
#include "stdbool.h"



void TM1637_Init(void);
void TM1637_Demo(void);
void TM1637_DisplayDecimal(int v, int displaySeparator);
void TM1637_SetBrightness(char brightness);
void TM1637_Start(void);
void TM1637_Stop(void);
void TM1637_ReadResult(void);
void TM1637_WriteByte(unsigned char b);
void TM1637_DelayUsec(unsigned int i);
void TM1637_ClkHigh(void);
void TM1637_ClkLow(void);
void TM1637_DataHigh(void);
void TM1637_DATALow(void);
void TM1637_DisplayDecimal_mci(int v, int displaySeparator);
void TM1637_displayFixaddress(int grid, int data);
void TM1637_displayDecimal_only(int v);
void Cock_12_pin(int v,bool colon);
void Cock_14_pin(int v,bool colon);
void TM1637_displayDecimal_zero_lead(int v);
void TM1637_displaySet4(uint8_t arr[5]);
uint8_t getSegmentmap(uint8_t number);
uint8_t char2segments(char c);
void TM1637_ClearDisplay();
#endif /* TM1637_H_ */
