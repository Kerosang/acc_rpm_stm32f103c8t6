/*MIT License

Copyright (c) 2016 Roger Dahl

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*/
/*
 * TM1637_.c
 *
 *  Created and Modified on: 20-Oct-2018
 *      Author: Vidura Embedded
 */
#include "TM1637.h"
#include "main.h"
// #include "gpio.h"

const char segmentMap[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
    0x00};

uint8_t getSegmentmap(uint8_t number)
{
    return segmentMap[number];
}
void TM1637_ClkHigh(void)
{
    HAL_GPIO_WritePin(TM_CLK_GPIO_Port, TM_CLK_Pin, GPIO_PIN_SET);
}

void TM1637_ClkLow(void)
{
    HAL_GPIO_WritePin(TM_CLK_GPIO_Port, TM_CLK_Pin, GPIO_PIN_RESET);
}

void TM1637_DataHigh(void)
{
    HAL_GPIO_WritePin(TM_DIO_GPIO_Port, TM_DIO_Pin, GPIO_PIN_SET);
}

void TM1637_DataLow(void)
{
    HAL_GPIO_WritePin(TM_DIO_GPIO_Port, TM_DIO_Pin, GPIO_PIN_RESET);
}

void TM1637_Demo(void)
{
    uint8_t i = 0;

    TM1637_Init();
    TM1637_SetBrightness(8);

    while (1)
    {
        TM1637_DisplayDecimal(i++, 0);
    }
}

void TM1637_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(TM_DIO_GPIO_Port, TM_DIO_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(TM_CLK_GPIO_Port, TM_CLK_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : DATA_Pin */
    GPIO_InitStruct.Pin = TM_DIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TM_DIO_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : CLK_Pin */
    GPIO_InitStruct.Pin = TM_CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TM_CLK_GPIO_Port, &GPIO_InitStruct);

    TM1637_SetBrightness(8);
}

void TM1637_DisplayDecimal(int v, int displaySeparator)
{
    unsigned char digitArr[4];
    for (int i = 0; i < 4; ++i)
    {
        digitArr[i] = segmentMap[v % 10];
        if (i == 2 && displaySeparator)
        {
            digitArr[i] |= 1 << 7;
        }
        v /= 10;
    }

    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();

    for (int i = 0; i < 4; ++i)
    {
        TM1637_WriteByte(digitArr[3 - i]);
        TM1637_ReadResult();
    }

    TM1637_Stop();
}
void TM1637_DisplayDecimal_mci(int v, int displaySeparator)
{
    unsigned char digitArr[5];
    if (displaySeparator)
    {
        digitArr[0] = 0xff;
    }
    else
    {
        digitArr[0] = 0x0;
    }
    for (int i = 1; i < 5; ++i)
    {
        digitArr[i] = segmentMap[v % 10];

        v /= 10;
    }

    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();

    for (int i = 0; i < 5; ++i)
    {
        TM1637_WriteByte(digitArr[4 - i]);
        TM1637_ReadResult();
    }

    TM1637_Stop();
}

void TM1637_displayFixaddress(int grid, int data)
{
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(0xc0 + grid);
    TM1637_ReadResult();
    TM1637_WriteByte(data);
    TM1637_ReadResult();
    TM1637_Stop();
}
void TM1637_ClearDisplay()
{
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();
    for (int i = 0; i < 5; ++i)
    {
        TM1637_WriteByte(0);
        TM1637_ReadResult();
    }
    TM1637_Stop();
}
void TM1637_displayDecimal_zero_lead(int v)
{
    unsigned char digitArr[5];
    int b = v;
    for (int i = 1; i < 5; ++i)
    {
        digitArr[i] = segmentMap[v % 10];
        v /= 10;
    }
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();
    digitArr[0] = 0;
    if (b < 10)
    {
        digitArr[2] = 0;
        digitArr[3] = 0;
        digitArr[4] = 0;
    }
    else if (b < 100)
    {
        digitArr[3] = 0;
        digitArr[4] = 0;
    }
    else if (b < 1000)
    {
        digitArr[4] = 0;
    }
    for (int i = 0; i < 5; ++i)
    {
        TM1637_WriteByte(digitArr[4 - i]);
        TM1637_ReadResult();
    }
    TM1637_Stop();
}
void TM1637_displaySet4(uint8_t arr[5])
{
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();
    arr[5] = 0;
    for (int i = 0; i < 5; ++i)
    {
        TM1637_WriteByte(arr[i]);
        TM1637_ReadResult();
    }
    TM1637_Stop();
}
void TM1637_displayDecimal_only(int v)
{
    unsigned char digitArr[5];
    for (int i = 1; i < 5; ++i)
    {
        digitArr[i] = segmentMap[v % 10];

        v /= 10;
    }
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();
    digitArr[0] = 0;
    for (int i = 0; i < 5; ++i)
    {
        TM1637_WriteByte(digitArr[4 - i]);
        TM1637_ReadResult();
    }
    TM1637_Stop();
}
void Cock_12_pin(int v, bool colon)
{
    unsigned char digitArr[5];
    for (int i = 1; i < 5; ++i)
    {
        digitArr[i] = segmentMap[v % 10];

        v /= 10;
    }
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();
    for (int i = 0; i < 5; ++i)
    {
        if (colon)
            TM1637_WriteByte(digitArr[4 - i] | 0x80);
        else
        {
            TM1637_WriteByte(digitArr[4 - i]);
        }
        TM1637_ReadResult();
    }
    TM1637_Stop();
}
void Cock_14_pin(int v, bool colon)
{
    unsigned char digitArr[5];
    for (int i = 1; i < 5; ++i)
    {
        digitArr[i] = segmentMap[v % 10];

        v /= 10;
    }
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_ReadResult();
    TM1637_Stop();
    TM1637_Start();
    TM1637_WriteByte(0xc0);
    TM1637_ReadResult();
    if (colon)
    {
        digitArr[0] = 0xff;
    }
    else
    {
        digitArr[0] = 0;
    }
    for (int i = 0; i < 5; ++i)
    {
        TM1637_WriteByte(digitArr[4 - i]);
        TM1637_ReadResult();
    }
    TM1637_Stop();
}
// Valid brightness values: 0 - 8.
// 0 = display off.
void TM1637_SetBrightness(char brightness)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness
    TM1637_Start();
    TM1637_WriteByte(0x87 + brightness);
    TM1637_ReadResult();
    TM1637_Stop();
}

void TM1637_Start(void)
{
    TM1637_ClkHigh();
    TM1637_DataHigh();
    TM1637_DelayUsec(2);
    TM1637_DataLow();
}

void TM1637_Stop(void)
{
    TM1637_ClkLow();
    TM1637_DelayUsec(2);
    TM1637_DataLow();
    TM1637_DelayUsec(2);
    TM1637_ClkHigh();
    TM1637_DelayUsec(2);
    TM1637_DataHigh();
}

void TM1637_ReadResult(void)
{
    TM1637_ClkLow();
    TM1637_DelayUsec(5);

    TM1637_ClkHigh();
    TM1637_DelayUsec(2);
    TM1637_ClkLow();
}

void TM1637_WriteByte(unsigned char b)
{
    for (int i = 0; i < 8; ++i)
    {
        TM1637_ClkLow();
        if (b & 0x01)
        {
            TM1637_DataHigh();
        }
        else
        {
            TM1637_DataLow();
        }
        TM1637_DelayUsec(3);
        b >>= 1;
        TM1637_ClkHigh();
        TM1637_DelayUsec(3);
    }
}

void TM1637_DelayUsec(unsigned int i)
{
    for (; i > 0; i--)
    {
        for (int j = 0; j < 500; ++j)
        {
            __NOP();
        }
    }
}
uint8_t char2segments(char c)
{
    switch (c)
    {
    case '0':
        return 0x3f;
    case '1':
        return 0x06;
    case '2':
        return 0x5b;
    case '3':
        return 0x4f;
    case '4':
        return 0x66;
    case '5':
        return 0x6d;
    case '6':
        return 0x7d;
    case '7':
        return 0x07;
    case '8':
        return 0x7f;
    case '9':
        return 0x6f;
    case '_':
        return 0x08;
    case '^':
        return 0x01; // ¯
    case '-':
        return 0x40;
    case '*':
        return 0x63; // °
    case ' ':
        return 0x00; // space
    case 'A':
        return 0x77; // upper case A
    case 'a':
        return 0x5f; // lower case a
    case 'B':        // lower case b
    case 'b':
        return 0x7c; // lower case b
    case 'C':
        return 0x39; // upper case C
    case 'c':
        return 0x58; // lower case c
    case 'D':        // lower case d
    case 'd':
        return 0x5e; // lower case d
    case 'E':        // upper case E
    case 'e':
        return 0x79; // upper case E
    case 'F':        // upper case F
    case 'f':
        return 0x71; // upper case F
    case 'G':        // upper case G
    case 'g':
        return 0x35; // upper case G
    case 'H':
        return 0x76; // upper case H
    case 'h':
        return 0x74; // lower case h
    case 'I':
        return 0x06; // 1
    case 'i':
        return 0x04; // lower case i
    case 'J':
        return 0x1e; // upper case J
    case 'j':
        return 0x16; // lower case j
    case 'K':        // upper case K
    case 'k':
        return 0x75; // upper case K
    case 'L':        // upper case L
    case 'l':
        return 0x38; // upper case L
    case 'M':        // twice tall n
    case 'm':
        return 0x37; // twice tall ∩
    case 'N':        // lower case n
    case 'n':
        return 0x54; // lower case n
    case 'O':        // lower case o
    case 'o':
        return 0x5c; // lower case o
    case 'P':        // upper case P
    case 'p':
        return 0x73; // upper case P
    case 'Q':
        return 0x7b; // upper case Q
    case 'q':
        return 0x67; // lower case q
    case 'R':        // lower case r
    case 'r':
        return 0x50; // lower case r
    case 'S':        // 5
    case 's':
        return 0x6d; // 5
    case 'T':        // lower case t
    case 't':
        return 0x78; // lower case t
    case 'U':        // lower case u
    case 'u':
        return 0x1c; // lower case u
    case 'V':        // twice tall u
    case 'v':
        return 0x3e; // twice tall u
    case 'W':
        return 0x7e; // upside down A
    case 'w':
        return 0x2a; // separated w
    case 'X':        // upper case H
    case 'x':
        return 0x76; // upper case H
    case 'Y':        // lower case y
    case 'y':
        return 0x6e; // lower case y
    case 'Z':        // separated Z
    case 'z':
        return 0x1b; // separated Z
    }
    return 0;
}