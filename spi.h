//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: Mega
//
//  Compiler....: 
//
//  Description.: Драйвер SPI
//
//  Data........: 2.10.12
//
//***************************************************************************
#ifndef SPI_H
#define SPI_H


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define SPI_PORTX   PORTB
#define SPI_DDRX    DDRB

//Обязательно указывать все пины аппаратного SPI
//выводы интерфейса SPI atmega328
#define SPI_SS     2 
#define SPI_MOSI   3
#define SPI_MISO   4
#define SPI_SCK    5

/*____________функции____________________*/

/*инициализация SPI модуля*/
void SPI_Init(void); 

/*отправить байт данных по SPI*/
void SPI_WriteByte(uint8_t data); 

/*отправить и получить байт данных по SPI*/
uint8_t SPI_ReadByte(uint8_t data);

/*отправить несколько байт данных по SPI*/
void SPI_WriteArray(uint8_t num, uint8_t *data);

/*отправить и получить несколько байт данных по SPI*/
void SPI_ReadArray(uint8_t num, uint8_t *data);

#endif //SPI_H