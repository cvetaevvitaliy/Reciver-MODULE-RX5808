#ifndef	_ENCODER_H_
#define	_ENCODER_H_
#include <avr/io.h>
#include <avr/interrupt.h>
//_________________________________________
//порт и выводы к которым подключен энкодер
#define PORT_Enc 	PORTC 	
#define PIN_Enc 	PINC
#define DDR_Enc 	DDRC
#define Pin1_Enc 	1  // для изминения стороны вращения , поменяйте местами пины
#define Pin2_Enc 	2
//______________________

//unsigned int  EncData = 0; - счетный регистор енкодера

void EncoderInit(void);
void timer0_init(void);                    // Инцилизация таймера0, прерывания каждую милисекунду
void EncoderScan(void);                    
unsigned char EncoderRead(void);           // возвращает значения энкодера
void EncoderWrite(unsigned int a);         // Записываем новое значения энкодера
unsigned int timer_count_ms_read(void);    // Считываем значения таймера
void timer_count_ms_write(unsigned int t); // Записываем значания в системный таймер
#endif  //encoder_h
