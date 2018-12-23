#include "encoder.h"
#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define SET(p, n)    p |= _BV(n)
#define CLR(p, n)    p &= ~_BV(n)
#define INV(p, n)    p ^= _BV(n)

//переменные для енкодера
unsigned char EncState = 0;  //глобальная переменная u08 -- предыдущее состояние энкодера
unsigned int  EncData = 0;   //глобальная переменная u16 -- счетный регистр энкодера
unsigned char EncDiv = 4;    //коифициент делителя для энкодера
unsigned int TimerCount_ms = 0; // счетчик милисекунд , максимум 65535 мс, это 65 секунд ..Системный таймер


void timer0_init(void)  // Настройка таймера 0 на прерывания каждую 1 милисекунду
{
    //Настройка таймеров
    TIMSK0 = 0b00000010;            // Разрешить прерывание по совподению Т0 
    TCCR0A = 0b00000010;            // сброс при совподенеи OCR0A
    TCCR0B = 0b00000011;               
    OCR0A = 125;                     // 8000000/64=125000, 250000/1000= 125 тактов на 1мс, прерывания каждую милисекунду.
}

unsigned int timer_count_ms_read(void)
{
  return TimerCount_ms;
}

void timer_count_ms_write(unsigned int t)
{
  TimerCount_ms = t;
  return;
}

SIGNAL(TIMER0_COMPA_vect) // прерывания каждую 1ms
{
EncoderScan(); // сканируем енкодер
TimerCount_ms++;
if(TimerCount_ms==65535) TimerCount_ms=0; // обнуляем счетчик при переполнеинии 
}

//функция инициализации
//__________________________________________
void EncoderInit(void)
{
  CLR(DDR_Enc, Pin1_Enc); //вход
  CLR(DDR_Enc, Pin2_Enc);
  SET(PORT_Enc, Pin1_Enc);//вкл подтягивающий резистор
  SET(PORT_Enc, Pin2_Enc);
  sei();

}


void EncoderScan(void)
{
unsigned char New = 0;
 // Берем текущее значение 
  if(PIN_Enc & (1<<Pin1_Enc)) SET(New,0);
  if(PIN_Enc & (1<<Pin2_Enc)) SET(New,1);
// И сравниваем со старым
// Смотря в какую сторону оно поменялось -- увеличиваем
// Или уменьшаем счетный регистр
switch(EncState)
  {
  case 2:
    {
    if(New == 3) EncDiv--;
    if(New == 0) EncDiv++; 
    break;
    }
 
  case 0:
    {
    if(New == 2) EncDiv--;
    if(New == 1) EncDiv++; 
    break;
    }
  case 1:
    {
    if(New == 0) EncDiv--;
    if(New == 3) EncDiv++; 
    break;
    }
  case 3:
    {
    if(New == 1) EncDiv--;
    if(New == 2) EncDiv++; 
    break;
    }
  }
  //делитель тактов енкодера на 4, так как за 1 такт проходят 4 импульса
  //if(EncDiv>=8){EncData++;EncDiv=4;};
  //if(EncDiv==0){EncData--;EncDiv=4;};
  if(EncDiv>=8){EncData++;EncDiv=4;};
  if(EncDiv==0){EncData--;EncDiv=4;};
  if(EncData>=60000){EncData=0;};     // ограничиваем переменную ниже нуля и до 60000
EncState = New;                      // Записываем новое значение Предыдущего состояния
}

unsigned char EncoderRead(void)     //Читаем счетчик энкодера
{
  return EncData;
}

void EncoderWrite(unsigned int a)     // изменяем счетчик энкодера
{
  EncData = a;
}