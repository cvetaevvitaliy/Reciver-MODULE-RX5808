
#include <avr/io.h>

void ADC_Init(void);
unsigned int ADC_read(unsigned char adc_mux);
/* Функция инициализация АЦП */
void ADC_Init()
{
 ADCSRA |= (1 << ADEN) // Включаем АЦП
 |(1 << ADPS1)|(1 << ADPS0);    // устанавливаем предделитель преобразователя на 8
 ADMUX |= (0 << REFS1)|(0 << REFS0) //выставляем опорное напряжение
 |(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3); // снимать сигнал будем с  входа PC0 
/*
REFS1 REFS0   // Опорное напряжения ADC
  0     0    -Внешний опорный ИОН
  0     1    -Питания напряжения питания AVcc
  1     0    -Резерв
  1     1    -Внутренний ИОН 1.1в с внешним конденсатором на AREF пине
 
ADPS2 ADPS1 ADPS0  // предделитель ADC
  0     0     0      -2
  0     0     1      -2
  0     1     0      -4
  0     1     1      -8 
  1     0     0      -16 
  1     0     1      -32
  1     1     0      -64
  1     1     1      -128

*/
}

unsigned int ADC_read(unsigned char adc_mux) 
{
  ADMUX |= (0b00001111 & adc_mux); // задаем канал мультиплексора
  ADCSRA |= (1 << ADSC);
  while ((ADCSRA & (1 << ADIF)) == 0);
  unsigned int a = (ADCL|ADCH << 8);
  return a;
}
