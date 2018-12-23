/*
Тестовый коментрарий
// Программа - "продвинутый приемник на 5.8ГГц"
// Дисплей - NOKIA 5110 , драйвер использует программный SPI.
// Радио модуль RX5808 из приемника RC305, программа использует аппаратный SPI.
// В настройках проекта обязательно правильно укажите свою тактовую частоту
// Контроллер Atmega328p , тактовая частота 8МГц
// Опорное напряжения для ADC - внешний источник на 3.3 вольта. (Реальное 3.27 вольт, под него и буди вычислять) очень важно чистое питания!
Выход с RX5805 уровень сигнала RSSI: минимум = 500mV(-91dB), максимум 1200mV(+5dB)
3270mV - это 1023 еденицы (ед) ->
3270mV/1023ed = 3.196mV/ed     ->
500mV/3.196 = 156.44ed 
1200mV/3.196 = 375.46ed
-----------------------
Реально измеренное:
Максимальный уровень RSSI: 1267mV
Минимальный уровень RSSI:  500mV-512mV
=> пересчитаем под реальные значения:
500mV/3.196 =  156.44ed 
1267mV/3.196 = 396.46ed 
Вот теперь их и будим использовать:
Уровень АЦП:
156 - это min RSSI, -91dB
396 - это max RSSI, +5dB
396-156 = 240 - это дельта RSSI

Управления  - энкодер с кнопкой
короткое нажатия (менее 500мс)- вход,
длинное нажатия (более 500мс)- выход.


*/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "n3310.h"
#include "spi.h"
#include "adc.c"
#include "encoder.h"
#include "rx5808.h"
#include "picture.h"

#define F_CPU 8000000UL
#define SET(p, n)    p |= _BV(n)
#define CLR(p, n)    p &= ~_BV(n)
#define INV(p, n)    p ^= _BV(n)
//Обозначаем ножку аналогово входу куда подключин пин RSSI от RX5808
#define RSSI_pin       0 //ADC pin 0
#define DDR_RSSI       DDRC
//Обозначаем кнопку на энкодере
#define PORT_Button    PORTC   
#define PIN_Button     PINC
#define DDR_Button     DDRC
#define PinButton      3

//Устанавливаем значения уровня сингала с ADC
#define minRSSI  156
#define maxRSSI  396
#define deltaRSSI  (maxRSSI-minRSSI) //(deltaRSSI = maxRSSI-minRSSI)

unsigned int  masRSSI[100];   // обьявляем масив для хранения уровня сигнала всего диапазона частот
unsigned char buf_Encoder=0;  // предыдущее значения энкодера
unsigned int  eeprom_freg;    // сюда читаем нашу частоту из еепром
unsigned char exit_meny = 0;   
unsigned char pressing =0;       // флаг о нажатии кнопки
unsigned char frequency_step =5; // шаг перестройки сетки частот
unsigned char contrast = 63;     // 0..127

unsigned int eeprom_eemem_freguence EEMEM ;  
unsigned int eeprom_eemem_contrast EEMEM ;  

 //eeprom_write_word(&eeprom_eemem_freguence, 5799); // запись в eeprom
 //eeprom_freg = eeprom_read_word(&eeprom_eemem_freguence); // чтения из eeprom

unsigned char Freg_to_pixel(unsigned char freg);  // перевод значения энкодера в значения столба на экране
void          Button_Init(void);                  // Инцилизация кнопки энкодера
unsigned int ButtonRead(void);                    // Возвращает время нажатия кнопки в милисекундах
unsigned int Read_RSSI(void);                    // Измеряем уровень RSSI и фильтруем его 0...1860
unsigned int Reciver_meny(unsigned int frequency);
unsigned int Spectr_meny(void);

unsigned char Freg_to_pixel(unsigned char freg) //перевод значения энкодера в значения столба на экране
{
     if(freg>100)freg=100;
     unsigned char g=5;       // каждая пятая частота совмещена со следующей
     unsigned char a=0;       // счетчик пикселя на экране
     unsigned char c=0; 
   //  unsigned char frequency_step = 5; // шаг перестройки частоты
     for(unsigned char t=0; t<=100 ;t=t+1) // Переводим значения энкодера в значения столбика на экране
     {
        if(t==g){g=g+5;}else{a++;}
        if(freg == t){c=a;}
     }
   return c;
 };
void Button_Init(void)
{
    DDR_Button  |= (0<<PinButton); // ножку кнопки на ввод
    PORT_Button |= (1<<PinButton); // подключаем подтягивающий резистор
}
unsigned int ButtonRead(void)     // Опрашиваем кнопку
{
unsigned int time_button = 0;
if(pressing)
               {
                 if(PIN_Button &(1<<PinButton)) {if(timer_count_ms_read()<500){ time_button = timer_count_ms_read();pressing=0;}else{pressing=0;}}
                 else {if(pressing!=2){if(timer_count_ms_read()>510){time_button = timer_count_ms_read();pressing=2;} }}
               }
              else  
              {
                 if((~PIN_Button)&(1<<PinButton))  {timer_count_ms_write(0);pressing=1;}
              }
return time_button;
}

unsigned int Read_RSSI(void)               // Измеряем уровень RSSI и фильтруем его, возвращает 0...2400
{
         unsigned int sval=0;
         for ( byte u = 0; u < 10; u++)
         {
             sval = sval + ADC_read(RSSI_pin);
             _delay_ms(5);
         };
              if(sval < (minRSSI*10)){sval=(minRSSI*10);};        // 155 - это значения нуля  RSSI
              sval = sval-(minRSSI*10);                     // Вычисляем дельтаRSSI, тоесть готовый сигнал
  return sval;
}



unsigned int reciver_meny_chanel(unsigned int frequency)
{
    unsigned char CH = 8; // канал частоты
    unsigned char band = 0;    // сетка частоты
    unsigned char CH_32 = frequency_to_CH_32(frequency);   // Номер канала по 32-ух разряднорй шкале
    if(CH_32==0) CH_32 = 1; // если ноль, то не найден канал для этой частоты
    unsigned int buf_freg1 =0;
    EncoderWrite(CH_32-1);
    while(!ButtonRead())
    {
    LcdClear();

    if(EncoderRead()>31) EncoderWrite(31);

    CH_32 = EncoderRead()+1;
    band = CH_32_to_band(CH_32); // определяем сетку
    CH =   CH_32_to_CH_8(CH_32); // определяем канал

    LcdGotoXYFont(2,1); 
    LcdFStr(FONT_2X,(unsigned char*)PSTR("канал"));
    LcdGotoXYFont(0,2); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("CH"));
    LcdChr (FONT_1X, CH+0x30);
   // LcdFStr(FONT_1X,(unsigned char*)PSTR("Сетка "));
    if(band == 0) LcdFStr(FONT_1X,(unsigned char*)PSTR(" A  "));
    if(band == 1) LcdFStr(FONT_1X,(unsigned char*)PSTR(" B  "));
    if(band == 2) LcdFStr(FONT_1X,(unsigned char*)PSTR(" E  "));
    if(band == 3) LcdFStr(FONT_1X,(unsigned char*)PSTR(" F  "));
    frequency = CH_32_to_frequency(CH_32);

    LcdString_4(frequency);
     if(buf_freg1 != frequency) {prog_freg(frequency);buf_freg1 = frequency;}
    
    LcdFStr(FONT_1X,(unsigned char*)PSTR("МГц"));
    //______________Отрисовка уровня сигнала_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("Уровень: "));
     unsigned int rssi = Read_RSSI();
     unsigned char rssi_dB = rssi_db((rssi/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //рисуем знак минус перед децибелами, если он нужен
     {
        CLR(rssi_dB,7);                              // Если знак "-" есть, то его стираем
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("дБ"));
      unsigned int a = (deltaRSSI*10)/79;// a - коифициент, 79pix - рабочие поле полоски 
      unsigned char pixel1 = (rssi/a);     
      // кординаты начальной точки X,Y, высота, ширина, тип пикселя. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // Линия для RSSI верх
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // Линия для RSSI низ  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // Линия для RSSI  правый бок
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // Линия для RSSI  левый бок
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // Рисуем сам уровень RSSI
    //________________________________________________________________________________
    LcdUpdate();
    LcdClear();

      unsigned int Button_time = ButtonRead();
        if(Button_time)
        {
            if(Button_time<500) {return frequency;} // одно нажатия, просто выходим
            if(Button_time>500) {exit_meny = 1; return frequency; }// длинное нажатия, выходим в главное меню
        }
   }
   return frequency;
}

//==============================================================================
//========================= РЕЖИМ - СКАНЕР =====================================
//______________________________________________________________________________  
unsigned int Scaner_meny(void)
{
      unsigned int frequency =0;
      unsigned int sval=0;
      unsigned int rssi=0;
      unsigned int maxi=0;
      unsigned int scan=1;
      
  while(1)
  {
      if(scan)
    {
      frequency =0;
       sval=0;
       rssi=0;
       maxi=0;
       scan=0;
     // ____________________________________________________
     // _______________ЭКРАН ПОИСКА_________________________
      for ( unsigned char i = 0; i < 249; i++)
      {
      prog_freg(5501+2*i);
      LcdGotoXYFont(1,1);
      LcdFStr(FONT_2X,(unsigned char*)PSTR("сканер"));
      LcdGotoXYFont(0,2);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("поиск  "));
      LcdString_4(5501+2*i);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("МГц"));
      sval = Read_RSSI();
      /*
      _delay_ms(30);
      sval = ADC_read(RSSI_pin);
      sval = sval*10;
      if(sval < (minRSSI*10)){sval=(minRSSI*10);};        // 155 - это значения нуля  RSSI
      sval = sval-(minRSSI*10);   
      */   
      if (sval > rssi){ rssi=sval; maxi=i ; }; 

  //______________Отрисовка уровня сигнала_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("Уровень: "));
     unsigned char rssi_dB = rssi_db((sval/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //рисуем знак минус перед децибелами, если он нужен
     {
        CLR(rssi_dB,7);                              // Если знак "-" есть, то его стираем
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("дБ"));
      unsigned int a = (deltaRSSI*10)/79;// a - коифициент, 79pix - рабочие поле полоски 
      unsigned char pixel1 = sval/a;     
      // кординаты начальной точки X,Y, высота, ширина, тип пикселя. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // Линия для RSSI верх
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // Линия для RSSI низ  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // Линия для RSSI  правый бок
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // Линия для RSSI  левый бок
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // Рисуем сам уровень RSSI
    //________________________________________________________________________________
     LcdUpdate();  
     LcdClear();   

      if(ButtonRead()>500) {exit_meny = 1; return frequency; } 
     }
   }

 // ____________________________________________________
 // _______________ЭКРАН ОЖИДАНИЯ_______________________
   if(rssi>(minRSSI+20)) // 20 (-82dB) минимальный порог для уровня
   {
     frequency = 5501+2*maxi;
     prog_freg(frequency);
      LcdGotoXYFont(1,1);
      LcdFStr(FONT_2X,(unsigned char*)PSTR("сканер"));
      LcdGotoXYFont(0,2);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("найден "));
      LcdString_4(frequency);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("МГц"));
    }
    else
    {
      LcdGotoXYFont(1,1);
      LcdFStr(FONT_2X,(unsigned char*)PSTR("сканер"));
      LcdGotoXYFont(2,2);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("не найдено"));
    }
  
   //______________Отрисовка уровня сигнала_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("Уровень: "));
     unsigned int rssi = Read_RSSI();
     unsigned char rssi_dB = rssi_db((rssi/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //рисуем знак минус перед децибелами, если он нужен
     {
        CLR(rssi_dB,7);                              // Если знак "-" есть, то его стираем
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("дБ"));
      unsigned int a = (deltaRSSI*10)/79;// a - коифициент, 79pix - рабочие поле полоски 
      unsigned char pixel1 = rssi/a;     
      // кординаты начальной точки X,Y, высота, ширина, тип пикселя. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // Линия для RSSI верх
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // Линия для RSSI низ  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // Линия для RSSI  правый бок
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // Линия для RSSI  левый бок
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // Рисуем сам уровень RSSI
    //________________________________________________________________________________
     LcdUpdate();  
     LcdClear(); 

      unsigned int Button_time = ButtonRead();
        if(Button_time)
        {
            if(Button_time<500) {scan = 1;} // если нажата кнопка меньше 500ms, то запускаем сканер
            if(Button_time>500) {exit_meny = 1; return frequency; }// // если нажата кнопка больше 1 секунды, то выходим
        }

  }
   return frequency;
}

//==============================================================================
//========================= РЕЖИМ - ПРИЕМНИК ===================================
//______________________________________________________________________________ 
unsigned int Reciver_meny(unsigned int frequency) 
{
    exit_meny = 0;
    unsigned int buf_freg = frequency; 
    EncoderWrite((6000-frequency)/frequency_step); // записываем в энкодер текущую частоту
    prog_freg(frequency);
    while(!exit_meny)
    {
    LcdClear();

    if(EncoderRead()>500/frequency_step) EncoderWrite(500/frequency_step); // ограничиваем счетчик энкодера в зависимости от шага настройки
    frequency = (6000-(frequency_step*EncoderRead()));
    if(buf_freg != frequency) {prog_freg(frequency);buf_freg = frequency;}// если изменили частоту, то устанавливаем ее в приемник
    
    LcdGotoXYFont(0,1); 
    LcdFStr(FONT_2X,(unsigned char*)PSTR("частотa"));
    LcdGotoXYFont(0,2);
    LcdString_4(frequency);
    LcdFStr(FONT_1X,(unsigned char*)PSTR(" МГц "));
    unsigned char u =  frequency_to_CH_32(frequency);

     if(u)
     {
     LcdFStr(FONT_1X,(unsigned char*)PSTR("CH"));
     unsigned char k  = (CH_32_to_CH_8(u))+0x30;
     LcdChr (FONT_1X, k);
     unsigned char band = CH_32_to_band(u);
     if(band == 0) LcdFStr(FONT_1X,(unsigned char*)PSTR(" A"));
      if(band == 1) LcdFStr(FONT_1X,(unsigned char*)PSTR(" B"));
         if(band == 2) LcdFStr(FONT_1X,(unsigned char*)PSTR(" E"));
            if(band == 3) LcdFStr(FONT_1X,(unsigned char*)PSTR(" F"));
    }
    //______________Отрисовка уровня сигнала_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("Уровень: "));
     unsigned int rssi = Read_RSSI();
     unsigned char rssi_dB = rssi_db((rssi/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //рисуем знак минус перед децибелами, если он нужен
     {
        CLR(rssi_dB,7);                              // Если знак "-" есть, то его стираем
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("дБ"));
      unsigned int a = (deltaRSSI*10)/79;// a - коифициент, 79pix - рабочие поле полоски 
      unsigned char pixel1 = (rssi/a);     
      // кординаты начальной точки X,Y, высота, ширина, тип пикселя. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // Линия для RSSI верх
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // Линия для RSSI низ  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // Линия для RSSI  правый бок
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // Линия для RSSI  левый бок
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // Рисуем сам уровень RSSI
    //________________________________________________________________________________
    LcdUpdate();
    LcdClear();

     unsigned int Button_time = ButtonRead();
        if(Button_time)
        {
            if(Button_time>500) {exit_meny = 1; return frequency;} // если нажата кнопка меньше 500ms, то просто выходим
            if(Button_time<500) {frequency = reciver_meny_chanel(frequency); EncoderWrite((6000-frequency)/frequency_step); }// // если нажата кнопка больше 1 секунды, то переходим в режим Reciver
        }
   }
   return frequency;
}
//==============================================================================
//========================= РЕЖИМ - СПЕКТР =====================================
//______________________________________________________________________________  
unsigned int Spectr_meny(void)
{   
    unsigned int frequency = 0;
    LcdClear();
    LcdGotoXYFont(4,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("МГц  "));
    LcdGotoXYFont(12,0);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("дБ")); 
    LcdLine ( 0,9,84,9,   PIXEL_ON);                     // Рисуем верхнию линию
    LcdLine ( 0,47,84,47, PIXEL_ON);                     // Рисуем нижнию  линию
    LcdUpdate();
    for(unsigned char i=0;i<101;i++){ masRSSI[i]=0;}; // инцилизируем масив 

  while(exit_meny!=1) // Рисования спектра 
  {
     unsigned char column = 0;                           // обнуляем счетчик линий
     unsigned char b=5;                                  // каждая 5-я линия будит совмещена с соседней частотой  
     for ( byte i = 0; i < 100; i++)                     
     {
      prog_freg(5500+5*i);                               // диапазон сканирования 5500 - 6000 МГц
      _delay_ms(30);
              unsigned int RSSI_adc =  Read_RSSI();      // Измеряем RSSI
              unsigned int v = (deltaRSSI*10)/36;
              unsigned int spec = RSSI_adc/v;           
              masRSSI[i]=RSSI_adc;                       // сохраняем уровень сигнала в масив
              if(EncoderRead()>100) EncoderWrite(100);   // ограничиваем счетчик энкодера до 100
              unsigned char rssi_dB_mas = rssi_db(masRSSI[EncoderRead()]/10,deltaRSSI); // Переводим измеренный уровень в децибелы
     //______________Рисуем знак уровня сигнала____________________________________________________
     if(rssi_dB_mas & (1<<7)) //рисуем знак минус перед децибелами, если он нужен
     {
        CLR(rssi_dB_mas,7);                              // Если знак "-" есть, то его стираем
        LcdGotoXYFont(9,0);
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdGotoXYFont(9,0);
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
     //______________Рисуем сам спектр____________________________________________________________
     if(i==b){b=b+5;}else{column++;}                       // тут считаем каждую пятую линию, и пропускаем инкримент счетчика линий
     LcdSingleBar( column ,46, spec, 1, PIXEL_ON  );       // Рисуем вертикальную развертку спектра
     LcdSingleBar( column+1,46, 37 , 3, PIXEL_OFF );       // Затираем перед стоящию линию спектра
     //______________Рисуем параметры в загаловке__________________________________________________
     LcdGotoXYFont(0,0);
     LcdString_4(5500+5*EncoderRead());                          // Место для вывода частоты в заголовке 
     LcdGotoXYFont(10,0);
     LcdString_2(rssi_dB_mas);                             // Место для вывода RSSI в децибелах в заголовке
     //______________Рисуем подвижную линию________________________________________________________
    // if(EncoderRead()>100) EncoderWrite(100); 
     if(buf_Encoder>100) buf_Encoder=100; 
     if(buf_Encoder!=EncoderRead())                              // Если мы покрутили энкодер, то отрисовываем новое значения линии а старую затераем
     {
        unsigned int m = (deltaRSSI*10)/36;                       // Вычисляем уровень сигнала в пикселях(delta) 37pix-рабочие поле в пикселях
        unsigned int db_unsigned = masRSSI[buf_Encoder]/m;                
        LcdSingleBar(Freg_to_pixel(buf_Encoder), 46, 37, 1, PIXEL_OFF);           // Стираем весь столбик 
        LcdSingleBar(Freg_to_pixel(buf_Encoder), 46, db_unsigned, 1,PIXEL_ON);    // На этом месте рисуем уровень из буфера масива RSSI
        buf_Encoder=EncoderRead();                                                      // Обновляем буфер енкодера
        LcdSingleBar(Freg_to_pixel(EncoderRead()),46, 38, 1, PIXEL_ON  );               // Рисуем новую линию, куда накрутили енкодером
     }
     LcdUpdate();
    unsigned int Button_time = ButtonRead();
    //_delay_ms(200);
        if(Button_time)
        {
            if(Button_time>500) {exit_meny = 1; return 5500+5*EncoderRead(); } // 
            if(Button_time<500) {exit_meny = 1;frequency = Reciver_meny(5500+5*EncoderRead());return frequency;}// 
        }
    }
  }
  return frequency;
}

void meny_setup_step(void) // Меню настройки шага изминения частоты
{
    while(!ButtonRead())
    {
        //Устанавливаем шаг настройки сетки
    if(EncoderRead()>4) EncoderWrite(4); 
    if(EncoderRead()==0) frequency_step = 1;
    if(EncoderRead()==1) frequency_step = 2;
    if(EncoderRead()==2) frequency_step = 5;
    if(EncoderRead()==3) frequency_step = 10;

    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  настройки: "));
    LcdGotoXYFont(0,2);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("шаг     "));
    LcdString_no_nul_2(frequency_step);
    LcdFStr(FONT_1X,(unsigned char*)PSTR(" МГц"));
    LcdSingleBar(0,24,9,84, PIXEL_XOR);
    LcdUpdate();
    LcdClear();
    }
}

unsigned int meny_setup_frecyency(unsigned int frequency) // меню настройки частоты, которая будит устанавливаться по дифолту после выключения
{
  EncoderWrite((6000-frequency)/frequency_step); // записываем в энкодер текущую частоту
  while(!ButtonRead())
    {
    if(EncoderRead()>500/frequency_step) EncoderWrite(500/frequency_step); // ограничиваем счетчик энкодера в зависимости от шага настройки
    frequency = (6000-(frequency_step*EncoderRead()));
    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  настройки: "));
    LcdGotoXYFont(0,3); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("частот "));
    LcdString_4(frequency);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("МГц"));
    LcdSingleBar(0,24+8,9,84, PIXEL_XOR);
    LcdUpdate();
    LcdClear();
    }
    return frequency;
}

unsigned char meny_setup_contrast(void) // меню настройки частоты, которая будит устанавливаться по дифолту после выключения
{ 
  EncoderWrite(contrast);
  while(!ButtonRead())
    {
    if(EncoderRead()>127) EncoderWrite(127); // ограничиваем счетчик энкодера в зависимости от шага настройки
    if(EncoderRead()<57) EncoderWrite(57);
    contrast = EncoderRead();
    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  настройки: "));
     LcdGotoXYFont(0,4); 
     LcdFStr(FONT_1X,(unsigned char*)PSTR("контраст "));
     LcdString_4(contrast);
     LcdSingleBar(0,24+8+8, 9,84, PIXEL_XOR);

    LcdContrast(contrast);
    LcdUpdate();
    LcdClear();

    }
    return contrast;
}
//==============================================================================
//========================= РЕЖИМ - НАСТРОЙКИ ==================================
//______________________________________________________________________________  
void meny_setup(void)
{
  unsigned int eeprom_freg = eeprom_read_word(&eeprom_eemem_freguence);   // Читаем наши настройки из eeprom
  contrast = eeprom_read_word(&eeprom_eemem_contrast);                 
  exit_meny = 0;
    while(!exit_meny)
    {
    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  настройки: "));
    LcdGotoXYFont(0,2);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("шаг     "));
    LcdString_no_nul_2(frequency_step);
    LcdFStr(FONT_1X,(unsigned char*)PSTR(" МГц"));

     LcdGotoXYFont(0,3); 
     LcdFStr(FONT_1X,(unsigned char*)PSTR("частот "));
     LcdString_4(eeprom_freg);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("МГц"));

     LcdGotoXYFont(0,4); 
     LcdFStr(FONT_1X,(unsigned char*)PSTR("контраст "));
     LcdString_no_nul_2(contrast);

     if(EncoderRead()>2) EncoderWrite(2);

     unsigned int Button_time = ButtonRead(); // читаем время нажатия кнопки
     unsigned char pressed_button=0;
     if(Button_time)
     {
       if(Button_time>500) {exit_meny =1;} // Если длинное нажатия, то выходим в меню настроек
       if(Button_time<500){ pressed_button=1;} // если короткое нажатия, то выставляем флаг
     }
     switch(EncoderRead())
     {
    case 0: LcdSingleBar(0,24,     9,84, PIXEL_XOR); if(pressed_button){meny_setup_step();EncoderWrite(0);} break; 
    case 1: LcdSingleBar(0,24+8,   9,84, PIXEL_XOR); if(pressed_button){eeprom_freg = meny_setup_frecyency(eeprom_freg); EncoderWrite(1);} break;  
    case 2: LcdSingleBar(0,24+8+8, 9,84, PIXEL_XOR); if(pressed_button){contrast =  meny_setup_contrast(); EncoderWrite(2);} break; 
     }

    LcdUpdate();
    LcdClear();
    }
     eeprom_write_word(&eeprom_eemem_freguence, eeprom_freg); // сохраняем наши настройки в eeprom при выходи
     eeprom_write_word(&eeprom_eemem_contrast, contrast);
}


int main(void)
{ 
    ADC_Init();
    LcdInit();
    EncoderInit();
    timer0_init();    // инцилизация таймера для энкодера
    RX5808_Init();
    Button_Init();
    DDR_RSSI |= (0<<RSSI_pin);
    LcdClear();
    contrast = eeprom_read_word(&eeprom_eemem_contrast); 
    if(contrast>127) contrast = 127;
    if(contrast<55) contrast =  55;
    LcdContrast(contrast);
    LcdImage(Picture);    // Пуляем заставку
    LcdUpdate();
    _delay_ms(1300);
    LcdClear();
    LcdUpdate();

    eeprom_freg = eeprom_read_word(&eeprom_eemem_freguence); 
while(1)
{
     exit_meny = 0;
     unsigned int frequency = eeprom_freg;
     LcdGotoXYFont(3,0);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("ПРИЕМНИК"));
     LcdGotoXYFont(4,1);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("СКАНЕР"));
     LcdGotoXYFont(4,2);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("СПЕКТР"));
     LcdSingleBar(3,47,1 ,83, PIXEL_ON );    // горизонтальная линия нижняя
     LcdGotoXYFont(1,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("Уст: "));
     prog_freg(eeprom_freg ); 
     LcdString_4(eeprom_freg );
     LcdFStr(FONT_1X,(unsigned char*)PSTR("МГц"));

      unsigned int pixel = Read_RSSI()/23;     // 23 - коифициент, (341(maxRSSIadc)*10)-(155(minRSSIadc)*10)/79pix  = 23   
      LcdSingleBar(3,41,1 ,80, PIXEL_ON );     // Линия для RSSI
      LcdSingleBar(83,47, 7 , 1, PIXEL_ON );   // Ветикальная короткая линия RSSI с право
       LcdSingleBar(2,47,7 ,1, PIXEL_ON );     // Линия для RSSI  левый бок
      LcdSingleBar(3,46,5,pixel,PIXEL_ON);     // Рисуем сам уровень RSSI
     frequency =0;
     if(EncoderRead()>2) EncoderWrite(2);

     unsigned int Button_time = ButtonRead(); // читаем время нажатия кнопки
     unsigned char pressed_button=0;
     if(Button_time)
     {
       if(Button_time>500) {meny_setup();eeprom_freg = eeprom_read_word(&eeprom_eemem_freguence);} // Если длинное нажатия, то выходим в меню настроек
       if(Button_time<500){ pressed_button=1;} // если короткое нажатия, то выставляем флаг
     }
     switch(EncoderRead())
     {
    case 0: LcdSingleBar(4,8,  9,76, PIXEL_XOR); if(pressed_button) {eeprom_freg = Reciver_meny(eeprom_freg);  } break; 
    case 1: LcdSingleBar(4,16, 9,76, PIXEL_XOR); if(pressed_button) {frequency =   Scaner_meny();exit_meny = 0;} break;  
    case 2: LcdSingleBar(4,24, 9,76, PIXEL_XOR); if(pressed_button) {frequency =   Spectr_meny();exit_meny = 0;} break; 
     }

     if(frequency>5000) eeprom_freg = frequency;
    LcdUpdate();
     LcdClear();

}}
    






