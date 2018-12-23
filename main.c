/*
�������� �����������
// ��������� - "����������� �������� �� 5.8���"
// ������� - NOKIA 5110 , ������� ���������� ����������� SPI.
// ����� ������ RX5808 �� ��������� RC305, ��������� ���������� ���������� SPI.
// � ���������� ������� ����������� ��������� ������� ���� �������� �������
// ���������� Atmega328p , �������� ������� 8���
// ������� ���������� ��� ADC - ������� �������� �� 3.3 ������. (�������� 3.27 �����, ��� ���� � ���� ���������) ����� ����� ������ �������!
����� � RX5805 ������� ������� RSSI: ������� = 500mV(-91dB), �������� 1200mV(+5dB)
3270mV - ��� 1023 ������� (��) ->
3270mV/1023ed = 3.196mV/ed     ->
500mV/3.196 = 156.44ed 
1200mV/3.196 = 375.46ed
-----------------------
������� ����������:
������������ ������� RSSI: 1267mV
����������� ������� RSSI:  500mV-512mV
=> ����������� ��� �������� ��������:
500mV/3.196 =  156.44ed 
1267mV/3.196 = 396.46ed 
��� ������ �� � ����� ������������:
������� ���:
156 - ��� min RSSI, -91dB
396 - ��� max RSSI, +5dB
396-156 = 240 - ��� ������ RSSI

����������  - ������� � �������
�������� ������� (����� 500��)- ����,
������� ������� (����� 500��)- �����.


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
//���������� ����� ��������� ����� ���� ��������� ��� RSSI �� RX5808
#define RSSI_pin       0 //ADC pin 0
#define DDR_RSSI       DDRC
//���������� ������ �� ��������
#define PORT_Button    PORTC   
#define PIN_Button     PINC
#define DDR_Button     DDRC
#define PinButton      3

//������������� �������� ������ ������� � ADC
#define minRSSI  156
#define maxRSSI  396
#define deltaRSSI  (maxRSSI-minRSSI) //(deltaRSSI = maxRSSI-minRSSI)

unsigned int  masRSSI[100];   // ��������� ����� ��� �������� ������ ������� ����� ��������� ������
unsigned char buf_Encoder=0;  // ���������� �������� ��������
unsigned int  eeprom_freg;    // ���� ������ ���� ������� �� ������
unsigned char exit_meny = 0;   
unsigned char pressing =0;       // ���� � ������� ������
unsigned char frequency_step =5; // ��� ����������� ����� ������
unsigned char contrast = 63;     // 0..127

unsigned int eeprom_eemem_freguence EEMEM ;  
unsigned int eeprom_eemem_contrast EEMEM ;  

 //eeprom_write_word(&eeprom_eemem_freguence, 5799); // ������ � eeprom
 //eeprom_freg = eeprom_read_word(&eeprom_eemem_freguence); // ������ �� eeprom

unsigned char Freg_to_pixel(unsigned char freg);  // ������� �������� �������� � �������� ������ �� ������
void          Button_Init(void);                  // ����������� ������ ��������
unsigned int ButtonRead(void);                    // ���������� ����� ������� ������ � ������������
unsigned int Read_RSSI(void);                    // �������� ������� RSSI � ��������� ��� 0...1860
unsigned int Reciver_meny(unsigned int frequency);
unsigned int Spectr_meny(void);

unsigned char Freg_to_pixel(unsigned char freg) //������� �������� �������� � �������� ������ �� ������
{
     if(freg>100)freg=100;
     unsigned char g=5;       // ������ ����� ������� ��������� �� ���������
     unsigned char a=0;       // ������� ������� �� ������
     unsigned char c=0; 
   //  unsigned char frequency_step = 5; // ��� ����������� �������
     for(unsigned char t=0; t<=100 ;t=t+1) // ��������� �������� �������� � �������� �������� �� ������
     {
        if(t==g){g=g+5;}else{a++;}
        if(freg == t){c=a;}
     }
   return c;
 };
void Button_Init(void)
{
    DDR_Button  |= (0<<PinButton); // ����� ������ �� ����
    PORT_Button |= (1<<PinButton); // ���������� ������������� ��������
}
unsigned int ButtonRead(void)     // ���������� ������
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

unsigned int Read_RSSI(void)               // �������� ������� RSSI � ��������� ���, ���������� 0...2400
{
         unsigned int sval=0;
         for ( byte u = 0; u < 10; u++)
         {
             sval = sval + ADC_read(RSSI_pin);
             _delay_ms(5);
         };
              if(sval < (minRSSI*10)){sval=(minRSSI*10);};        // 155 - ��� �������� ����  RSSI
              sval = sval-(minRSSI*10);                     // ��������� ������RSSI, ������ ������� ������
  return sval;
}



unsigned int reciver_meny_chanel(unsigned int frequency)
{
    unsigned char CH = 8; // ����� �������
    unsigned char band = 0;    // ����� �������
    unsigned char CH_32 = frequency_to_CH_32(frequency);   // ����� ������ �� 32-�� ���������� �����
    if(CH_32==0) CH_32 = 1; // ���� ����, �� �� ������ ����� ��� ���� �������
    unsigned int buf_freg1 =0;
    EncoderWrite(CH_32-1);
    while(!ButtonRead())
    {
    LcdClear();

    if(EncoderRead()>31) EncoderWrite(31);

    CH_32 = EncoderRead()+1;
    band = CH_32_to_band(CH_32); // ���������� �����
    CH =   CH_32_to_CH_8(CH_32); // ���������� �����

    LcdGotoXYFont(2,1); 
    LcdFStr(FONT_2X,(unsigned char*)PSTR("�����"));
    LcdGotoXYFont(0,2); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("CH"));
    LcdChr (FONT_1X, CH+0x30);
   // LcdFStr(FONT_1X,(unsigned char*)PSTR("����� "));
    if(band == 0) LcdFStr(FONT_1X,(unsigned char*)PSTR(" A  "));
    if(band == 1) LcdFStr(FONT_1X,(unsigned char*)PSTR(" B  "));
    if(band == 2) LcdFStr(FONT_1X,(unsigned char*)PSTR(" E  "));
    if(band == 3) LcdFStr(FONT_1X,(unsigned char*)PSTR(" F  "));
    frequency = CH_32_to_frequency(CH_32);

    LcdString_4(frequency);
     if(buf_freg1 != frequency) {prog_freg(frequency);buf_freg1 = frequency;}
    
    LcdFStr(FONT_1X,(unsigned char*)PSTR("���"));
    //______________��������� ������ �������_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("�������: "));
     unsigned int rssi = Read_RSSI();
     unsigned char rssi_dB = rssi_db((rssi/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //������ ���� ����� ����� ����������, ���� �� �����
     {
        CLR(rssi_dB,7);                              // ���� ���� "-" ����, �� ��� �������
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("��"));
      unsigned int a = (deltaRSSI*10)/79;// a - ����������, 79pix - ������� ���� ������� 
      unsigned char pixel1 = (rssi/a);     
      // ��������� ��������� ����� X,Y, ������, ������, ��� �������. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // ����� ��� RSSI ����
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // ����� ��� RSSI ���  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ������ ���
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ����� ���
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // ������ ��� ������� RSSI
    //________________________________________________________________________________
    LcdUpdate();
    LcdClear();

      unsigned int Button_time = ButtonRead();
        if(Button_time)
        {
            if(Button_time<500) {return frequency;} // ���� �������, ������ �������
            if(Button_time>500) {exit_meny = 1; return frequency; }// ������� �������, ������� � ������� ����
        }
   }
   return frequency;
}

//==============================================================================
//========================= ����� - ������ =====================================
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
     // _______________����� ������_________________________
      for ( unsigned char i = 0; i < 249; i++)
      {
      prog_freg(5501+2*i);
      LcdGotoXYFont(1,1);
      LcdFStr(FONT_2X,(unsigned char*)PSTR("������"));
      LcdGotoXYFont(0,2);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("�����  "));
      LcdString_4(5501+2*i);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("���"));
      sval = Read_RSSI();
      /*
      _delay_ms(30);
      sval = ADC_read(RSSI_pin);
      sval = sval*10;
      if(sval < (minRSSI*10)){sval=(minRSSI*10);};        // 155 - ��� �������� ����  RSSI
      sval = sval-(minRSSI*10);   
      */   
      if (sval > rssi){ rssi=sval; maxi=i ; }; 

  //______________��������� ������ �������_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("�������: "));
     unsigned char rssi_dB = rssi_db((sval/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //������ ���� ����� ����� ����������, ���� �� �����
     {
        CLR(rssi_dB,7);                              // ���� ���� "-" ����, �� ��� �������
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("��"));
      unsigned int a = (deltaRSSI*10)/79;// a - ����������, 79pix - ������� ���� ������� 
      unsigned char pixel1 = sval/a;     
      // ��������� ��������� ����� X,Y, ������, ������, ��� �������. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // ����� ��� RSSI ����
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // ����� ��� RSSI ���  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ������ ���
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ����� ���
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // ������ ��� ������� RSSI
    //________________________________________________________________________________
     LcdUpdate();  
     LcdClear();   

      if(ButtonRead()>500) {exit_meny = 1; return frequency; } 
     }
   }

 // ____________________________________________________
 // _______________����� ��������_______________________
   if(rssi>(minRSSI+20)) // 20 (-82dB) ����������� ����� ��� ������
   {
     frequency = 5501+2*maxi;
     prog_freg(frequency);
      LcdGotoXYFont(1,1);
      LcdFStr(FONT_2X,(unsigned char*)PSTR("������"));
      LcdGotoXYFont(0,2);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("������ "));
      LcdString_4(frequency);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("���"));
    }
    else
    {
      LcdGotoXYFont(1,1);
      LcdFStr(FONT_2X,(unsigned char*)PSTR("������"));
      LcdGotoXYFont(2,2);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("�� �������"));
    }
  
   //______________��������� ������ �������_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("�������: "));
     unsigned int rssi = Read_RSSI();
     unsigned char rssi_dB = rssi_db((rssi/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //������ ���� ����� ����� ����������, ���� �� �����
     {
        CLR(rssi_dB,7);                              // ���� ���� "-" ����, �� ��� �������
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("��"));
      unsigned int a = (deltaRSSI*10)/79;// a - ����������, 79pix - ������� ���� ������� 
      unsigned char pixel1 = rssi/a;     
      // ��������� ��������� ����� X,Y, ������, ������, ��� �������. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // ����� ��� RSSI ����
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // ����� ��� RSSI ���  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ������ ���
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ����� ���
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // ������ ��� ������� RSSI
    //________________________________________________________________________________
     LcdUpdate();  
     LcdClear(); 

      unsigned int Button_time = ButtonRead();
        if(Button_time)
        {
            if(Button_time<500) {scan = 1;} // ���� ������ ������ ������ 500ms, �� ��������� ������
            if(Button_time>500) {exit_meny = 1; return frequency; }// // ���� ������ ������ ������ 1 �������, �� �������
        }

  }
   return frequency;
}

//==============================================================================
//========================= ����� - �������� ===================================
//______________________________________________________________________________ 
unsigned int Reciver_meny(unsigned int frequency) 
{
    exit_meny = 0;
    unsigned int buf_freg = frequency; 
    EncoderWrite((6000-frequency)/frequency_step); // ���������� � ������� ������� �������
    prog_freg(frequency);
    while(!exit_meny)
    {
    LcdClear();

    if(EncoderRead()>500/frequency_step) EncoderWrite(500/frequency_step); // ������������ ������� �������� � ����������� �� ���� ���������
    frequency = (6000-(frequency_step*EncoderRead()));
    if(buf_freg != frequency) {prog_freg(frequency);buf_freg = frequency;}// ���� �������� �������, �� ������������� �� � ��������
    
    LcdGotoXYFont(0,1); 
    LcdFStr(FONT_2X,(unsigned char*)PSTR("������a"));
    LcdGotoXYFont(0,2);
    LcdString_4(frequency);
    LcdFStr(FONT_1X,(unsigned char*)PSTR(" ��� "));
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
    //______________��������� ������ �������_________________________________________
     LcdGotoXYFont(0,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("�������: "));
     unsigned int rssi = Read_RSSI();
     unsigned char rssi_dB = rssi_db((rssi/10),deltaRSSI);
      if(rssi_dB & (1<<7)) //������ ���� ����� ����� ����������, ���� �� �����
     {
        CLR(rssi_dB,7);                              // ���� ���� "-" ����, �� ��� �������
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
      LcdString_2(rssi_dB);
      LcdFStr(FONT_1X,(unsigned char*)PSTR("��"));
      unsigned int a = (deltaRSSI*10)/79;// a - ����������, 79pix - ������� ���� ������� 
      unsigned char pixel1 = (rssi/a);     
      // ��������� ��������� ����� X,Y, ������, ������, ��� �������. 
      LcdSingleBar(0,41,1 ,84, PIXEL_ON );     // ����� ��� RSSI ����
      LcdSingleBar(0,47,1 ,84, PIXEL_ON );     // ����� ��� RSSI ���  
      LcdSingleBar(83,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ������ ���
      LcdSingleBar(0,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ����� ���
      LcdSingleBar(0,46,6,pixel1,PIXEL_ON);    // ������ ��� ������� RSSI
    //________________________________________________________________________________
    LcdUpdate();
    LcdClear();

     unsigned int Button_time = ButtonRead();
        if(Button_time)
        {
            if(Button_time>500) {exit_meny = 1; return frequency;} // ���� ������ ������ ������ 500ms, �� ������ �������
            if(Button_time<500) {frequency = reciver_meny_chanel(frequency); EncoderWrite((6000-frequency)/frequency_step); }// // ���� ������ ������ ������ 1 �������, �� ��������� � ����� Reciver
        }
   }
   return frequency;
}
//==============================================================================
//========================= ����� - ������ =====================================
//______________________________________________________________________________  
unsigned int Spectr_meny(void)
{   
    unsigned int frequency = 0;
    LcdClear();
    LcdGotoXYFont(4,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("���  "));
    LcdGotoXYFont(12,0);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("��")); 
    LcdLine ( 0,9,84,9,   PIXEL_ON);                     // ������ ������� �����
    LcdLine ( 0,47,84,47, PIXEL_ON);                     // ������ ������  �����
    LcdUpdate();
    for(unsigned char i=0;i<101;i++){ masRSSI[i]=0;}; // ������������ ����� 

  while(exit_meny!=1) // ��������� ������� 
  {
     unsigned char column = 0;                           // �������� ������� �����
     unsigned char b=5;                                  // ������ 5-� ����� ����� ��������� � �������� ��������  
     for ( byte i = 0; i < 100; i++)                     
     {
      prog_freg(5500+5*i);                               // �������� ������������ 5500 - 6000 ���
      _delay_ms(30);
              unsigned int RSSI_adc =  Read_RSSI();      // �������� RSSI
              unsigned int v = (deltaRSSI*10)/36;
              unsigned int spec = RSSI_adc/v;           
              masRSSI[i]=RSSI_adc;                       // ��������� ������� ������� � �����
              if(EncoderRead()>100) EncoderWrite(100);   // ������������ ������� �������� �� 100
              unsigned char rssi_dB_mas = rssi_db(masRSSI[EncoderRead()]/10,deltaRSSI); // ��������� ���������� ������� � ��������
     //______________������ ���� ������ �������____________________________________________________
     if(rssi_dB_mas & (1<<7)) //������ ���� ����� ����� ����������, ���� �� �����
     {
        CLR(rssi_dB_mas,7);                              // ���� ���� "-" ����, �� ��� �������
        LcdGotoXYFont(9,0);
        LcdFStr(FONT_1X,(unsigned char*)PSTR("-"));
     }
     else
     {
        LcdGotoXYFont(9,0);
        LcdFStr(FONT_1X,(unsigned char*)PSTR(" "));
     };
     //______________������ ��� ������____________________________________________________________
     if(i==b){b=b+5;}else{column++;}                       // ��� ������� ������ ����� �����, � ���������� ��������� �������� �����
     LcdSingleBar( column ,46, spec, 1, PIXEL_ON  );       // ������ ������������ ��������� �������
     LcdSingleBar( column+1,46, 37 , 3, PIXEL_OFF );       // �������� ����� ������� ����� �������
     //______________������ ��������� � ���������__________________________________________________
     LcdGotoXYFont(0,0);
     LcdString_4(5500+5*EncoderRead());                          // ����� ��� ������ ������� � ��������� 
     LcdGotoXYFont(10,0);
     LcdString_2(rssi_dB_mas);                             // ����� ��� ������ RSSI � ��������� � ���������
     //______________������ ��������� �����________________________________________________________
    // if(EncoderRead()>100) EncoderWrite(100); 
     if(buf_Encoder>100) buf_Encoder=100; 
     if(buf_Encoder!=EncoderRead())                              // ���� �� ��������� �������, �� ������������ ����� �������� ����� � ������ ��������
     {
        unsigned int m = (deltaRSSI*10)/36;                       // ��������� ������� ������� � ��������(delta) 37pix-������� ���� � ��������
        unsigned int db_unsigned = masRSSI[buf_Encoder]/m;                
        LcdSingleBar(Freg_to_pixel(buf_Encoder), 46, 37, 1, PIXEL_OFF);           // ������� ���� ������� 
        LcdSingleBar(Freg_to_pixel(buf_Encoder), 46, db_unsigned, 1,PIXEL_ON);    // �� ���� ����� ������ ������� �� ������ ������ RSSI
        buf_Encoder=EncoderRead();                                                      // ��������� ����� ��������
        LcdSingleBar(Freg_to_pixel(EncoderRead()),46, 38, 1, PIXEL_ON  );               // ������ ����� �����, ���� ��������� ���������
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

void meny_setup_step(void) // ���� ��������� ���� ��������� �������
{
    while(!ButtonRead())
    {
        //������������� ��� ��������� �����
    if(EncoderRead()>4) EncoderWrite(4); 
    if(EncoderRead()==0) frequency_step = 1;
    if(EncoderRead()==1) frequency_step = 2;
    if(EncoderRead()==2) frequency_step = 5;
    if(EncoderRead()==3) frequency_step = 10;

    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  ���������: "));
    LcdGotoXYFont(0,2);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("���     "));
    LcdString_no_nul_2(frequency_step);
    LcdFStr(FONT_1X,(unsigned char*)PSTR(" ���"));
    LcdSingleBar(0,24,9,84, PIXEL_XOR);
    LcdUpdate();
    LcdClear();
    }
}

unsigned int meny_setup_frecyency(unsigned int frequency) // ���� ��������� �������, ������� ����� ��������������� �� ������� ����� ����������
{
  EncoderWrite((6000-frequency)/frequency_step); // ���������� � ������� ������� �������
  while(!ButtonRead())
    {
    if(EncoderRead()>500/frequency_step) EncoderWrite(500/frequency_step); // ������������ ������� �������� � ����������� �� ���� ���������
    frequency = (6000-(frequency_step*EncoderRead()));
    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  ���������: "));
    LcdGotoXYFont(0,3); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("������ "));
    LcdString_4(frequency);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("���"));
    LcdSingleBar(0,24+8,9,84, PIXEL_XOR);
    LcdUpdate();
    LcdClear();
    }
    return frequency;
}

unsigned char meny_setup_contrast(void) // ���� ��������� �������, ������� ����� ��������������� �� ������� ����� ����������
{ 
  EncoderWrite(contrast);
  while(!ButtonRead())
    {
    if(EncoderRead()>127) EncoderWrite(127); // ������������ ������� �������� � ����������� �� ���� ���������
    if(EncoderRead()<57) EncoderWrite(57);
    contrast = EncoderRead();
    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  ���������: "));
     LcdGotoXYFont(0,4); 
     LcdFStr(FONT_1X,(unsigned char*)PSTR("�������� "));
     LcdString_4(contrast);
     LcdSingleBar(0,24+8+8, 9,84, PIXEL_XOR);

    LcdContrast(contrast);
    LcdUpdate();
    LcdClear();

    }
    return contrast;
}
//==============================================================================
//========================= ����� - ��������� ==================================
//______________________________________________________________________________  
void meny_setup(void)
{
  unsigned int eeprom_freg = eeprom_read_word(&eeprom_eemem_freguence);   // ������ ���� ��������� �� eeprom
  contrast = eeprom_read_word(&eeprom_eemem_contrast);                 
  exit_meny = 0;
    while(!exit_meny)
    {
    LcdGotoXYFont(0,0); 
    LcdFStr(FONT_1X,(unsigned char*)PSTR("  ���������: "));
    LcdGotoXYFont(0,2);
    LcdFStr(FONT_1X,(unsigned char*)PSTR("���     "));
    LcdString_no_nul_2(frequency_step);
    LcdFStr(FONT_1X,(unsigned char*)PSTR(" ���"));

     LcdGotoXYFont(0,3); 
     LcdFStr(FONT_1X,(unsigned char*)PSTR("������ "));
     LcdString_4(eeprom_freg);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("���"));

     LcdGotoXYFont(0,4); 
     LcdFStr(FONT_1X,(unsigned char*)PSTR("�������� "));
     LcdString_no_nul_2(contrast);

     if(EncoderRead()>2) EncoderWrite(2);

     unsigned int Button_time = ButtonRead(); // ������ ����� ������� ������
     unsigned char pressed_button=0;
     if(Button_time)
     {
       if(Button_time>500) {exit_meny =1;} // ���� ������� �������, �� ������� � ���� ��������
       if(Button_time<500){ pressed_button=1;} // ���� �������� �������, �� ���������� ����
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
     eeprom_write_word(&eeprom_eemem_freguence, eeprom_freg); // ��������� ���� ��������� � eeprom ��� ������
     eeprom_write_word(&eeprom_eemem_contrast, contrast);
}


int main(void)
{ 
    ADC_Init();
    LcdInit();
    EncoderInit();
    timer0_init();    // ����������� ������� ��� ��������
    RX5808_Init();
    Button_Init();
    DDR_RSSI |= (0<<RSSI_pin);
    LcdClear();
    contrast = eeprom_read_word(&eeprom_eemem_contrast); 
    if(contrast>127) contrast = 127;
    if(contrast<55) contrast =  55;
    LcdContrast(contrast);
    LcdImage(Picture);    // ������ ��������
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
     LcdFStr(FONT_1X,(unsigned char*)PSTR("��������"));
     LcdGotoXYFont(4,1);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("������"));
     LcdGotoXYFont(4,2);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("������"));
     LcdSingleBar(3,47,1 ,83, PIXEL_ON );    // �������������� ����� ������
     LcdGotoXYFont(1,4);
     LcdFStr(FONT_1X,(unsigned char*)PSTR("���: "));
     prog_freg(eeprom_freg ); 
     LcdString_4(eeprom_freg );
     LcdFStr(FONT_1X,(unsigned char*)PSTR("���"));

      unsigned int pixel = Read_RSSI()/23;     // 23 - ����������, (341(maxRSSIadc)*10)-(155(minRSSIadc)*10)/79pix  = 23   
      LcdSingleBar(3,41,1 ,80, PIXEL_ON );     // ����� ��� RSSI
      LcdSingleBar(83,47, 7 , 1, PIXEL_ON );   // ����������� �������� ����� RSSI � �����
       LcdSingleBar(2,47,7 ,1, PIXEL_ON );     // ����� ��� RSSI  ����� ���
      LcdSingleBar(3,46,5,pixel,PIXEL_ON);     // ������ ��� ������� RSSI
     frequency =0;
     if(EncoderRead()>2) EncoderWrite(2);

     unsigned int Button_time = ButtonRead(); // ������ ����� ������� ������
     unsigned char pressed_button=0;
     if(Button_time)
     {
       if(Button_time>500) {meny_setup();eeprom_freg = eeprom_read_word(&eeprom_eemem_freguence);} // ���� ������� �������, �� ������� � ���� ��������
       if(Button_time<500){ pressed_button=1;} // ���� �������� �������, �� ���������� ����
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
    






