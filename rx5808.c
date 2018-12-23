#include "rx5808.h"
#include "spi.h"
void RX5808_Init(void)
{
    SPI_Init();
	  RX5808_DDRX  |= (1<<RX5808_SS);
    RX5808_PORTX |= (1<<RX5808_SS);

    CLR(RX5808_PORTX, RX5808_SS);
    SPI_WriteByte(0x10); // отослать байт в RX5808
    SPI_WriteByte(0x01); // отослать байт в RX5808
    SPI_WriteByte(0x00); // отослать байт в RX5808
    SPI_WriteByte(0x00); // отослать байт в RX5808
    SET(RX5808_PORTX, RX5808_SS);
}

 void prog_freg (unsigned int Freguency)               // устанавливаем частоту в RX5808 из регистра freguencu
{ 
	  unsigned int Delitel;
	  unsigned int DelitelH;
	  unsigned int DelitelL;
	
      Delitel=(Freguency -479)/2 ;
      DelitelH=Delitel>>5 ;
      DelitelL=Delitel &0x1F;
  
      CLR(RX5808_PORTX, RX5808_SS);
      SPI_WriteByte(DelitelL *32 +17 );
      SPI_WriteByte(DelitelH *16 + DelitelL /8);
      SPI_WriteByte(DelitelH /16);
      SPI_WriteByte(0x00);
      SET(RX5808_PORTX, RX5808_SS);
}
  
  unsigned int rssi_db(unsigned int rssi, unsigned int deltaRSSI) // перевод данных с RSSI в dB
{
  //Расчеты для диапазона уровня -91..+5дБ , в сумме 96дБ
  unsigned char db=0;
  unsigned int a = (deltaRSSI*100)/96;
  db = (rssi*100)/a; 
  if(db>=91){db=db-91;}else{db=91-db; SET(db,7);} //вычисляем в форму +5dB...-91dB , если меньше то прикрепляем знак минус (db|=0b10000000;)
  return db;
}

unsigned char CH_32_to_band(unsigned char CH_32)  // возвращает сетку частот этого канала
{
   unsigned char CH;
  // Определяем сетку частот 
    if(CH_32<=8 ) CH = 0;
    if(CH_32> 8 ) {if(CH_32<=16) CH = 1;}
    if(CH_32> 16) {if(CH_32<=24) CH = 2;}
    if(CH_32> 24) CH = 3;
    return CH;
}

unsigned char CH_32_to_CH_8(unsigned char CH_32) // переводим из 32 разрядной сетки в 8 ми
{
  unsigned char CH1 = 0;
  // определяем канал
    if(CH_32<=8 ) CH1 = CH_32;
    if(CH_32> 8 ) {if(CH_32<=16) CH1 = CH_32-8;}
    if(CH_32> 16 ){if(CH_32<=24) CH1 = CH_32-16;}
    if(CH_32> 24 ) CH1 = CH_32-24;
    //CH1 - 1
    //CH2 - 2
    //CH3 - 3
    //CH4 - 4
    //CH5 - 5
    //CH6 - 6
    //CH7 - 7
    //CH8 - 8
    return CH1;
}

unsigned int CH_32_to_frequency(unsigned char CH_32)
{
  unsigned int frequency = masFreg[CH_32-1];
  return frequency;
}

unsigned char frequency_to_CH_32(unsigned int frequency1) // поиск канала по частота, если нету то возвращает ноль
{
  unsigned char t=0;
  for(unsigned char i=0; i<32; i++)
  {
    if(frequency1 == masFreg[i]) {t = i+1;}
  }
  return t;
}