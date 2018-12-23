// Эта библиотека использует библиотеку апаратного SPI, которая лежит в папки.
#ifndef RX5808_H
#define RX5808_H

#define SET(p, n)    p |= _BV(n)
#define CLR(p, n)    p &= ~_BV(n)
#define INV(p, n)    p ^= _BV(n)

//Указываем куда подключили вывод SS от RX5808,
//Он не должын являться аппаратным SS SPI
#define RX5808_PORTX  PORTB
#define RX5808_DDRX   DDRB
#define RX5808_SS    0 //Он не должын являться аппаратным SS, SPI, любой другой пин

//Обьявляем сетки частот
static const unsigned int masFreg[32]  = {
//   ch1  ch2  ch3  ch4  ch5  ch6  ch7  ch8   
    5865,5845,5825,5805,5785,5765,5745,5725, //FR1 (A)
    5733,5752,5771,5790,5809,5828,5847,5866, //FR2 (B)
    5705,5685,5665,5645,5885,5905,5925,5945, //FR3 (E)
    5740,5760,5780,5800,5820,5840,5860,5880};//FR4 (F)

void RX5808_Init(void);                  // инцилизация
void prog_freg (unsigned int Freguency); // устанавливаем частоту в RX5808 из регистра freguencu
 unsigned int rssi_db(unsigned int rssi, unsigned int deltaRSSI); // перевод данных с RSSI в dB

//CH_32 - значения каналов от 0 до 31, что соответствует 1канал-0, 2канал-1, 3канал-2...
 unsigned char CH_32_to_band(unsigned char CH_32);     // возвращает сетку частот этого канала A=0, B=1, E=2, F=3
 unsigned char CH_32_to_CH_8(unsigned char CH_32);     // переводим из 32 разрядной сетки в 8 ми
 unsigned int CH_32_to_frequency(unsigned char CH_32); // возвращает частоту канала
 unsigned char frequency_to_CH_32(unsigned int frequency); // поиск канала по частота, если нету то возвращает ноль

#endif //RX5808_H