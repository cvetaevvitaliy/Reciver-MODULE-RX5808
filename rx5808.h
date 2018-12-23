#ifndef RX5808_H
#define RX5808_H

#define SET(p, n)    p |= _BV(n)
#define CLR(p, n)    p &= ~_BV(n)
#define INV(p, n)    p ^= _BV(n)

//Specify where you connected the SS output from the RX5808,
#define RX5808_PORTX  PORTB
#define RX5808_DDRX   DDRB
#define RX5808_SS    0

//declare frequency grids
static const unsigned int masFreg[32]  = {
//   ch1  ch2  ch3  ch4  ch5  ch6  ch7  ch8   
    5865,5845,5825,5805,5785,5765,5745,5725, //FR1 (A)
    5733,5752,5771,5790,5809,5828,5847,5866, //FR2 (B)
    5705,5685,5665,5645,5885,5905,5925,5945, //FR3 (E)
    5740,5760,5780,5800,5820,5840,5860,5880};//FR4 (F)

void RX5808_Init(void);                  // initialization
void prog_freg (unsigned int Freguency); // set the frequency in RX5808 from the register frequency
 unsigned int rssi_db(unsigned int rssi, unsigned int deltaRSSI); // RSSI to dB

//CH_32 - channel values ​​from 0 to 31, which corresponds to 1 channel-0, 2 channel-1, 3 channel-2 ...
 unsigned char CH_32_to_band(unsigned char CH_32);     // returns the frequency grid of this channel A = 0, B = 1, E = 2, F = 3
 unsigned char CH_32_to_CH_8(unsigned char CH_32);     // translate from 32 grids to 8
 unsigned int CH_32_to_frequency(unsigned char CH_32); // returns channel frequency
unsigned char frequency_to_CH_32(unsigned int frequency); // channel search by frequency, if not, then returns NULL

#endif //RX5808_H