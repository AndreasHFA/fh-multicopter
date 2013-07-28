#ifndef USART_H
#define USART_H

//buffer uart
#define rxBUF_SZ 265
#define txBUF_SZ 265

//uart structure
typedef struct {
  unsigned char rxbuffer[rxBUF_SZ];
  unsigned char txbuffer[txBUF_SZ];
  unsigned int rxready;
  unsigned int txready;
  unsigned int rxcnt;
  unsigned int txcnt;
  unsigned int txlen;	// Need for Answer is longer multiple read
}USART_OBJ;

void Usart2Put(uint8_t ch);
void Usart3Put(uint8_t ch);
uint8_t Usart3Get(void);
void InttoBuffer(unsigned char *ptr, uint32_t Value);

#endif /*USART_H */
