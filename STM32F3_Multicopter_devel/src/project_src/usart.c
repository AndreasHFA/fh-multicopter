#include <stm32f30x.h>
#include <stm32f30x_conf.h>

void Usart2Put(uint8_t ch)
{
      USART_SendData(USART2, (uint8_t) ch);
      //Loop until the end of transmission
      while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
      {
      }
}

/*
 * USART3 wurde auf Interrupt modus umprogrammiert
 * */

void Usart3Put(uint8_t ch)
{
      USART_SendData(USART3, (uint8_t) ch);
      //Loop until the end of transmission
      while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
      {
      }
}

uint8_t Usart3Get(){
	uint8_t ch;
	while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
	{
	}
	ch = USART_ReceiveData(USART3);
	return ch;
}

/* TODO
 * Evtl. Typecast auf Bytestream
 */
void InttoBuffer(unsigned char *ptr, uint32_t Value){
	*ptr= (char) (Value >> 24);
	ptr++;
	*ptr= (char) (Value >> 16);
	ptr++;
	*ptr= (char) (Value >> 8);
	ptr++;
	*ptr= (char) Value;
}
