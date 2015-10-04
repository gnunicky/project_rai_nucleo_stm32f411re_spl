/**
 * MCMRWM is a program written in C for STMicroelectronics Nucleo F411RE
 * microcontroller.
 * The software was produced as a part of an examination proof of Industrial
 * Automation Networks course held by the University of Catania, Italy,
 * academic year 2014-2015. 
 * The code, flashed no an STMicroelectronics microcontroller, implements
 * basic, read/write routines of a magnetic core memory array and is a porting
 * of an existing project for Arduino I developed by Ben North and Oliver Yash.
 *
 * Copyright (C) 2015 onwards Nicola Didomenico (nicola.didomenico@gmail.com)
 * Copyright (C) 2015 onwards Salvatore Del Popolo (popolo@tin.it)
 *
 * This program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version.
 * This program is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public Licens along with 
 * this program. If not, see <http://www.gnu.org/licenses/>.
**/
 
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#define ADDRSIZE	5
#define	WORDSIZE	(1 << ADDRSIZE)

static GPIO_InitTypeDef  GPIO_InitStruct_A;
static GPIO_InitTypeDef  GPIO_InitStruct_AS;
static GPIO_InitTypeDef  GPIO_InitStruct_B;
static GPIO_InitTypeDef  GPIO_InitStruct_C;
USART_InitTypeDef USART_InitStructure;

static void w_test(void);
static void r_test(void);
static void W_test(void);
static void R_test(void);
static void t_test(void);

int read_bit(const int n);
void write_bit(int n, const int v);
unsigned long read_word(void);
void write_word(unsigned long v);

void long_to_binary(unsigned long decimalNumber);
void address_to_binary(unsigned long decimalNumber);

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART2_Configuration(void);
void printf2(const char *format, ...);
void USART_putc(USART_TypeDef* USARTx,char c);
void USART_puts(USART_TypeDef* USARTx,const char *s);

void timing_enable(void);
void timing_delay_us(unsigned int tick);
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC; //address of the register

int option;
/* Variable used to identify the command inserted in the interactive menu. */

void address_to_binary(unsigned long decimalNumber)
{
/* The function prints the address of the selected read/written core into
   binary form. */

    unsigned long quotient;
    int binaryNumber[5],i=0,j;
    quotient = decimalNumber;
    while(quotient!=0){
         binaryNumber[i++]= quotient % 2;
         quotient = quotient / 2;
    }
    printf(" B"); 
    if (decimalNumber == 0 ) printf2("00000");
    else
    { 
    for(j = i -1 ;j>=0;j--)
         printf2("%d",binaryNumber[j]);
    }    
}

void long_to_binary(unsigned long decimalNumber)
{
/* The function prints the value contained or to be written in the Magnetic
   Core Memory into binary form. All 32 bits are displayed. */

    unsigned long quotient;
    int binaryNumber[WORDSIZE],i,j;
    for(i=0; i<WORDSIZE;i++)
         binaryNumber[i]=0;
    quotient = decimalNumber;
    i=0;
    while(quotient!=0){
         binaryNumber[i++]= quotient % 2;
         quotient = quotient / 2;
    }
    printf2(" B"); 
    for(j = WORDSIZE-1;j>=0;j--)
         printf2("%d",binaryNumber[j]);
    printf2 ("\n\r");
}

void write_bit(int n, const int v)
{
/* This function writes 0 or 1 in the selected core of the Memory Array. It is a
   basic routine called from write_word(). */

  int q, r;

  if (v == 0)
  {
    GPIO_ResetBits(GPIOA, GPIO_Pin_9);
  }
  else
  {
    GPIO_SetBits(GPIOA, GPIO_Pin_9);
  }
/* At this stage it is clear whether 0 or 1 is to be written in the selected
   core. */

  q = n / 2;
  r = n % 2;
  if (r == 1)
   GPIO_SetBits(GPIOB, GPIO_Pin_3);
  else
   GPIO_ResetBits(GPIOB, GPIO_Pin_3);
/* This if-then structure and the following 4 are used to set the appropriate
   pin to address the selected core to be written. */

  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
   GPIO_SetBits(GPIOB, GPIO_Pin_5);
  else
   GPIO_ResetBits(GPIOB, GPIO_Pin_5);
  
  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
    GPIO_SetBits(GPIOB, GPIO_Pin_4);
  else
    GPIO_ResetBits(GPIOB, GPIO_Pin_4);
  
  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
   GPIO_SetBits(GPIOB, GPIO_Pin_10); 
  else
   GPIO_ResetBits(GPIOB, GPIO_Pin_10);
  
  n = q;
  q = n / 2;
  r = n % 2;
  if (r == 1)
   GPIO_SetBits(GPIOA, GPIO_Pin_8); 
  else
   GPIO_ResetBits(GPIOA, GPIO_Pin_8);  
/* Now we are ready to write 0 or 1 in the selected core. */

  GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  GPIO_SetBits(GPIOA, GPIO_Pin_10); 
  timing_delay_us(128); 
  GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  timing_delay_us(128);
}

int read_bit(const int n)
{
/* This function reads the selected core of the Memory Array. It is a basic
   routine called from read_word(). */

  write_bit(n, 0);
  if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)==1) 
  {
    write_bit(n, 1);
    return 1;
   }
   else
   {
    return 0;
   }
}

unsigned long read_word(void)
{
/* This piece of code reads the content of the entire Memory Array, all 32
   cores. */

 unsigned long v = 0;
 int n;
	for (n = WORDSIZE-1; n >= 0; n--)
	{
		v <<= 1;
		v |= read_bit(n);
	}
	return v;
}

static void R_test()
{
/* This routine, calling read_word(), displays the content of the entire
   Memory Array. */

 unsigned long d = read_word();
 printf2("\n\r Core data read: "); 
 printf2(" 0x%lx",d); 
 printf2(" =>"); 
 long_to_binary(d);  
}

void write_word(unsigned long v)
{
/* This routine, calling write_bit(), writes a value, max 32 bits, in the
   Memory Array. */

  int n;
  for(n = 0; n < WORDSIZE; n++)
  {
    write_bit(n, v & 1);
    v >>= 1;
  }
}

static void W_test()
{
/* The function, calling write_word(), writes the desired value, max 32 bits,
   on the Memory Array. */

  char ch[9];
  int i=0;
  unsigned long d = 0;
  printf2("\n\r Please enter 8 hexadecimal digits: \n\r"); 
  while(1) 
  { 
   while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
   ch[i++]=USART_ReceiveData(USART2); // Collect Char
   if (i == 8) 
   { 
     ch[8]='\0'; 
     break;
   }
  }
  for(i = 0; i <= 7; i++)
  {
    d <<= 4;
    if('0' <= ch[i] && ch[i] <= '9')
    {
      d += ch[i] - '0'; 
    }
    else if('A' <= ch[i] && ch[i] <= 'F')
    {
      d += ch[i] - 'A' + 10;
    }
    else if('a' <= ch[i] && ch[i] <= 'f')
    {
      d += ch[i] - 'a' + 10;
    }
    else
    {
      printf2("\n\r Assuming 0 for non-hexadecimal digit: %c \r\n",ch[i]);    
    }
    }
    write_word(d);
    printf2("\n\r Core data write: "); 
    printf2(" 0x%lx",d); 
    printf2(" =>"); 
    long_to_binary(d);
}

static void w_test()
{
/* This routine, calling write_bit(), writes 0 or 1 on the desired core, from
   0 to 31. */

  char ch[6];
  int i =0, a = 0; 
  printf2("\n\r Please enter address of bit to write: \n\r");
  while(1) 
  { 
   while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
   ch[i++] = USART_ReceiveData(USART2); // Collect Char  
   if (i == 5) 
   { 
     ch[5]='\0'; 
     break;
   }
  } 
  for(i = 0; i < ADDRSIZE; i++)
  {
    a <<= 1;
    if(ch[i] != '0') // Assert ch == '1'
    {
      a += 1;
    }
  } 
  printf2("\n\r Please enter bit to write: \n\r");
  i=0;
  while (1)
  {
   while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
   ch[i++]= USART_ReceiveData(USART2); // Collect Char 
   if (i == 1) 
   { 
     ch[1]='\0'; 
     break;
   }
  }
  printf2("\n\r Core data write to address: "); 
  address_to_binary (a);
  printf2(" value ");
  if(ch[0] == '0')
  {
    write_bit(a, 0);
    printf2("0 \n\r");
  }
  else // Assert ch == '1'
  {
    write_bit(a, 1);
    printf2(" 1 \n\r");
  }
}

static void r_test()
{
/* The function, calling read_bit(), reads the desired core, from 0 to 31. */

  char ch[6];
  int i=0, a = 0; 
  printf2("\n\r Please enter address of bit to read: \n\r");
  while (1)
  { 
   while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET); // Wait for Char
   ch[i++]= USART_ReceiveData(USART2); // Collect Char 
   if (i == 5) 
   { 
     ch[5]='\0'; 
     break;
   }
  }
  for(i = 0; i < ADDRSIZE; i++)
  {
    a <<= 1;
    if(ch[i] != '0') // Assert ch == '1'
    {
      a += 1;
    }
  }  
  printf2("\n\r Core data read from address: ");
  address_to_binary(a);
  printf2(" found: ");
  printf2(" %d\n\r",read_bit(a));
}

static void t_test()
{
/* This is an utility function that verifies whether all 32 cores are
   correctly functioning. Cores are scanned from 0 to 31 and read and write
   operations are performed displaying a results. table. */

  int i;
  unsigned long d;
  for(i = 0; i < WORDSIZE; i++)
  {
    write_bit(i, 0);
    d = read_bit(i);
    printf2("\n\r Address (");
    printf2("%d",i>>3);
    printf2("%d",i&7);
    printf2(") ");
    address_to_binary(i);
    printf2(": wrote 0 read "); 
    printf2("%d",d);
    write_bit(i, 1);
    d = read_bit(i);
    printf2(" wrote 1 read "); 
    printf2("%d\n\r",d);
  }    
}

void RCC_Configuration(void)
{
/* This function sets clock configuration for USART2 and GPIOs. */

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  // USART2 clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  // GPIOA clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  // GPIOB Peripheral clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  // GPIOC Peripheral clock enable
}

void GPIO_Configuration(void)
{
/* In this function GPIOs re configured and initialized. */

   // Connect USART pins to AF, PA02 USART2_TX, PA03 USART2_RX
  GPIO_InitStruct_AS.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; 
  GPIO_InitStruct_AS.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct_AS.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_AS.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct_AS.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOA, &GPIO_InitStruct_AS);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
  
  // Configure PA10 IO in output D2 PIN ARDUINO
  // Configure PA08 IO in output D7 PIN ARDUINO 
  // Configure PA09 IO in output D8 PIN ARDUINO     
  GPIO_InitStruct_A.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStruct_A.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct_A.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_A.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct_A.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOA, &GPIO_InitStruct_A);
  GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  
  // Configure PB03 IO in outpur D3 PIN ARDUINO
  // Configure PB05 IO in output D4 PIN ARDUINO 
  // Configure PB04 IO in output D5 PIN ARDUINO  
  // Configure PB10 IO in output D6 PIN ARDUINO
  GPIO_InitStruct_B.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_10;
  GPIO_InitStruct_B.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct_B.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_B.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct_B.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStruct_B);
  
  // Configure PC07 IO in input D9 PIN ARDUINO
  GPIO_InitStruct_C.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStruct_C.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct_C.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_C.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct_C.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStruct_C); 
}  

void USART2_Configuration(void)
{
/* In this function, USART2 is configured and iintialized. */

  /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);
}

void USART_putc(USART_TypeDef* USARTx,char c)
{
/* Function required by printf2(). */

 while(!(USARTx->SR & 0x00000040));
 USART_SendData(USART2,c);
}
 
void USART_puts(USART_TypeDef* USARTx,const char *s)
{
/* Function required by printf2(). */

 int i;
 for(i=0;s[i]!=0;i++) USART_putc(USARTx,s[i]);
}

void printf2(const char *format, ...)
{
/* This piece of code is a reimplementation of the C printf() function. */

 va_list list;
 va_start(list, format);
 int len = vsnprintf(0, 0, format, list);
 char *s;
 s = (char *)malloc(len + 1);
 vsprintf(s, format, list);
 USART_puts(USART2,s);
 free(s);
 va_end(list);
 return;
}

void timing_enable(void)
{
/* This piece of code set microcontroller's registers required from the
   timing_delay_us() function. */

    static int enabled = 0;
    if (!enabled)
    {
        *SCB_DEMCR = *SCB_DEMCR | 0xF42400;
        *DWT_CYCCNT = 0; // reset the counter
        *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
        enabled = 1;
    }
}

void timing_delay_us(unsigned int tick)
{
/* This piece of code, together with timing_enable(), implements microseconds
   delays. We use a 2 microseconds delay. */

    unsigned int start, current;
    start = *DWT_CYCCNT;
    do
    {
        current = *DWT_CYCCNT;
    } while((current - start) < tick);
}

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f40_41xxx.s/startup_stm32f427_437xx.s/startup_stm32f429_439xx.s)
       before to branch to application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */  

  RCC_Configuration();
  GPIO_Configuration();
  USART2_Configuration();
  timing_enable();

  printf2("\n\rWelcome! If you're new, try using the commands 'r', 'w', 't' and 'R', 'W' to get started.\n\r");
  while (1)
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET | USART_GetFlagStatus(USART2, USART_FLAG_IDLE) == RESET  ); // Wait for Char
    option = USART_ReceiveData(USART2); // Collect Char 
   switch (option) 
    {
/* We test which command is to be executed. */

      case 'w':
        w_test();
        break;
      case 'r':  
        r_test();
        break;    
      case 'W':
        W_test();
        break;
      case 'R':  
        R_test();
        break;
      case 't':  
        t_test();
        break;
      default:
        printf2("\n\r Ignoring unknown command: %c \n\r", option);
    }
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

