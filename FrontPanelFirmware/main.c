/*

    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#include "ch.h"
#include "hal.h"
#include "stdio.h"
#include "string.h"
//#include "stdlib.h"

#include <string.h>
#include "stm32f3xx.h"
//#include "star.h"
#include "font.h"

#define CS 9
#define RST 12
#define SPISELECT 11
#define CK 13
#define MISO 14
#define MOSI 15
#define DC 10



static uint8_t vbuf2[64][128];
volatile static uint8_t vbuf[64][128];
static uint8_t txbuf[2];

static uint8_t oled_current_row;
static uint8_t oled_current_column;
//static char device = '3';

static mailbox_t KeyMbx,RxMbx,BeepMbx;
static binary_semaphore_t Keypressed;
#define MAILBOX_SIZE 25
//static msg_t KeyMbxBuff[MAILBOX_SIZE];
//static msg_t RxMbxBuff[MAILBOX_SIZE];
//static msg_t BeepMbxBuff[MAILBOX_SIZE];
static char current_key =255;
static uint8_t current_row= 4;
static uint8_t strobe_wait= 1;



static char rx_text[32][32];
static int rx_queue_pos=0;
static int rx_queue_num=0;

static SerialConfig uartCfg =
{
    115200,// bit rate
    0,
    0,
    0,
};


static SerialConfig uartCfg2 =
{
    9600,// bit rate
    0,
    0,
    0,
};







static const SPIConfig std_spicfg0 = {
  NULL,
  NULL,
  GPIOB,                                                        /*port of CS  */
  9,                                                /*pin of CS   */

  //SPI_CR1_BR_2  |SPI_CR1_BR_1  |SPI_CR1_BR_0  |	\
  //SPI_CR1_SPE|SPI_CR1_MSTR,
  0,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0                    /*CR2 register*/
};






void spi_write0(location)
{
  spiStart(&SPID2,&std_spicfg0);
  spiSelect(&SPID2);
  txbuf[0] = location;
  spiSend(&SPID2,1,&txbuf);
  spiUnselect(&SPID2);
  spiStop(&SPID2);
}

void write_oled_command(char data)
{
    palClearPad(GPIOB,DC);
    spi_write0(data);
}


void write_oled_data(char data)
{
    palSetPad(GPIOB,DC);
    spi_write0(data);
}


void spi_write(location,data)
{
  spiStart(&SPID2,&std_spicfg0);
  spiSelect(&SPID2);
  txbuf[0] = location;
  txbuf[1] = data;
  spiSend(&SPID2,2,&txbuf);
  spiUnselect(&SPID2);
  spiStop(&SPID2);
}


void set_oled_text_pos(uint8_t x,uint8_t y)
{
    oled_current_column = (x*6+2);
    oled_current_row = (y*16);
}



void write_oled_char(char a)
{
    uint8_t j,i;

    for (j=0; j<16; j++)
	{
	    
	    for (i=0; i<6; i++)
		{
		    vbuf2[oled_current_row+j][oled_current_column+i] = font[a][j][i];
		}
	}
}


void oled_draw_string(uint8_t x,uint8_t y,char* text)
{
    //uint8_t currentx = x;
  uint8_t j;

  for (j=0;j<(strlen(text)-1);j++)
    {
      set_oled_text_pos(x+j,y);
      write_oled_char(text[j]);
    }
}



void init_spi()
{
  palSetPadMode(GPIOB, CS, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, RST, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, DC, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, SPISELECT, PAL_MODE_OUTPUT_PUSHPULL);
  

  palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 6, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 7, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 8, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 9, PAL_MODE_INPUT_PULLUP);
//  palSetPadMode(GPIOD, 10, PAL_MODE_INPUT_PULLUP);
//  palSetPadMode(GPIOD, 11, PAL_MODE_INPUT_PULLUP);
//  palSetPadMode(GPIOD, 12, PAL_MODE_INPUT_PULLUP);
//  palSetPadMode(GPIOD, 13, PAL_MODE_INPUT_PULLUP);
//  palSetPadMode(GPIOD, 14, PAL_MODE_INPUT_PULLUP);
//  palSetPadMode(GPIOD, 15, PAL_MODE_INPUT_PULLUP);
//


  
  palSetPad(GPIOB,CS);
  palSetPad(GPIOB,RST);
  //palClearPad(GPIOB,CK);
  palClearPad(GPIOB,SPISELECT); // this actually selects SPI rather than 8 bit interface

}

void clear_oled()
{

    //chprintf(&SD1,"Did Clear OLED\r\n");
    memset(vbuf2,0xff,100); // I set the clear to be 0x11 instead of 0x00
    
                               // because the LCD would 'freak out' if lots
                               // of null data was sent

}



void shade_oled(uint8_t shade)
{
    memset(vbuf2,shade,128*64);
}




void init_oled()
{
      uint8_t i, j,x;


      chprintf(&SD1,"do oled command A\r\n");


  write_oled_command(0xfd); // Unlock 
  write_oled_data(0x12);

  write_oled_command(0xae | 0); // Set_Display_Off

  write_oled_command(0x15); // Set_Column_Address
  write_oled_data(0x1c);
  write_oled_data(0x5b);

  write_oled_command(0x75); // Set_Row_Address
  write_oled_data(0x00);
  write_oled_data(0x3f);

  write_oled_command(0xca); // Set_Multiplex_Ratio
  write_oled_data(0x3f);

  write_oled_command(0xa0); // Set_Remap_Format
  write_oled_data(0x14); 
  write_oled_data(0x11); 


  write_oled_command(0xb3); // Set_Display_Clock
  write_oled_data(0x31);  // was 22 (31)

  write_oled_command(0xc1); // Set_Contrast_Current
  write_oled_data(0xff);

  write_oled_command(0xc7); // Set_Master_Current
  write_oled_data(0x0f);

  write_oled_command(0xb9); // Set_Linear_Gray_Scale_Table();
  //write_oled_command(0x00); // enable gray scale table

  write_oled_command(0xb1); // Set_Phase_Length(0xE2); 
  write_oled_data(0xe2);

  write_oled_command(0xbb); // Set_Precharge_Voltage(0x1F);
  write_oled_data(0x1f);

  write_oled_command(0xb6); // Set_Precharge_Period(0x08);
  write_oled_data(0x08);

  write_oled_command(0xbe); // Set_VCOMH(0x07);
  write_oled_data(0x00);



    write_oled_command(0xA5); // set display all on

  
    //clear_oled(); 
  write_oled_command(0xae | 1); // Set_Display_On_Off(0x01);
  write_oled_command(0xa4 | 0x02); // Set_Display_Mode(0x02);
  write_oled_command(0x5c);
  palSetPad(GPIOB,DC);
  
}


void graphics_init()
{
  uint8_t row;
  uint8_t col;


  //clear_oled();
  oled_draw_string(0,3,"Test ");
  //clear_oled();
  //oled_draw_string(0,0,"7890 ");
  //oled_draw_string(5,2,"System ");
  //oled_draw_string(4,3,"& 4W Alignment ");


}


void graphics_init2()
{
  uint8_t row;
  uint8_t col;


  clear_oled();
  oled_draw_string(0,3,"tEST ");
  oled_draw_string(0,0,"7890 ");
  //oled_draw_string(5,2,"System ");
  //oled_draw_string(4,3,"& 4W Alignment ");


}



dbg(char *string)
{
    chprintf((BaseSequentialStream*)&SD1,string);
}


/*
 * Application entry point.
 */



uint32_t checksum()
{
    uint32_t checksum;
    int i;
    int j;
    checksum =0;
    for (i=0;i<64;i++)
	for(j=0;j<128;j++)
	    checksum = checksum+vbuf2[i][j];
    return checksum;
}




static THD_WORKING_AREA(waThread2, 1024);
static THD_FUNCTION(Thread2, arg) {
  (void)arg;
  int pass = 0;
  int cleared;
  uint32_t cksum;
  chRegSetThreadName("ScreenRefresh");

  //  chprintf((BaseSequentialStream*)&SD1,"Start Update\r\n");

  while (TRUE) {
      spiStart(&SPID2,&std_spicfg0);
      spiSelect(&SPID2);      

      //cksum = checksum();
      //if (cksum == 0)	  
      spiSend(&SPID2,64*128,&vbuf2);
      //else
      //	  spiSend(&SPID2,64*128,&vbuf);
      
      spiUnselect(&SPID2);
      spiStop(&SPID2);
      chThdSleepMilliseconds(10);
  
  }

  return MSG_OK;
}






static THD_WORKING_AREA(waThread3, 128);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  int pass = 0;
  char b = 0;
  chRegSetThreadName("serial");
  //chprintf((BaseSequentialStream*)&SD1,"entered serial %d\r\n",rx_queue_num);
  while(TRUE)
      {
	  b = sdGet(&SD2);
	  //chprintf((BaseSequentialStream*)&SD1,"got char: %c\r\n",b);
	  if (b == 64)
	      {
		  rx_queue_pos = 0;
	      }
	  
	  if (rx_queue_pos < 31)
	      {
		  rx_text[rx_queue_num][rx_queue_pos++]=b;
	      }
	  if (b == 13)
	      {
		  //chprintf((BaseSequentialStream*)&SD1,"got endstream %d\r\n",rx_queue_num);
		  rx_text[rx_queue_num][rx_queue_pos] = 0;
		  rx_queue_pos = 0;
		  chMBPostTimeout(&RxMbx,rx_queue_num,TIME_INFINITE); // let our mailbox know
		  // we have a new entry
		  rx_queue_num = (++rx_queue_num)%32;
		  //memset(rx_text[rx_queue_num],0,5);
	      }
	  
  
      }

  return MSG_OK;
}

uint8_t decode_pos(char pos)
{
  return pos - 32;
}

    
//static THD_WORKING_AREA(waThread4, 2048);
//static THD_FUNCTION(Thread4, arg) {
//    int charnum;
//    int index;
//    msg_t key;
//    int x;
//    char *starttxt;
//    char text[255];
//    char lcltext[32];
//    int row;
//    int col;
//    int len;
//    msg_t rxRow;
//    msg_t response;
// 
//    while (TRUE)
//	{
//	
//	    chMBFetchTimeout(&RxMbx,&rxRow,TIME_INFINITE);
//
//
//	    strncpy(lcltext,rx_text[rxRow],32);
//	  
//	    if (lcltext[0] == SOMC && lcltext[1] == device)
//		{
//
//		    if (lcltext[2] == 13)
//			{
//			    // rework this part
//			    response=chMBFetchTimeout(&KeyMbx,&key,TIME_IMMEDIATE);
//			    if (response == MSG_TIMEOUT)
//				key = 0x00;
//
//
//			    //chprintf(&SD1,"sending key %x \r\n",key);
//			    text[0] = '@';
//			    text[1] = key;
//			    text[2] = 0x0d;
//			    palSetPad(GPIOA,1);
//			    sdWrite(&SD2,&text,3);
//			    chThdSleepMilliseconds(1);
//			    palClearPad(GPIOA,1);
//			    chprintf(&SD1,"%c%2x%2x\r\n",text[0],text[1],text[2]);
//	      	     
//			}
//
//		    if (lcltext[3] == 'Y' && lcltext[6] == 7)
//			{
//			    //sprintf(text,"Beep!\r\n");
//			    dbg("Beep!\r\n");
//			    chMBPostTimeout(&BeepMbx,500,TIME_INFINITE); // let our mailbox know
//			}
//
//		    if (lcltext[3] == 'Y' && lcltext[6] != 7)
//			{
//			    len = strlen(lcltext+6)-1;
//			    row=decode_pos(lcltext[4]);
//			    col=decode_pos(lcltext[5]);
//			    //memcpy(lcd_screen[row]+col,lcltext+6,len);		 
//			    oled_draw_string(col,row,lcltext+6);
//			    chprintf(&SD1,"%d,%d:%s\r\n",row,col,lcltext+6);
//			}
//		    if (lcltext[3] == 'E')
//			{
//			    //sprintf(text,"*clr*\r\n");
//			    dbg("*clr*\r\n");			   
//			    clear_oled();
//			}
//		    if (lcltext[3] == 'S')
//			{
//			    
//
//			    chprintf(&SD1,"*blink-on* %x\r\n",checksum());
//			    //dbg(text);
//			    
//			    //blink_on_lcd();
//			}
//		    if (lcltext[3] == 'R')
//			{
//			    //sprintf(text,"*blink-off*\r\n");
//			    dbg("*blink-off*\r\n");	      	    
//			    //blink_off_lcd();
//			}
//		    if (lcltext[3] == 'G')
//			{
//			    //sprintf(text,"*underline-off*\r\n");
//			    dbg("*underline-off\r\n");	      	    
//			    //underline_off_lcd();
//			}
//		    if (lcltext[3] == 'F')
//			{
//			    //sprintf(text,"*underline-on*\r\n");
//			    dbg("*underline-on*\r\n");	      	    
//			    //underline_on_lcd();
//			}
//
//
//
//		}
//	    //if it's not meant for us, then ignore message.
//	}
//
//    }
//
//
//static THD_WORKING_AREA(waThread5, 128);
//static THD_FUNCTION(Thread5, arg) {
//    while (TRUE)
//	{
//	    if (! strobe_wait)
//		{
//		    //		    chprintf(&SD1,"CurrentRow: %d\r\n",current_row);
//		    palClearPad(GPIOC,current_row);
//		    current_row ++;
//		    if (current_row > 10)
//			current_row = 4;
//		    palSetPad(GPIOC,current_row);
//			
//		}
//	    chThdSleepMilliseconds(10);
//	}
//}
//
//
//
//static THD_WORKING_AREA(waThread6, 128);
//static THD_FUNCTION(Thread6, arg) {
//    while (TRUE)
//	{
//	    chBSemWait(&Keypressed);
//	    chprintf(&SD1,"GotKeypress: %X,%X\r\n",current_row,current_key);
//	    
//	    while (palReadGroup(GPIOC,0xf,0) != 0)
//		{
//		    chprintf(&SD1,"GotKeypress: %X\r\n",palReadGroup(GPIOC,0xf,0));		  
//		    chThdSleepMilliseconds(100);
//		}
//	    dbg("Keypress Done\r\n");
//	    strobe_wait = 0;
//	}
//}
//		
//

int main(void) {
  unsigned i;
  char text[255];
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  //chMBObjectInit(&KeyMbx,&KeyMbxBuff,MAILBOX_SIZE);
  //chMBObjectInit(&RxMbx,&RxMbxBuff,MAILBOX_SIZE);
  //chMBObjectInit(&BeepMbx,&BeepMbxBuff,MAILBOX_SIZE);
  //chBSemObjectInit(&Keypressed,1);
  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));

  //palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));    
  //palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
  //palSetPadMode(GPIOB, 11, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  
  
  sdStart(&SD1, &uartCfg);
    sdStart(&SD2, &uartCfg2);
    // chprintf((BaseSequentialStream*)&SD2,"Hello World 2\r\n");
  chprintf((BaseSequentialStream*)&SD1,"Hello World 1\r\n");
  chprintf((BaseSequentialStream*)&SD1,"vbuf %X %X\r\n",vbuf,&vbuf2);
  //palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL);        
  //palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetPadMode(GPIOB, 8, PAL_MODE_OUTPUT_PUSHPULL); 
  //palClearPad(GPIOB, 0);     /* Green.  */
  //palClearPad(GPIOB, 8);     /* Green.  */


  palSetPadMode(GPIOC, 11, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_LOWEST);
  palSetPadMode(GPIOC, 12, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_LOWEST); 
  palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_LOWEST);
  palSetPadMode(GPIOC, 14, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_LOWEST); 




  palSetPad(GPIOC, 11);
  palSetPad(GPIOC, 12);
  palSetPad(GPIOC, 13);
  palSetPad(GPIOC, 14);

    
  chprintf(&SD1,"test from printf \r\n");
  chprintf((BaseSequentialStream*)&SD1,"After start thread\r\n");
  chThdSleepMilliseconds(1000);




  chprintf((BaseSequentialStream*)&SD1,"After set pad Init\r\n");

  init_spi();
  chprintf((BaseSequentialStream*)&SD1,"SPI init\r\n");
  init_oled();
  chprintf((BaseSequentialStream*)&SD1,"OLED init\r\n");
  chThdSleepMilliseconds(1000);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
  chprintf((BaseSequentialStream*)&SD1,"start Thread\r\n");
  chThdSleepMilliseconds(1000);

  //chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);
  //chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);
  //chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL);
  strobe_wait = 0;
  //chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO, Thread6, NULL);  





  // replace this with dma
  chprintf((BaseSequentialStream*)&SD1,"start GRAPHICS\r\n");
  chThdSleepMilliseconds(1000);

  //graphics_init();
  //chThdSleepMilliseconds(1000);
  graphics_init2();
  chprintf((BaseSequentialStream*)&SD1,"Point A\r\n");


    


  while (TRUE)
      {
	  i = (i +1)%128;
	    palTogglePad(GPIOC, 11);
	    palTogglePad(GPIOC, 12);
	    palTogglePad(GPIOC, 13);
	    palTogglePad(GPIOC, 14);

	  chprintf((BaseSequentialStream*)&SD1,"spin\r\n");
	  chThdSleepMilliseconds(1000);
	  //shade_oled(i);
	  vbuf2[0][i] = 0x55;
	  //sprintf(text,"%d",i);
	  //oled_draw_string(0,0,"1");
	      
   }



  return 0;
}
