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

#define CHPRINTF_USE_FLOAT   TRUE
#include "ch.h"
#include "hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
//#include "chprintf.h"
#include "hal_queues.h"
#include <string.h>
#include "stm32f3xx.h"

// Create our initial arrays - I don't want this stuff on the stack



#define ADC_GRP1_NUM_CHANNELS   2

#define ADC_GRP1_BUF_DEPTH      1

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];





size_t nx = 0, ny = 0;
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
  dbg('error!!');
}



static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  ADC_CFGR_CONT,            /* CFGR    */
  ADC_TR(0, 4095),          /* TR1     */
  {                         /* SMPR[2] sample Register */
    0,
    ADC_SMPR2_SMP_AN18(ADC_SMPR_SMP_601P5)|ADC_SMPR2_SMP_AN13(ADC_SMPR_SMP_601P5)
  },
  {                         /* SQR[4]  Sequence Register - order & channel to read*/
      ADC_SQR1_SQ1_N(ADC_CHANNEL_IN13)|ADC_SQR1_SQ2_N(ADC_CHANNEL_IN18),
    0,
    0,
    0
  }
};




#define CS 0
#define CS2 1

#define CK 5
#define MISO 6
#define MOSI 7

#define a_lsb 0.061
#define g_lsb 0.00875


// this should set a timeout of .625 seconds (LSI = 40k / (64 * 1000))

static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(1000),
  STM32_IWDG_WIN_DISABLED};

// This got redefined in a later version of Chibios for this board
#define GPIOA_PIN0 0

static const SPIConfig std_spicfg1 = {
  NULL,
  NULL,
  GPIOA,                                                        /*port of CS  */
  GPIOA_PIN0,                                                /*pin of CS   */
  //  SPI_CR1_CPOL|SPI_CR1_CPHA|		\
  //#  SPI_CR1_SPE|SPI_CR1_MSTR,
  0,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0                    /*CR2 register*/
};

static const SPIConfig std_spicfg2 = {
  NULL,
  NULL,
  GPIOA,                                                        /*port of CS  */
  GPIOA_PIN1,                                                /*pin of CS   */
  //  SPI_CR1_CPOL|SPI_CR1_CPHA|		\
  //SPI_CR1_SPE|SPI_CR1_MSTR,
  0,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0                    /*CR2 register*/
};

static uint8_t txbuf[10];
static uint8_t rxbuf[10];
float currentAngle;
uint8_t spi_read(device,location)
{

  spiStart(&SPID1,device);
  spiSelect(&SPID1);
  txbuf[0] = 0x80 | location;
  spiSend(&SPID1,1,&txbuf);
  spiReceive(&SPID1,1,&rxbuf);
  spiUnselect(&SPID1);
  spiStop(&SPID1);
  return(rxbuf[0]);
}
void spi_write(device,location,data)
{
  spiStart(&SPID1,device);
  spiSelect(&SPID1);
  txbuf[0] = 0x00 | location;
  txbuf[1] = data;
  spiSend(&SPID1,2,&txbuf);
  spiUnselect(&SPID1);
  spiStop(&SPID1);
}





//static uint8_t vbuf[64][128];
//static uint8_t vbuf2[64][128];

static mailbox_t RxMbx;
#define MAILBOX_SIZE 25
static msg_t RxMbxBuff[MAILBOX_SIZE];

static mailbox_t RxMbx2;
static msg_t RxMbx2Buff[MAILBOX_SIZE];


static char current_key;
static  uint16_t step =0;
static  int16_t deg,speed =0;
static double deg2;
static char rx_text[32][32];
static char rx2_text[32][32];
static int rx_queue_pos=0;
static int rx_queue_num=0;
static int rx2_queue_pos=0;
static int rx2_queue_num=0;

static uint8_t my_address = 0x10;
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



dbg(char *string)
{
    chprintf((BaseSequentialStream*)&SD1,string);
    chThdSleepMilliseconds(100);
}


/*
 * Application entry point.
 */


/* *******************
stolen from http://www.modbustools.com/modbus.html#crc
*/
uint16_t CRC16 (const char *nData, uint16_t wLength)
{
static const uint16_t wCRCTable[] = {
   0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
   0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
   0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
   0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
   0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
   0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
   0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
   0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
   0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
   0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
   0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
   0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
   0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
   0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
   0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
   0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
   0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
   0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
   0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
   0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
   0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
   0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
   0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
   0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
   0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
   0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
   0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
   0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
   0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
   0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
   0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
   0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

char nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord  ^= wCRCTable[nTemp];
   }
   return wCRCWord;
} // End: CRC16


				





static THD_WORKING_AREA(waThread3, 512);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  int pass = 0;
  msg_t b = 0;

  chRegSetThreadName("serial");
  while(TRUE)
      {
	  
	  b = sdGetTimeout(&SD2,TIME_MS2I(3));

	  
	  if ((b!= Q_TIMEOUT) && (rx_queue_pos < 31))
	      {
		  //chprintf((BaseSequentialStream*)&SD1,"got char: %x\r\n",b);
		  rx_text[rx_queue_num][rx_queue_pos++]=b;
	      }
	  if ((b == Q_TIMEOUT) && (rx_queue_pos > 0))
	      {

		  rx_text[rx_queue_num][rx_queue_pos] = 0;

		  chMBPostTimeout(&RxMbx,(rx_queue_num<<8)|rx_queue_pos,TIME_INFINITE); // let our mailbox know
		  rx_queue_pos = 0;
		  // we have a new entry
		  rx_queue_num = (++rx_queue_num)%32;
		  memset(rx_text[rx_queue_num],0,5);
	      }
	  
  
      }

  return MSG_OK;
}


static THD_WORKING_AREA(waThread5, 512);
static THD_FUNCTION(Thread5, arg) {
  (void)arg;
  int pass = 0;
  msg_t b = 0;

  chRegSetThreadName("serial2");
  while(TRUE)
      {
	  
	  b = sdGetTimeout(&SD3,TIME_MS2I(3));

	  
	  if ((b!= Q_TIMEOUT) && (rx2_queue_pos < 31))
	      {
		  //chprintf((BaseSequentialStream*)&SD1,"got char: %x\r\n",b);
		  rx2_text[rx2_queue_num][rx2_queue_pos++]=b;
	      }
	  if ((b == Q_TIMEOUT) && (rx2_queue_pos > 0))
	      {
		  rx2_text[rx2_queue_num][rx2_queue_pos] = 0;

		  chMBPostTimeout(&RxMbx2,(rx2_queue_num<<8)|rx2_queue_pos,TIME_INFINITE); // let our mailbox know
		  rx2_queue_pos = 0;
		  // we have a new entry
		  rx2_queue_num = (++rx2_queue_num)%32;
		  memset(rx2_text[rx2_queue_num],0,5);
	      }
	  
  
      }

  return MSG_OK;
}





uint8_t decode_pos(char pos)
{
  return pos - 32;
}

    
static THD_WORKING_AREA(waThread4, 2048);
static THD_FUNCTION(Thread4, arg) {
    int charnum;
    int index;
    msg_t key;
    int x;
    char *starttxt;
    char text[255];
    char lcltext[32];
    uint8_t command;
    int row;
    int col;
    int len;
    msg_t rxRow;
    msg_t rxPos;
    msg_t response;
    uint8_t skip_next;
    uint16_t reg;
    uint16_t value;
    while (TRUE)
	{
	    // the skip is because the way I have it hooked up right now
	    // causes it to read whatever we send.
	    chMBFetchTimeout(&RxMbx,&rxRow,TIME_INFINITE);
	    rxPos = rxRow & 0xFF;
	    rxRow = rxRow >> 8;
	    memcpy(lcltext,rx_text[rxRow],rxPos);
	    // if the message is for us and the CRC matches - otherwise -
	    // ignore.

            if ((lcltext[0] == my_address) &&
		(*(uint16_t*)(lcltext+rxPos-2) == CRC16(lcltext,rxPos-2)))

		{
		    
		    command = lcltext[1];		
		    palSetPad(GPIOA,4);
		    //chprintf((BaseSequentialStream*)&SD1,"+");
		    if (command == 4)
			{
			    reg = (lcltext[2]<<8)|lcltext[3];
			    value = step;
			    if (reg==1)
				value = deg;
			    if (reg==2)
				value = speed;
			    lcltext[0] = my_address;
			    lcltext[1] = 4;
			    lcltext[2] = 2;
			    lcltext[3] = (value & 0xFF00 ) >> 8;
			    lcltext[4] = value & 0xFF ;
			    *(uint16_t*)(lcltext+5) = CRC16(lcltext,5);
			    lcltext[7] = 0;
			    sdWrite(&SD2,lcltext,7);
			}
		    else
			sdWrite(&SD2,lcltext,rxPos);
		    //chprintf((BaseSequentialStream*)&SD1,"Queue not empty %X %x %x\r\n",SD3.oqueue.q_counter,SD3.oqueue.q_rdptr,SD3.oqueue.q_wrptr);
		    //chprintf((BaseSequentialStream*)&SD1,"command %d - register %d, %d\r\n",lcltext[1],reg,value);
		    // I've been having problems with this - setting it too
		    // short causes truncated communications back to the
		    // PLC - I should really find a way to trigger it once the
		    // call co sdWrite is done.chOQIsEmptyI

		    while (!(oqIsEmptyI(&(&SD2)->oqueue)))
		    {
			//chprintf((BaseSequentialStream*)&SD1,".");
		    	chThdSleepMilliseconds(1);
		    }

		    chThdSleepMilliseconds(2);
		    palClearPad(GPIOA,4);
		    //chThdSleepMilliseconds(1);
		    //chprintf((BaseSequentialStream*)&SD1,"-");
		    //hprintf(&SD1,lcltext);
		    //skip_next = 0;
		}

	}

    }

void requestAngle(){
    char lcltext[32] = {0x01,0x03,0x00,0x00,0x00,0x02,0xc4,0x0b,0x00,0x00};

    palSetPad(GPIOB,9);
    sdWrite(&SD3,lcltext,8);
    while (!(oqIsEmptyI(&(&SD3)->oqueue)))
	{
	    
	    chThdSleepMilliseconds(1);
	}
    chThdSleepMilliseconds(2);
    palClearPad(GPIOB,9);
    
}

static THD_WORKING_AREA(waThread6, 2048);
static THD_FUNCTION(Thread6, arg) {
    int charnum;
    int index;
    msg_t key;
    int x;
    char *starttxt;
    char text[255];
    char lcltext[32];
    uint8_t command;
    int row;
    int col;
    int len;
    msg_t rxRow;
    msg_t rxPos;
    msg_t response;
    uint8_t skip_next;
    uint16_t reg;
    uint16_t value;

    while (TRUE)
	{
	    // the skip is because the way I have it hooked up right now
	    // causes it to read whatever we send.
	    chMBFetchTimeout(&RxMbx2,&rxRow,TIME_INFINITE);
	    rxPos = rxRow & 0xFF;
	    rxRow = rxRow >> 8;
	    memcpy(lcltext,rx2_text[rxRow],rxPos);
	    // if the message is for us and the CRC matches - otherwise -
	    // ignore.

            if ((lcltext[0] == 1) &&
		(*(uint16_t*)(lcltext+rxPos-2) == CRC16(lcltext,rxPos-2)))

		{
		    currentAngle = lcltext[3]*100+lcltext[4]+lcltext[5]*.01;
		    currentAngle = currentAngle - 180.0;

		 
		}

	}

    }



void adcSTM32EnableTSVREFE(void) {
    //    ADC12_COMMON->CCR |= ADC12_CCR_VREFEN;
    ADC34_COMMON->CCR |= ADC34_CCR_VREFEN;
}


float calc_volts(float vdd,int rawread)
{
    return (rawread/4095.0)*vdd;
}



int main(void) {
  unsigned i;
  float VDD;
  float amps;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  palSetPad(GPIOB, 5);
  wdgStart(&WDGD1, &wdgcfg);
  wdgReset(&WDGD1);
  chMBObjectInit(&RxMbx,&RxMbxBuff,MAILBOX_SIZE);
  chMBObjectInit(&RxMbx2,&RxMbx2Buff,MAILBOX_SIZE);
  palSetPadMode(GPIOE, 7, PAL_MODE_INPUT_ANALOG);

  adcStart(&ADCD3, NULL);
  // The referenece voltage enable MUST come after the adcStart 
  adcSTM32EnableTSVREFE();


  palSetPadMode(GPIOE, 5, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOE, 6, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOE,5);
  palClearPad(GPIOE,6);
  



  
  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));

  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));



  palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 8, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOB, 9, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 15, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 14, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palClearPad(GPIOA,4);
  palSetPad(GPIOB,9);




  
  sdStart(&SD1, &uartCfg);
  sdStart(&SD2, &uartCfg2);
  sdStart(&SD3, &uartCfg2);
    // chprintf((BaseSequentialStream*)&SD2,"Hello World 2\r\n");
  chprintf((BaseSequentialStream*)&SD1,"Hello World - I am # %d\r\n",my_address);
  palSetPadMode(GPIOA, 0, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL);        

  //palSetPadMode(GPIOB, 8, PAL_MODE_OUTPUT_PUSHPULL); 

  //palClearPad(GPIOB, 8);     /* Green.  */

  palSetPad(GPIOA,CS);
  palSetPad(GPIOA,CS2);






    


  //  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chprintf((BaseSequentialStream*)&SD1,"HelloA\r\n")  ;
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);
  chprintf((BaseSequentialStream*)&SD1,"HelloB\r\n")  ;
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);
  chprintf((BaseSequentialStream*)&SD1,"HelloC\r\n")  ;
  
  chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL);
  chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO, Thread6, NULL);

  uint32_t x,y;



  


  chprintf((BaseSequentialStream*)&SD1,"HelloD\r\n")  ;






  



  
  
  
  
  chThdSleepMilliseconds(500);
  chprintf((BaseSequentialStream*)&SD1,"\r\n");
  //  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  chprintf((BaseSequentialStream*)&SD1,"Point B\r\n");


  while (TRUE)
      {
	  if (abs(currentAngle) > 30)
	      {
		  if (currentAngle > 1){
		      palClearPad(GPIOE,5);
		      palSetPad(GPIOE,6);
		  }
		  else {
		      palSetPad(GPIOE,5);
		      palClearPad(GPIOE,6);
		  }
		      
		      
	      }
	  else
	      {
		  palClearPad(GPIOE,5);
		  palClearPad(GPIOE,6);
	      }
	  wdgReset(&WDGD1);
	  // datasheet RM0316 VDDA = 3.3 V ₓ VREFINT_CAL / VREFINT_DATA
	  // ADC3 wasn't working for vdd - I'm not sure why that is
	  adcConvert(&ADCD3, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);


	  chThdSleepMilliseconds(250);
	  VDD = 3.3 * (*(uint16_t*)0x1FFFF7BA) / (samples1[1] * 1.0);
	  amps = calc_volts(VDD,samples1[0])/.14;
	  step = (step +1)%100;
	  chprintf(&SD1,"Angle: %.2f amps: %.2f volts %.2f\r\n",currentAngle,amps,VDD);
	  requestAngle();
	  chThdSleepMilliseconds(500);
	  

   }



  return 0;
}
