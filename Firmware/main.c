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
size_t nx = 0, ny = 0;

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static uint8_t txbuf[10];
static uint8_t rxbuf[10];
static uint8_t step;
static uint8_t mode = 0; // 0 = auto 1 = manual
static float currentAngle1;
static float currentAngle2;
static float currentAngle;
static bool angleFlipFlop = false;
static float maxAmps = 0.0;
static float currentAmps;
static float lastAngle;
static float setPoint;

static mailbox_t RxMbx;
#define MAILBOX_SIZE 25
static msg_t RxMbxBuff[MAILBOX_SIZE];

static mailbox_t RxMbx2;
static msg_t RxMbx2Buff[MAILBOX_SIZE];

static mailbox_t SSMbx;
static msg_t SSMbxBuff[MAILBOX_SIZE];

static uint16_t stallSeconds =0;
static uint16_t strainSeconds =0;
static int16_t deg,speed,setpoint =0;
static float currentSpeed = 0.0;
static float speedThresh = .01;  // threshold to tell if it is moving or not
static uint16_t fastAmpsThresh = 120;  // threshold to tell if it is moving or not
static uint16_t slowAmpsThresh = 50;  // threshold to tell if it is moving or not
static uint16_t error;
static uint16_t hysterisisDeg = 20; // these are in integers so that we can
                                    // maybe set them via modbus later
static uint16_t stopDeg = 430;
static uint16_t stoptime = 66;
static uint16_t running;
static uint8_t startMove;
static uint8_t eastLimit;
static uint8_t westLimit;

static uint8_t goingEast;
static uint8_t goingWest;

static double deg2;
static char rx_text[32][32];
static char rx2_text[32][32];
static char btn_state[32];
static int rx_queue_pos=0;
static int rx_queue_num=0;
static int rx2_queue_pos=0;
static int rx2_queue_num=0;
static int rx3_queue_pos=0;


static uint8_t my_address = 0x10;




static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
  dbg('error!!');
}

static PWMConfig pwmcfg = {
  20000,                                    /* 10kHz PWM clock frequency.   */
  10,                                    /* Initial PWM period 1S.       */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};



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
	  
	  b = sdGetTimeout(&SD3,TIME_MS2I(2));

	  
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
	  
	  b = sdGetTimeout(&SD4,TIME_MS2I(3));

	  
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
    float tstFloat = 123.45;
    int row;
    int col;
    int len;
    msg_t rxRow;
    msg_t rxPos;
    msg_t response;
    uint8_t skip_next;
    uint16_t reg;
    int16_t value;
    while (TRUE)
	{
	    int16_t lclsetpoint;
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
		    palSetPad(GPIOD,10);
		    //chprintf((BaseSequentialStream*)&SD1,"+");
		    reg = (lcltext[2]<<8)|lcltext[3];
		    if ((command == 4)&&(reg==250))
			{
			    lcltext[0] = my_address;
			    lcltext[1] = 4;
			    lcltext[2] = 4;
			    lcltext[3] = ((uint32_t)tstFloat & 0xFF000000 ) >> 24;
			    lcltext[4] = ((uint32_t)tstFloat & 0xFF0000 ) >> 16;
			    lcltext[5] = ((uint32_t)tstFloat & 0xFF00 ) >> 8;
			    lcltext[6] = (uint32_t)tstFloat & 0xFF ;

			    *(uint16_t*)(lcltext+7) = CRC16(lcltext,7);
			    lcltext[9] = 0;
			    sdWrite(&SD3,lcltext,9);
			    
			}


		    if (command == 4)
			{

			    value = step;
			    if (reg==1)
				value = deg;
			    if (reg==2)
				value = speed;
			    if (reg==3)
				value = setPoint*10.0;

			    lcltext[0] = my_address;
			    lcltext[1] = 4;
			    lcltext[2] = 2;
			    lcltext[3] = (value & 0xFF00 ) >> 8;
			    lcltext[4] = value & 0xFF ;
			    *(uint16_t*)(lcltext+5) = CRC16(lcltext,5);
			    lcltext[7] = 0;
			    sdWrite(&SD3,lcltext,7);
			}
		    if (command == 6)
			{			    
			    reg = (lcltext[2]<<8)|lcltext[3];
			    if (reg==3)
				{
				    lclsetpoint = lcltext[4]<<8|lcltext[5];
				    setPoint = lclsetpoint / 10.0;
				    startMove = 1;
				}
			    if (reg==4)
				{
				    
				    error = 0;
				    
				}

			    sdWrite(&SD3,lcltext,rxPos);
			}



			
		    //chprintf((BaseSequentialStream*)&SD1,"Queue not empty %X %x %x\r\n",SD3.oqueue.q_counter,SD3.oqueue.q_rdptr,SD3.oqueue.q_wrptr);
		    //chprintf((BaseSequentialStream*)&SD1,"command %d - register %d, %d\r\n",lcltext[1],reg,value);
		    // I've been having problems with this - setting it too
		    // short causes truncated communications back to the
		    // PLC - I should really find a way to trigger it once the
		    // call co sdWrite is done.chOQIsEmptyI

		    while (!(oqIsEmptyI(&(&SD3)->oqueue)))
		    {
			//chprintf((BaseSequentialStream*)&SD1,".");
		    	chThdSleepMilliseconds(1);
		    }

		    chThdSleepMilliseconds(2);
		    palClearPad(GPIOD,10);
		    //chThdSleepMilliseconds(1);
		    //chprintf((BaseSequentialStream*)&SD1,"-");
		    //hprintf(&SD1,lcltext);
		    //skip_next = 0;
		}

	}

    }

void requestAngle(){
    char lcltext[32] = {0x01,0x03,0x00,0x00,0x00,0x02,0xc4,0x0b,0x00,0x00};
    char lcltext2[32] = {0x02,0x03,0x00,0x00,0x00,0x02,0xc4,0x38,0x00,0x00};


    palSetPad(GPIOC,12);
    angleFlipFlop  = !angleFlipFlop;
    if (angleFlipFlop)
	sdWrite(&SD4,lcltext,8);
    else
	sdWrite(&SD4,lcltext2,8);
    while (!(oqIsEmptyI(&(&SD4)->oqueue)))
	{
	    
	    chThdSleepMilliseconds(1);
	}
    chThdSleepMilliseconds(2);
    palClearPad(GPIOC,12);
    
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
    float lclAngle;
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
		    lclAngle = lcltext[3]*100+lcltext[4]+lcltext[5]*.01;
		    lclAngle = lclAngle - 180.0;
		    currentAngle1 = currentAngle1*.9 + lclAngle*.1;
		 
		}
            if ((lcltext[0] == 2) &&
		(*(uint16_t*)(lcltext+rxPos-2) == CRC16(lcltext,rxPos-2)))

		{
		    lclAngle = lcltext[3]*100+lcltext[4]+lcltext[5]*.01;
		    lclAngle = lclAngle - 180.0;
		    currentAngle2 = currentAngle2*.9 + lclAngle*.1;


		 
		}

	}

    }


static THD_WORKING_AREA(waThread7, 512);
static THD_FUNCTION(Thread7, arg) {
  (void)arg;
  int pass = 0;
  msg_t b = 0;
  char lcl_btn_state[32]; 
  chRegSetThreadName("serial3");
  while(TRUE)
      {
	  
	  b = sdGetTimeout(&SD2,TIME_MS2I(2));

	  if (rx3_queue_pos > 20) // something went wrong - we should only get 10
	      {
		  rx3_queue_pos = 0;
	      }
	  if ((b!= Q_TIMEOUT) && (rx3_queue_pos < 31))

	      {
		  
		  //chprintf((BaseSequentialStream*)&SD1,"got char: %x\r\n",b);
		  lcl_btn_state[rx3_queue_pos++]=b;
		  if ((b == 10) && (rx3_queue_pos > 0)){
		      
		      lcl_btn_state[rx3_queue_pos-1] =0;
		      strcpy(btn_state,lcl_btn_state);
		      rx3_queue_pos = 0;
		  }

		  
	      }
	  
	  
  
      }

  return MSG_OK;
}


static THD_WORKING_AREA(waThread8, 1024);
static THD_FUNCTION(Thread8, arg) {
    int x;
    int percent;
    while (TRUE)
	{
	    chMBFetchTimeout(&SSMbx,&x,TIME_INFINITE);
	    for (x=0;x<11;x++){
		percent = x*1000;
		if (percent < 5000)
		    percent = 5000;
		pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, percent));
		chThdSleepMilliseconds(100);
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

void stopTracker(void){
    palClearPad(GPIOE,5);
    palClearPad(GPIOE,6);
    //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    running =0;
    goingEast =0;
    goingWest =0;
}

void goEast(void){
    palSetPad(GPIOE,5);
    palClearPad(GPIOE,6);
    if (running == 0)
	chMBPostTimeout(&SSMbx,0,TIME_INFINITE); // let our mailbox know
    running =1;

    goingEast = 1;
    goingWest = 0;
}

void goWest(void){
    palClearPad(GPIOE,5);
    palSetPad(GPIOE,6);
    if (running==0)
	chMBPostTimeout(&SSMbx,0,TIME_INFINITE); // let our mailbox know
    running =1;
    goingEast = 0;
    goingWest = 1;

}


uint8_t encodePos(int pos){
    return pos + 32;
}


int main(void) {
  unsigned i;
  float VDD;
  
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  
  pwmStart(&PWMD1, &pwmcfg);
  palSetPad(GPIOB, 5);
  wdgStart(&WDGD1, &wdgcfg);
  wdgReset(&WDGD1);
  chMBObjectInit(&RxMbx,&RxMbxBuff,MAILBOX_SIZE);
  chMBObjectInit(&RxMbx2,&RxMbx2Buff,MAILBOX_SIZE);
  chMBObjectInit(&SSMbx,&SSMbxBuff,MAILBOX_SIZE);
  palSetPadMode(GPIOE, 7, PAL_MODE_INPUT_ANALOG); // current

  adcStart(&ADCD3, NULL);
  // The referenece voltage enable MUST come after the adcStart 
  adcSTM32EnableTSVREFE();

  palSetPadMode(GPIOE, 3, PAL_MODE_INPUT_PULLUP); // e/w limit sw
  palSetPadMode(GPIOE, 4, PAL_MODE_INPUT_PULLUP);

  palSetPadMode(GPIOE, 5, PAL_MODE_OUTPUT_PUSHPULL); // east west
  palSetPadMode(GPIOE, 6, PAL_MODE_OUTPUT_PUSHPULL);
  stopTracker();
  



  
  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));   // UART1 - Debug
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));

  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));  // UART2 - front panel
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  palSetPadMode(GPIOE, 9, PAL_MODE_ALTERNATE(2));   // PWM   

  palSetPadMode(GPIOD, 9, PAL_MODE_ALTERNATE(7));   // UART3 Loop  
  palSetPadMode(GPIOD, 8, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOD, 10, PAL_MODE_OUTPUT_PUSHPULL);

  palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(5));   // UART4 - Angle  
  palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(5));
  palSetPadMode(GPIOC, 12, PAL_MODE_OUTPUT_PUSHPULL);

  

  //palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetPadMode(GPIOB, 8, PAL_MODE_INPUT_PULLUP);
  //palSetPadMode(GPIOB, 9, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetPadMode(GPIOB, 15, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetPadMode(GPIOB, 14, PAL_MODE_OUTPUT_PUSHPULL);
  // palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL);

  
  //palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  // palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  //palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  //palClearPad(GPIOA,1);
  //palSetPad(GPIOB,9);




  
  sdStart(&SD1, &uartCfg);
  sdStart(&SD2, &uartCfg);
  sdStart(&SD3, &uartCfg2);
  sdStart(&SD4, &uartCfg2);
  
    // chprintf((BaseSequentialStream*)&SD2,"Hello World 2\r\n");
  chprintf((BaseSequentialStream*)&SD1,"Hello World - I am # %d\r\n",my_address);
  //  palSetPadMode(GPIOA, 0, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL);        

  //palSetPadMode(GPIOB, 8, PAL_MODE_OUTPUT_PUSHPULL); 

  palClearPad(GPIOD, 10);   // Clear tx/rx for UART3
  palClearPad(GPIOC, 12);   // Clear tx/rx for UART4

  //palSetPad(GPIOA,CS);
  //  palSetPad(GPIOA,CS2);






    


  //  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chprintf((BaseSequentialStream*)&SD1,"HelloA\r\n")  ;
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);
  chprintf((BaseSequentialStream*)&SD1,"HelloB\r\n")  ;
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);
  chprintf((BaseSequentialStream*)&SD1,"HelloC\r\n")  ;
  
  chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL);
  chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO, Thread6, NULL);
  chThdCreateStatic(waThread7, sizeof(waThread7), NORMALPRIO, Thread7, NULL);
  chThdCreateStatic(waThread8, sizeof(waThread8), NORMALPRIO, Thread8, NULL);

  uint32_t x,y;



  


  chprintf((BaseSequentialStream*)&SD1,"HelloD\r\n")  ;






  



  
  
  
  
  chThdSleepMilliseconds(500);
  chprintf((BaseSequentialStream*)&SD1,"\r\n");
  //  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  chprintf((BaseSequentialStream*)&SD1,"Point B\r\n");
  requestAngle();
  chThdSleepMilliseconds(500);
  requestAngle();
  chThdSleepMilliseconds(500);
  currentAngle = (currentAngle1 + currentAngle2)/2;
  setPoint = lastAngle = currentAngle;
  
  while (TRUE)
      {

	  wdgReset(&WDGD1);
	  currentAngle = (currentAngle1 + currentAngle2)/2;
	  westLimit = !(palReadPad(GPIOE,3));
	  eastLimit = !(palReadPad(GPIOE,4));
	  
	  if (error){
	      stopTracker();
	      
	  }
	  
	  if (goingEast && (currentAngle > setPoint)){
	      stopTracker();
	      chprintf(&SD2,"@%c%cStop \r\n",encodePos(1),encodePos(10));
	  }
	  if (goingWest && (currentAngle < setPoint)){
	      stopTracker();
	      chprintf(&SD2,"@%c%cStop \r\n",encodePos(1),encodePos(10));
	  }

	  if ((startMove)&&(currentAngle<setPoint)){
	      startMove = 0;
	      chprintf(&SD2,"@%c%cEast \r\n",encodePos(1),encodePos(10));
	      chprintf(&SD1,"Move East \r\n");
	      goEast();
	  }
	  if ((startMove)&&(currentAngle>setPoint)){
	      startMove = 0;
	      chprintf(&SD2,"@%c%cWest \r\n",encodePos(1),encodePos(10));
	      chprintf(&SD1,"Move West \r\n");

	      goWest();
	  }
	  if (goingWest && (currentAngle < -stopDeg/10.0)){
	      error = 1;
	      stopTracker();
	  }
	  if (goingEast && (currentAngle > stopDeg/10.0)){
	      error = 1;
	      stopTracker();
	  }

	  if (running && (fabs(currentSpeed) < speedThresh)){
	      stallSeconds = stallSeconds+1;
	  }
	  if (running && (fabs(currentSpeed) > speedThresh)){
	      stallSeconds = 0;
	  }
	  if (running && (stallSeconds > stoptime)){
	      error = 2;
	      stopTracker();
	      chprintf(&SD2,"@%c%cStall %d \r\n",encodePos(1),encodePos(10),error);
	  }

	  if (running && (currentAmps*10 > slowAmpsThresh)){
	      strainSeconds = strainSeconds+1;
	  }
	  if (running && (currentAmps*10 < slowAmpsThresh)){
	      strainSeconds = 0;
	  }
	  if (running && (strainSeconds > stoptime)){
	      error = 3;
	      stopTracker();
	      chprintf(&SD2,"@%c%cStrain %d \r\n",encodePos(1),encodePos(10),error);
	  }
	      
	if (running && (currentAmps*10 > fastAmpsThresh)){
	      error=4;
	      stopTracker();
	      chprintf(&SD2,"@%c%cOCurr %d \r\n",encodePos(1),encodePos(10),error);
	  }

	  if (goingEast && eastLimit){
	      error=5;
	      stopTracker();
	      chprintf(&SD2,"@%c%cELimit %d \r\n",encodePos(1),encodePos(10),error);
	  }
	  if (goingWest && westLimit){
	      error=6;
	      stopTracker();
	      chprintf(&SD2,"@%c%cWLimit %d \r\n",encodePos(1),encodePos(10),error);
	  }
		  
	      


	  // datasheet RM0316 VDDA = 3.3 V ₓ VREFINT_CAL / VREFINT_DATA
	  // ADC3 wasn't working for vdd - I'm not sure why that is
	  adcConvert(&ADCD3, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);


	  chThdSleepMilliseconds(100);
	  VDD = 3.3 * (*(uint16_t*)0x1FFFF7BA) / (samples1[1] * 1.0);
	  currentAmps = calc_volts(VDD,samples1[0])/.14;
	  if (currentAmps > maxAmps)
	      maxAmps = currentAmps;
	  step = (step +1)%100;
	  chprintf(&SD2,"@%c%cSP:%.2f  \r\n",encodePos(0),encodePos(0),setPoint);
	  chprintf(&SD2,"@%c%cA1:%.2f  \r\n",encodePos(1),encodePos(0),currentAngle1);
	  chprintf(&SD2,"@%c%cA2:%.2f  \r\n",encodePos(2),encodePos(0),currentAngle2);
	  chprintf(&SD2,"@%c%cA:%.2f  \r\n",encodePos(3),encodePos(0),maxAmps);
	  chprintf(&SD2,"@%c%c%d  \r\n",encodePos(2),encodePos(10),stallSeconds);
	  chprintf(&SD2,"@%c%c%s\r\n",encodePos(3),encodePos(8),btn_state);
	  //chprintf(&SD2,"@%c%cID:%d  \r\n",encodePos(0),encodePos(10),my_address);
	  chprintf(&SD2,"@%c%cSpd:%0.2f  \r\n",encodePos(0),encodePos(9),currentSpeed);
	  
	  if (strncmp(btn_state,"1111",4)==0){
	      chprintf(&SD2,"@%c%c1\r\d",encodePos(4),encodePos(2));
	      chprintf(&SD2,"@%c%c0\r\d",encodePos(4),encodePos(3));
	      chThdSleepMilliseconds(1000);
	      mode=1;
	  }
	  if ((btn_state[3] == '1')&&(mode==1)){
	      setPoint+=10;
	      startMove = 1;
	      if (setPoint> 45) setPoint = 45;
	  }
	  if ((btn_state[7] == '1')&&(mode==1)){
	      error =0;
	      maxAmps=0.0;
	      stallSeconds = 0;
	      chprintf(&SD2,"@%c%cReset \r\n",encodePos(1),encodePos(10),error);
		  
	  }

	  
	  if ((btn_state[1] == '1')&&(mode==1)){
	      setPoint-=10;
	      startMove = 1;
	      if (setPoint< -45) setPoint = -45;
	  }



	  if ((btn_state[0] == '1')&&(mode==1)){
	      setPoint+=1;
	      startMove = 1;
	      if (setPoint> 45) setPoint = 45;
	  }

	  if ((btn_state[2] == '1')&&(mode==1)){
	      setPoint-=1;
	      startMove = 1;
	      if (setPoint< -45) setPoint = -45;
	  }

	  
	  if ((btn_state[4] == '1')&&(mode==1))
	      mode = 0;
	  if (mode==1){
	      chprintf(&SD2,"@%c%c1\r\d",encodePos(4),encodePos(2));
	      chprintf(&SD2,"@%c%c0\r\d",encodePos(4),encodePos(3));
	  }	
	  else{
	      chprintf(&SD2,"@%c%c0\r\d",encodePos(4),encodePos(2));
	      chprintf(&SD2,"@%c%c1\r\d",encodePos(4),encodePos(3));
	  }


	  chprintf(&SD1,"Angle:%.2f amps:%.2f volts%.2f setPoint:%.2f startMove:%d run:%d strain:%d stall:%d error:%d E:%d W:%d EL:%d WL:%d\r\n",currentAngle,currentAmps,VDD,setPoint,startMove,running,strainSeconds,stallSeconds,error,goingEast,goingWest,eastLimit,westLimit);
	  speed = floor(currentAngle*10 - lastAngle*10);
	  currentSpeed = currentSpeed*0.9 + (currentAngle-lastAngle)*0.1;
	  deg = floor(currentAngle*10);

	  lastAngle = currentAngle;
	  requestAngle();
	  chThdSleepMilliseconds(100);
	  requestAngle();

   }



  return 0;
}
