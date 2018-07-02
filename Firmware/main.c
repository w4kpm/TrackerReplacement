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
static double z[2] = {0.0, 0.0};
static double x[3] = {0.0, 0.0, 0.0};
static double EYE3[3][3]={{1.0, 0,0},
			  {0,1.0,0},
			  {0,0,1.0}};

static double EYE2[2][2]={{1.0, 0},
			  {0,1.0}};


static double P[3][3] = {{1000, 0, 0},
			 {0, 1000, 0},
			 {0,0,1000}};
static double R[2][2] = {{2.77, 0},
			 { 0, 0.694}};
static double K[3][2] = {{0, 0},
			 { 0, 0},
			 { 0, 0}};

static double Q[3][3] = {{0.01, 0.00, 0.00},
			 {0.00, 0.01, 0.00},
			 {0.00, 0.00, 0.01}};

static double H[2][3] = {{1.00, 0.00, 0.00},
			 {0.00, 1.00, 0.00}};

static double F[3][3] = {{1.00, 0.00, 0.00},
			 {0.00, 1.00, 1.00},
			 {0.00, 0.00, 1.00}};
		 

double testm[3][2] = {{1,2},
		      {3,4},
		      {5,6}};
		    
double testm2[2][3] = {{1,2,3},
		       {4,5,6}};
// holding registers for kalman calulations    
static double result1[3][3];
static double result2[3][3];
static double result3[3][3];
static double result4[3][3];
static double result5[3][3];
		 


#define CS 0
#define CS2 1

#define CK 5
#define MISO 6
#define MOSI 7

#define a_lsb 0.061
#define g_lsb 0.00875

static uint8_t txbuf[2];
static uint8_t rxbuf[2];
static char text[255];


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
static char current_key;
static  uint16_t step =0;
static  int16_t deg,speed =0;
static double deg2;
static char rx_text[32][32];
static int rx_queue_pos=0;
static int rx_queue_num=0;
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


// find the determinant of a 2x2 matrix
double det2(double *a)
{
    
    return a[0]*a[3] - a[1] * a[2];
}



int cofactor_pos [3][2] = {{1,2},
			   {0,2},
			   {0,1}};

				
double* cofactor(double* mat,double* cf,int row,int col)
{
    int cfp[2][2][2];
    int r,c;
    for (r=0;r<2;r++)
	for(c=0;c<2;c++)	   
	    {
		cf[r*2+c] = mat[cofactor_pos[row][r]*3+cofactor_pos[col][c]];
	    }
    return cf;
}

double oddeven(int pos)
{
    if (pos%2==1)
	return -1.0;
    return 1.0;
}


double det3(double* mat)
{
    int row,col;
    double cf[2][2];
    double det = 0;
    for (row=0;row<3;row++)
	    {
		//chprintf((BaseSequentialStream*)&SD1,"%.2f(%.2f)\r\n",oddeven(row+col)*mat[row*3+col],det2(cofactor(mat,&cf,row,col)));
	    det=det+(oddeven(row)*
		     mat[row*3]*
		     det2(cofactor(mat,&cf,row,0)));
	    }
    return det;
}


double* inversem2(double* mat,double* inversem)
{
    double det;
    det= 1.0/(det2(mat));
    int row,col;

    inversem[0*2+0]=mat[1*2+1]*det;
    inversem[1*2+1]=mat[0*2+0]*det;
    inversem[1*2+0]=mat[1*2+0]*det*-1.0;
    inversem[0*2+1]=mat[0*2+1]*det*-1.0;
		
    return inversem;
}


double* inversem3(double* mat,double* inversem)
{
    int row,col;
    double cf[2][2];
    double det;
    det = det3(mat);
    

    for (row=0;row<3;row++)
	for (col=0;col<3;col++)

	    {

		inversem[col*3+row] = ((oddeven(row+col)*
					det2(cofactor(mat,&cf,row,col)))/det);
	    }
    return inversem;
    
}


double* transpose(double* matrix,double* invm, int mrow, int mcol)
{
    // matrix and invm are assumed to be correct dimensions.
    // mrow and mcol are the dimensions of the original matrix
    int x,y;
    for(x=0;x<mrow;x++)
	for(y=0;y<mcol;y++)
	    invm[y*mrow+x] = matrix[x*mcol+y];
    return invm;
}


double* mmult(double* a,double* b,double* result, int arow,int acol,int brow, int bcol)
{
    int row,col,curpos;
    double* currcel;
    for (row=0;row<arow;row++)
	for (col=0;col<bcol;col++)
	    {
		currcel = &result[row*bcol+col];
		*currcel =0;
		for (curpos=0;curpos<brow;curpos++)
		    {
			*currcel = *currcel + b[curpos*bcol+col]*a[row*acol+curpos];
		    }
	    }
    return result;
}


double* madd(double* a,double* b,double* result, int rows,int cols)
{
    int row,col;
    for (row=0;row<rows;row++)
	for(col=0;col<cols;col++)
	    result[row*cols+col] = a[row*cols+col]+b[row*cols+col];
    return result;
}


double* msub(double* a,double* b,double* result, int rows,int cols)
{
    int row,col;
    for (row=0;row<rows;row++)
	for(col=0;col<cols;col++)
	    result[row*cols+col] = a[row*cols+col]-b[row*cols+col];
    return result;
}




void print_matrix(double* m,int rows,int cols,char* s)
{
    int x,y;
    chprintf((BaseSequentialStream*)&SD1,"matrix %s  %dx%d:\r\n",s,rows,cols);
    for (x=0;x<rows;x++)
      {
	  chprintf((BaseSequentialStream*)&SD1,"\r\n   ");
	  for(y=0;y<cols;y++)
	      chprintf((BaseSequentialStream*)&SD1,"%.7f ",m[x*cols+y]);
      }
    chprintf((BaseSequentialStream*)&SD1,"\r\n\r\n");
  
}




double magnitude (double a, double b)
{
    //return 1.0;
    return sqrt(a*a+b*b);
}


static THD_WORKING_AREA(waThread2, 2048);
static THD_FUNCTION(Thread2, arg) {
    uint8_t status;
    int lasttime = 0;
    double dt;
    double ticks = TIME_S2I(1) * 1.0;
  while (TRUE) {
    status = spi_read(&std_spicfg1,0x07);
    //status=1;
    //chprintf((BaseSequentialStream*)&SD1,"status %d \r\n",status);
    if (status != 0)
      {
	  dt = (chVTGetSystemTimeX()-lasttime)/ticks;
	  ///dt = .003;
	  //chprintf((BaseSequentialStream*)&SD1,"'time':%.4f, ",(chVTGetSystemTimeX()-lasttime)/ticks);
	  F[0][1] = dt;
	  F[1][2] = dt;
	  //print_matrix(&F,3,3,"F");
	lasttime  = chVTGetSystemTimeX();
	int16_t test;
	double ax;
	double ay;
	double az;
	double gx;
	double gy;
	double gz;

	ax = ((int16_t)(spi_read(&std_spicfg1,0x29)<<8|spi_read(&std_spicfg1,0x28))) * a_lsb ;
	ay = ((int16_t)(spi_read(&std_spicfg1,0x2b)<<8|spi_read(&std_spicfg1,0x2a))) * a_lsb;
	az = ((int16_t)(spi_read(&std_spicfg1,0x2d)<<8|spi_read(&std_spicfg1,0x2c)))* a_lsb;
	
	//chprintf((BaseSequentialStream*)&SD1,"accel:%.2f,%.2f,%.2f -mag %.2f ",ax,ay,az,360.0*(asin(ay/magnitude(ay,az)))/(2*3.1415));
	z[0] = 360.0*(asin(ay/magnitude(ay,az)))/(2*3.1415);
	//chprintf((BaseSequentialStream*)&SD1,"deg %.2f ",360.0*(asin(ay/magnitude(ay,az)))/(2*3.1415));

	gx = ((int16_t)(spi_read(&std_spicfg2,0x29)<<8|spi_read(&std_spicfg2,0x28))) * g_lsb;


	//gy = ((int16_t)(spi_read(&std_spicfg2,0x2b)<<8|spi_read(&std_spicfg2,0x2a)))* g_lsb;

	//gz = ((int16_t)(spi_read(&std_spicfg2,0x2d)<<8|spi_read(&std_spicfg2,0x2c))) * g_lsb;

	if (az < 0)
	    gx = gx*-1.0;

	z[1] = gx;
	//chprintf((BaseSequentialStream*)&SD1,"Z %f,%f\r\n",z[0],z[1]);
	//x = F*x ;
	mmult(&EYE3,mmult(&F,&x,&result1,3,3,3,1),&x,3,3,3,1);
	//print_matrix(&x,3,1,"xdot");
	//print_matrix(&z,2,1,"z");
	// P=(F*P*F') +Q
	//print_matrix(&F,3,3,"F");
	mmult(&F,&P,&result1,3,3,3,3);
	//print_matrix(&result1,3,3,"F*P");
	transpose(&F,&result2,3,3);
	//print_matrix(&result2,3,3,"F'");
	mmult(&result1,&result2,&result3,3,3,3,3);
	//print_matrix(&result3,3,3,"F*P*F'");
	madd(&result3,&Q,&P,3,3);
	//print_matrix(&P,3,3,"F*P*F' + Q");
	
	// K = P*H'*(H*P*H' +R)^-1;
	//---------------------------
	// result1 = P*H' result2 = H'
	mmult(&P,transpose(&H,result2,2,3),&result1,3,3,3,2);
	//print_matrix(&result1,3,2,"P*H'");
	//result3= H*P*H' result4 = (H*P*H' +R) result5 = (H*P*H'+R)^-1
	inversem2(madd(mmult(&H,&result1,&result3,2,3,3,2),&R,&result4,2,2),&result5);
	//print_matrix(&result3,2,2,"H*P*H'");
	//print_matrix(&result4,2,2,"(H*P*H'+R)");
	//print_matrix(&result5,2,2,"(H*P*H'+R)^-1");
	// K = P*H'*(H*P*H' +R)^-1;
	mmult(&result1,&result5,&K,3,2,2,2);
	//print_matrix(&K,3,2,"K");

	
	//x = x + K*(z-H*x);
	//---------------
	//result2 (z-H*x)
	msub(&z,mmult(&H,&x,&result1,2,3,3,1),&result2,2,1);
	//result1 = K*(z-H*x)
	mmult(&K,&result2,&result1,3,2,2,1);
	//x = x+K*(z-H*x);
	mmult(&EYE2,madd(&x,&result1,&result3,2,1),&x,2,2,2,1);
	//print_matrix(&x,3,1,"x");
	//print_matrix(&P,3,3,"P(2)");
	//P = (eye(3) - (K * H)) * P;
	//print_matrix(&K,3,2,"K");
	//print_matrix(&H,2,3,"H");
	mmult(&K,&H,&result1,3,2,2,3);
	//print_matrix(&result1,3,3,"K*H");
	msub(&EYE3,&result1,&result2,3,3);
	//print_matrix(&result2,3,3,"EYE3 - K*H");
	//print_matrix(&P,3,3,"P");
	mmult(&result2,&P,&result1,3,3,3,3);
	//print_matrix(&result1,3,3,"(EYE3 - K*H)*P");
	mmult(&EYE3,&result1,&P,3,3,3,3);
	//print_matrix(&P,3,3,"P");
	deg = x[0]*10;
	deg2 = x[0];
	speed = x[1];
	//	chprintf((BaseSequentialStream*)&SD1,"dt: %.4f deg %d orig %.2f, speed: %d\r\n",dt,deg,z[0],speed);
	chThdSleepMilliseconds(1);
      }

    

    chThdSleepMilliseconds(1);
    //siUnselect(&SPID1);
    // palClearPad(GPIOB, 5);
    //chThdSleepMilliseconds(100);
  }

  return MSG_OK;
    
}


static THD_WORKING_AREA(waThread3, 512);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  int pass = 0;
  msg_t b = 0;

  chRegSetThreadName("serial");
  while(TRUE)
      {
	  
	  b = sdGetTimeout(&SD2,TIME_MS2I(2));

	  
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


int main(void) {
  unsigned i;

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


  /*
   * SPI1 I/O pins setup.
   */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));

  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));    
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 8, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOB, 9, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOB, 15, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 14, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palClearPad(GPIOA,4);

  my_address = my_address | palReadPad(GPIOB,8);
  my_address = my_address | (palReadPad(GPIOB,9)<<1);

  
  sdStart(&SD1, &uartCfg);
  sdStart(&SD2, &uartCfg2);
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
  ////chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL);

  ////chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO, Thread6, NULL);  
  uint32_t x,y;



  

  //chThdSleepMilliseconds(500);
  chprintf((BaseSequentialStream*)&SD1,"HelloD\r\n")  ;

  spiAcquireBus(&SPID1);
  chprintf((BaseSequentialStream*)&SD1,"HelloE\r\n")  ;

  x = spi_read(&std_spicfg2,0x0f);
  chThdSleepMilliseconds(500);
  // should be 0xD4
  chprintf((BaseSequentialStream*)&SD1,"whoami 2  %x\r\n",x);
  x = spi_read(&std_spicfg1,0x0f);
  // should be 0x29
  chprintf((BaseSequentialStream*)&SD1,"whoami 1  %x\r\n",x);


  spi_read(&std_spicfg1,0x0f);
  spi_write(&std_spicfg2,0x20,0xcf);  // enable gyro
  spi_write(&std_spicfg2,0x23,0x10);  // set 500dps measurement





  
  spi_write(&std_spicfg1,0x24,0xf4);  // enable theromometer - magneto @100hz - hi res
  spi_write(&std_spicfg1,0x20,0x77);  // Read Accel at 100hz
  spi_write(&std_spicfg1,0x25,0x00);  // set to max 2gauss - best resolution
  spi_write(&std_spicfg1,0x26,0x00);  // take magneto out of lp mode & set to continuous

  
  chprintf((BaseSequentialStream*)&SD1,"Point A\r\n");


  mmult(&testm2,&testm,&result1,2,3,3,2);
  //print_matrix(&result1,2,2,"test1");
  mmult(&testm,&testm2,&result2,3,2,2,3);
  //print_matrix(&result2,3,3,"test2");
  
  
  
  
  chThdSleepMilliseconds(500);
  chprintf((BaseSequentialStream*)&SD1,"\r\n");
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  chprintf((BaseSequentialStream*)&SD1,"Point B\r\n");


  while (TRUE)
      {
	  wdgReset(&WDGD1);
	  step = (step +1)%100;
	  chprintf((BaseSequentialStream*)&SD1,"%d ",step);
	  chprintf((BaseSequentialStream*)&SD1,"deg %.2f \r\n",deg2);
	  if ((deg2 < 10) & (deg2 > -10)){
	      palClearPad(GPIOB,14);
	      palClearPad(GPIOB,15);
	  }
	      

	  if (deg2 > 10){	     
	      palSetPad(GPIOB,14);
	      palClearPad(GPIOB,15);
	  }
	  if (deg2 < -10){	     
	      palClearPad(GPIOB,14);
	      palSetPad(GPIOB,15);
	  }
		 

	  chThdSleepMilliseconds(250);
	  palSetPad(GPIOB, 5);
	  chThdSleepMilliseconds(250);
	  
	  palClearPad(GPIOB, 5);
   }



  return 0;
}
