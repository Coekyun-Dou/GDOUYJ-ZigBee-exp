#include <iocc2530.h>
#include "i2c.h"

// I2C IO Pins
// SDA on port p0.5
#define SDA_LOW()  (P0_5=0) // set SDA to low
#define SDA_HIGH() (P0_5=1) // set SDA to high
#define SDA_READ   (P0_5)   // read SDA
#define SDA_OUT()  (P0DIR |= (1<<5))
#define SDA_IN()   (P0DIR &= ~(1<<5))

// SCL on port p0.4
#define SCL_LOW()  (P0_4=0) // set SCL to low
#define SCL_HIGH() (P0_4=1) // set SCL to high
#define SCL_READ   (P0_4)   // read SCL
#define SCL_OUT()  (P0DIR |= (1<<4))
#define SCL_IN()   (P0DIR &= ~(1<<4))

//-- Static function prototypes -----------------------------------------------
static etError I2c_WaitWhileClockStreching(u8t timeout);
//-----------------------------------------------------------------------------
void DelayMicroSeconds(u32t xms)
{
   u32t i,j;

   for(i=xms;i>0;i--)
     for(j=8;j>0;j--)
      asm("nop");    //空语句消耗时钟
}

//-----------------------------------------------------------------------------
void I2c_Init(void)
{
  P0SEL &=~((1<<4)|(1<<5)); //P0.4 P0.5作为GPIO功能
  P0DIR |= (1<<4)|(1<<5);   //P0.4 P0.5设置为输出
  
  SDA_HIGH();                  // I2C-bus idle mode SDA released
  SCL_HIGH();                  // I2C-bus idle mode SCL released
}

//-----------------------------------------------------------------------------
void I2c_StartCondition(void)
{
  SDA_HIGH();
  DelayMicroSeconds(1);
  SCL_HIGH();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(10);  // hold time start condition (t_HD;STA)
  SCL_LOW();
  DelayMicroSeconds(10);
}

//-----------------------------------------------------------------------------
void I2c_StopCondition(void)
{
  SCL_LOW();
  DelayMicroSeconds(1);
  SDA_LOW();
  DelayMicroSeconds(1);
  SCL_HIGH();
  DelayMicroSeconds(10);  // set-up time stop condition (t_SU;STO)
  SDA_HIGH();
  DelayMicroSeconds(10);
}

//-----------------------------------------------------------------------------
etError I2c_WriteByte(u8t txByte)
{
  etError error = NO_ERROR;
  u8t     mask;
  for(mask = 0x80; mask > 0; mask >>= 1)// shift bit for masking (8 times)
  {
    if((mask & txByte) == 0) 
      SDA_LOW(); // masking txByte, write bit to SDA-Line
    else                     
      SDA_HIGH();
    DelayMicroSeconds(1);               // data set-up time (t_SU;DAT)
    SCL_HIGH();                         // generate clock pulse on SCL
    DelayMicroSeconds(5);               // SCL high time (t_HIGH)
    SCL_LOW();
    DelayMicroSeconds(1);               // data hold time(t_HD;DAT)
  }
  SDA_HIGH();                           // release SDA-line
  SCL_HIGH();                           // clk #9 for ack
  DelayMicroSeconds(1);                 // data set-up time (t_SU;DAT)
  if(SDA_READ) error = ACK_ERROR;       // check ack from i2c slave
  SCL_LOW();
  DelayMicroSeconds(20);                // wait to see byte package on scope
  return error;                         // return error code
}

//-----------------------------------------------------------------------------
etError I2c_ReadByte(u8t *rxByte, etI2cAck ack, u8t timeout)
{
  etError error = NO_ERROR;
  u8t mask;
  *rxByte = 0x00;
  SDA_HIGH();                            // release SDA-line
  for(mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
  { 
    SCL_HIGH();                          // start clock on SCL-line
    DelayMicroSeconds(1);                // clock set-up time (t_SU;CLK)
    error = I2c_WaitWhileClockStreching(timeout);// wait while clock streching
    DelayMicroSeconds(3);                // SCL high time (t_HIGH)
    if(SDA_READ) *rxByte |= mask;        // read bit
    SCL_LOW();
    DelayMicroSeconds(1);                // data hold time(t_HD;DAT)
  }
  if(ack == ACK)
    SDA_LOW();              // send acknowledge if necessary
  else
    SDA_HIGH();
  DelayMicroSeconds(1);                  // data set-up time (t_SU;DAT)
  SCL_HIGH();                            // clk #9 for ack
  DelayMicroSeconds(5);                  // SCL high time (t_HIGH)
  SCL_LOW();
  SDA_HIGH();                            // release SDA-line
  DelayMicroSeconds(20);                 // wait to see byte package on scope
  
  return error;                          // return with no error
}

//-----------------------------------------------------------------------------
etError I2c_GeneralCallReset(void)
{
  etError error;
  
  I2c_StartCondition();
  error = I2c_WriteByte(0x00);
  if(error == NO_ERROR)
    error = I2c_WriteByte(0x06);
  
  return error;
}

//-----------------------------------------------------------------------------
static etError I2c_WaitWhileClockStreching(u8t timeout)
{
  etError error = NO_ERROR;
  
  while(SCL_READ == 0)
  {
    if(timeout-- == 0)
      return TIMEOUT_ERROR;
    DelayMicroSeconds(1000);
  }
  
  return error;
}
