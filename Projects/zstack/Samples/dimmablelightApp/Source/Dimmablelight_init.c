#include <ioCC2530.h>
#include "Dimmablelight.h"



// 占空比变量（0~100，对应亮度0~100%） 后续还要做一个映射把协调器传来的0-255映射到0-100上
unsigned char msDlCount=0;     // 红色占空比
unsigned char msBlueCount=0;  // 绿色占空比
unsigned char msRedCount=0;    // 蓝色占空比
unsigned char msGreenCount=0;   // PWM周期计数（0~100，对应100ms周期）


void DimLightRouterApp_InitDimLightHardware(void)
{
  // 初始化GPIO为输出模式（根据调光灯PDF文档）
  // P0.1, P0.6, P0.7 作为调光灯控制引脚
 P0DIR |= (0x01<<1) | (0x01<<6) | (0x01<<7);  // 设置P0.1/6/7为输出
 
 //关闭电磁锁，电磁锁不能常开会烧坏
P0SEL &= ~(0x01 << 4);
P0DIR |= (0x01 << 4);
P0_4 =0;


  // 定时器1初始化设置     定义定时器溢出的时间
    T1CC0L = 0xFA;      // 低字节
    T1CC0H = 0x00;      // 高字节，定时1ms   0xFA00
    T1CTL = 0x33;       // 通道0，不分频，up/down模式

    // 使能中断
    EA = 1;             // 开启全局中断
    T1IE = 1;          // 开启定时器1中断
  
}


/****************************************************************
定时器中断中控制调光灯IO输出状态
***********************************************************/
#pragma vector=T1_VECTOR
__interrupt void Timer1(void)
{
  ++msDlCount;
  msDlCount = (msDlCount >= 100) ? 0 : msDlCount;
  P0_1 = (msDlCount <= msBlueCount) ? 1 : 0;
  P0_6 = (msDlCount <= msRedCount) ? 1 : 0;
  P0_7 = (msDlCount <= msGreenCount) ? 1 : 0;
}



/*根据传入的数据进行灯光切换*/
void LED_change(unsigned char RGB_Blue,unsigned char RGB_Red,unsigned char RGB_Green)
{
  msBlueCount = RGB_Blue;
  msRedCount = RGB_Red;
  msGreenCount = RGB_Green;
}


//延时函数，以毫秒为单位
void DelayMilliseconds(unsigned long xms) {
    unsigned long i, j;
    for (i = xms; i > 0; i--) {
        for (j = 6000; j > 0; j--) {
            // 空操作，仅循环计数，不执行额外操作
        }
    }
}