#ifndef Dimmablelight_H
#define Dimmablelight_H

void DelayMilliseconds(unsigned long xms);
void DimLightRouterApp_InitDimLightHardware(void);
void LED_change(unsigned char RGB_Blue,unsigned char RGB_Red,unsigned char RGB_Green);
__interrupt void Timer1(void);
#endif