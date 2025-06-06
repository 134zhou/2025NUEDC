#ifndef _ADF4531_H_
#define _ADF4531_H_

#include "main.h"
#include "spi.h"
#include "stdio.h"
#define ADF4351_R0			((uint32_t)0X2C8018)
#define ADF4351_R1			((uint32_t)0X8029)
#define ADF4351_R2			((uint32_t)0X10E42)
#define ADF4351_R3			((uint32_t)0X4B3)
#define ADF4351_R4			((uint32_t)0XEC803C)
#define ADF4351_R5			((uint32_t)0X580005)

#define ADF4351_R1_Base	((uint32_t)0X8001)
#define ADF4351_R4_Base	((uint32_t)0X8C803C)
#define ADF4351_R4_ON	  ((uint32_t)0X8C803C)
#define ADF4351_R4_OFF	((uint32_t)0X8C883C)

#define ADF4351_RF_OFF	((uint32_t)0XEC801C)

#define ADF4351_PD_ON		((uint32_t)0X10E42)
#define ADF4351_PD_OFF	((uint32_t)0X10E02)

void ADF4351_Init(void);
void ADF4351_Freq_Setting(float Fre); //MHz
#endif
