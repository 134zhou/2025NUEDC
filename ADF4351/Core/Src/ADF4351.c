#include "ADF4351.h"
void Write_ADF4351(uint32_t Reg)
{
	uint8_t buf[4];
	// �� 32 λ���ݷֳ� 4 ���ֽڣ�MSB ����
	buf[0] = (uint8_t)((Reg>>24)&(0X000000FF));
	buf[1] = (uint8_t)((Reg>>16)&(0X000000FF));
	buf[2] = (uint8_t)((Reg>>8) &(0X000000FF));
	buf[3] = (uint8_t)((Reg)&(0X000000FF));
	HAL_Delay(0);	
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi1, buf, 4, 100);
//	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_RESET);
}

void ADF4351_Init(void)
{
	Write_ADF4351(ADF4351_R0);
	Write_ADF4351(ADF4351_R1);
	Write_ADF4351(ADF4351_R2);
	Write_ADF4351(ADF4351_R3);
	Write_ADF4351(ADF4351_R4);
	Write_ADF4351(ADF4351_R5);
}

//void ADF4351_Freq_Setting(float Fre) //MHz
//{
//  uint16_t Fre_temp, N_Mul = 1, Mul_Core = 0;
//	uint16_t INT_Fre, Frac_temp, Mod_temp, i;
//	uint32_t W_ADF4351_R0 = 0, W_ADF4351_R1 = 0, W_ADF4351_R4 = 0;
//	float multiple;
//	
//	if(Fre < 35)
//		Fre = 35.0;
//	if(Fre > 4400)
//		Fre = 4400.0;
//	Mod_temp = 1000;
//	Fre = ((float)((uint32_t)(Fre*100)))/100;
//	
//	Fre_temp = Fre;
//	for(i = 0; i < 10; i++)
//	{
//		if(((Fre_temp*N_Mul) >= 2199.9) && ((Fre_temp*N_Mul) <= 4400.1))
//			break;
//		Mul_Core++;
//		N_Mul = N_Mul*2;
//	}
//	
//	multiple = (Fre*N_Mul)/25;		//25������Ƶ�ʣ�����100M�ο������Ĵ���4��Ƶ��25M���ࡣ���û�����Ϊ80M�ο����룬�轫25��Ϊ20��10M�ο����룬�轫25��Ϊ2.5���Դ����ơ�����
//	INT_Fre = (uint16_t)multiple;
//	Frac_temp = ((uint32_t)(multiple*1000))%1000;
//	while(((Frac_temp%5) == 0) && ((Mod_temp%5) == 0))
//	{
//		Frac_temp = Frac_temp/5;
//		Mod_temp = Mod_temp/5;
//	}
//	while(((Frac_temp%2) == 0)&&((Mod_temp%2) == 0))
//	{
//		Frac_temp = Frac_temp/2;
//		Mod_temp = Mod_temp/2;
//	}
//	
//	Mul_Core %= 7;
////	printf("%d\n",INT_Fre);
////	printf("%d\n",Frac_temp);
////	printf("%d\n",Mod_temp);
////	printf("%d\n",Mul_Core);
//	W_ADF4351_R0 = (INT_Fre<<15)+(Frac_temp<<3);
//	W_ADF4351_R1 = ADF4351_R1_Base + (Mod_temp<<3);
//	W_ADF4351_R4 = ADF4351_R4_ON + (Mul_Core<<20);
//	
//	Write_ADF4351(ADF4351_RF_OFF);
//	Write_ADF4351(W_ADF4351_R0);
//	Write_ADF4351(W_ADF4351_R1);
//	Write_ADF4351(W_ADF4351_R4);
//}
void ADF4351_Freq_Setting(float Fre) // MHz
{
    uint32_t N_Mul = 1, Mul_Core = 0;
    uint32_t INT_Fre, Frac_temp, Mod_temp = 2500;
    uint32_t Freq_KHz, PFD = 25000; // ����Ƶ��Ϊ25 MHz����λ kHz��
    uint64_t VCO_Freq_KHz;
    uint32_t i;
    uint32_t W_ADF4351_R0 = 0, W_ADF4351_R1 = 0, W_ADF4351_R4 = 0;

    // �޷�����
    if(Fre < 35.0f) Fre = 35.0f;
    if(Fre > 4400.0f) Fre = 4400.0f;

    // תΪ kHz ���������⸡�����
    Freq_KHz = (uint32_t)(Fre * 1000.0f);

    // ���Һ��ʵı�Ƶϵ����ʹ VCO Ƶ������ 2200~4400 MHz ��Χ��
    for(i = 0; i < 7; i++)
    {
        VCO_Freq_KHz = Freq_KHz * N_Mul;
        if(VCO_Freq_KHz >= 2199900 && VCO_Freq_KHz <= 4400100)
            break;
        N_Mul *= 2;
        Mul_Core++;
    }
    Mul_Core %= 7;

    // �����������ֺͷ�������
    uint64_t N = (VCO_Freq_KHz * (uint64_t)Mod_temp) / PFD;
    INT_Fre = N / Mod_temp;
    Frac_temp = N % Mod_temp;

    // ����Ĵ���ֵ
    W_ADF4351_R0 = (INT_Fre << 15) | (Frac_temp << 3);               // INT + FRAC
    W_ADF4351_R1 = ADF4351_R1_Base | (Mod_temp << 3);                 // MOD
    W_ADF4351_R4 = ADF4351_R4_ON | (Mul_Core << 20);                  // �����Ƶ��

    // д�Ĵ���
    Write_ADF4351(ADF4351_RF_OFF);
    Write_ADF4351(W_ADF4351_R0);
    Write_ADF4351(W_ADF4351_R1);
    Write_ADF4351(W_ADF4351_R4);
}
