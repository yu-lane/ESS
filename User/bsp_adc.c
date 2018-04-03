/**
  ******************************************************************************
  * �ļ�����: bsp_adc.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ����ADC��ѹ�ɼ��ײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_adc.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
__IO uint16_t ADC_ConvertedValue[ADC_NUMOFCHANNEL];
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ADC GPIO ��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
static void ADCx_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(ADC_CLK | ADC_GPIO_CLK, ENABLE);

	/* �� ADC IO�˿�ʱ�� 
	ADC_GPIO_APBxClock_FUN(ADC_GPIO_CLK, ENABLE );*/
	
	/* ���� ADC IO ����ģʽ */
	GPIO_InitStructure.GPIO_Pin = ADC_PIN1|ADC_PIN2|ADC_PIN3|ADC_PIN4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	
	/* ��ʼ�� ADC IO */
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);				
}

/**
  * ��������: ����ADC����ģʽ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
static void ADCx_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	
  DMA_InitTypeDef DMA_InitStructure;

	
	/* ��ADCʱ�� */
	ADC_APBxClock_FUN(ADC_CLK,ENABLE );
	/* ��DMAʱ�� */
	RCC_AHBPeriphClockCmd(ADC_DMA_CLK, ENABLE);
  
  /* ��λDMA������ */
	DMA_DeInit(ADC_DMA_CHANNEL);
	
	/* ���� DMA ��ʼ���ṹ�� */
	/* �����ַΪ��ADC ���ݼĴ�����ַ */
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(ADCx->DR));	
	/* �洢����ַ��ʵ���Ͼ���һ���ڲ�SRAM�ı��� */
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;	
	/* ����Դ�������� */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	
	/* ���ݳ��� */
	DMA_InitStructure.DMA_BufferSize = ADC_NUMOFCHANNEL;	
	/* ����Ĵ���ֻ��һ������ַ���õ��� */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	/* �洢����ַ�̶� */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 	
	/* �������ݴ�СΪ���֣��������ֽ� */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	
	/* �ڴ����ݴ�СҲΪ���֣����������ݴ�С��ͬ */
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	
	/* ѭ������ģʽ */
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
	/* DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ�� */
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	
	/* ��ֹ�洢�����洢��ģʽ����Ϊ�Ǵ����赽�洢�� */
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;	
	/* ��ʼ��DMA */
	DMA_Init(ADC_DMA_CHANNEL, &DMA_InitStructure);
	
	/* ʹ�� DMA ͨ�� */
	DMA_Cmd(ADC_DMA_CHANNEL , ENABLE);
  
	/* ADC ģʽ���� */
	/* ֻʹ��һ��ADC�����ڵ�ģʽ */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	
	/* ʹ��ɨ��ģʽ����ͨ����ҪҪ����ͨ������Ҫ */
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 
	/* ����ת��ģʽ */
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	/* �����ⲿ����ת��������������� */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	/* ת������Ҷ��� */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
	/* ת��ͨ��1�� */
	ADC_InitStructure.ADC_NbrOfChannel = ADC_NUMOFCHANNEL;			
	/* ��ʼ��ADC */
	ADC_Init(ADCx, &ADC_InitStructure);
	
	/* ����ADCʱ��ΪPCLK2��8��Ƶ����9MHz */
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	
	/* ���� ADC ͨ��ת��˳��Ϊ1����һ��ת��������ʱ��Ϊ55.5��ʱ������ */
	ADC_RegularChannelConfig(ADCx, ADC_CHANNEL1, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADCx, ADC_CHANNEL2, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADCx, ADC_CHANNEL3, 3, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADCx, ADC_CHANNEL4, 4, ADC_SampleTime_55Cycles5);
  
  /* ʹ��ADC DMA ���� */
	ADC_DMACmd(ADCx, ENABLE);
  
	/* ����ADC ������ʼת�� */
	ADC_Cmd(ADCx, ENABLE);
	
	/* ��ʼ��ADC У׼�Ĵ���   */
	ADC_ResetCalibration(ADCx);
	/*�ȴ�У׼�Ĵ�����ʼ����� */
	while(ADC_GetResetCalibrationStatus(ADCx));	
	/* ADC��ʼУ׼*/
	ADC_StartCalibration(ADCx);
	/*�ȴ�У׼��� */
	while(ADC_GetCalibrationStatus(ADCx));
	
	/* ����û�в����ⲿ����������ʹ���������ADCת��  */
	ADC_SoftwareStartConvCmd(ADCx, ENABLE);
}

/**
  * ��������: ADC��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void ADCx_Init(void)
{
	ADCx_GPIO_Config();
	ADCx_Mode_Config();	
}

void ADCx_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	ADC_InitTypeDef ADC_InitStructure;	
	
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	/* ���� ADC IO ����ģʽ */
	GPIO_InitStructure.GPIO_Pin = ADC_PIN1|ADC_PIN2|ADC_PIN3|ADC_PIN4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	/*������������Ϊ50MHz */   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	/* ��ʼ�� ADC IO */
	GPIO_Init(ADC_PORT, &GPIO_InitStructure);		

	/* ADC ģʽ���� */
	/* ֻʹ��һ��ADC�����ڵ�ģʽ */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	
	/* ʹ��ɨ��ģʽ����ͨ����ҪҪ����ͨ������Ҫ */
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 
	/* ����ת��ģʽ */
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	/* �����ⲿ����ת��������������� */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	/* ת������Ҷ��� */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
	/* ת��ͨ��1�� */
	ADC_InitStructure.ADC_NbrOfChannel = ADC_NUMOFCHANNEL;			
	/* ��ʼ��ADC */
	ADC_Init(ADCx, &ADC_InitStructure);
	
	/* ����ADCʱ��ΪPCLK2��8��Ƶ����9MHz */
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	
	/* ���� ADC ͨ��ת��˳��Ϊ1����һ��ת��������ʱ��Ϊ55.5��ʱ������ */
	ADC_RegularChannelConfig(ADCx, ADC_CHANNEL1, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADCx, ADC_CHANNEL2, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADCx, ADC_CHANNEL3, 3, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADCx, ADC_CHANNEL4, 4, ADC_SampleTime_55Cycles5);
}
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
