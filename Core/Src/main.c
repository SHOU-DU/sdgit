/***
	*************************************************************************************************
	*	@file  	main.c
	*	@version V1.0
	*  @date    2021-8-4
	*	@author  ���ͿƼ�
	*	@brief   SPI ���� W25Qxxϵ����غ���
   ************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32H7B0VBT6���İ� ���ͺţ�FK7B0M1-VBT6��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> ����˵����
	*
	*	1.����LED��ʹ��HAL���Դ�����ʱ����ʵ����˸
	*	2.ʹ��SPI6����W25Q64�����м򵥵Ķ�д����
   *  3. SPI6 SCK����ʱ������Ϊ 50MHz
	*
>>>>> ���ڴ�ӡ˵����
	*
	*	ʹ�� USART1 ��PA9/PA10����ӡ�����Ϣ�����ڲ�����115200
	*	
	************************************************************************************************
***/

#include "main.h"
#include "led.h"
#include "key.h"
#include "usart.h"
#include "spi_w25q64.h"

/********************************************** �������� *******************************************/

#define W25Qxx_NumByteToTest   	32*1024				// �������ݵĳ��ȣ�32K

volatile int32_t  SPI_Status ; 		 //	����־λ

volatile  uint32_t W25Qxx_TestAddr  =	0	;				// ���Ե�ַ
uint8_t  W25Qxx_WriteBuffer[W25Qxx_NumByteToTest];		//	д��������
uint8_t  W25Qxx_ReadBuffer[W25Qxx_NumByteToTest];		//	����������

//I got to smoke LiTangSummer
/***************************************************************************************************
*	�� �� ��: SPI_W25Qxx_Test
*	��ڲ���: ��
*	�� �� ֵ: SPI_W25Qxx_OK - ���Գɹ���ͨ��
*	��������: ���м򵥵Ķ�д���ԣ��������ٶ�
*	˵    ��: ��	
***************************************************************************************************/

int8_t SPI_W25Qxx_Test(void)		//Flash��д����
{
	uint32_t i = 0;	// ��������
	uint32_t ExecutionTime_Begin;		// ��ʼʱ��
	uint32_t ExecutionTime_End;		// ����ʱ��
	uint32_t ExecutionTime;				// ִ��ʱ��	
	float    ExecutionSpeed;			// ִ���ٶ�

// ���� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   
	
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	SPI_Status 			   = SPI_W25Qxx_BlockErase_32K(W25Qxx_TestAddr);	// ����32K�ֽ�
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime = ExecutionTime_End - ExecutionTime_Begin; // �������ʱ�䣬��λms
	
	if( SPI_Status == SPI_W25Qxx_OK )
	{
		printf ("\r\nW25Q64 �����ɹ�, ����32K�ֽ�����ʱ��: %d ms\r\n",ExecutionTime);		
	}
	else
	{
		printf ("\r\n ����ʧ��!!!!!  �������:%d\r\n",SPI_Status);
		while (1);
	}	

// д�� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	for(i=0;i<W25Qxx_NumByteToTest;i++)  //�Ƚ�����д������
	{
		W25Qxx_WriteBuffer[i] = i;
	}
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	SPI_Status				= SPI_W25Qxx_WriteBuffer(W25Qxx_WriteBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest); // д������
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 		// �������ʱ�䣬��λms
	ExecutionSpeed = (float)W25Qxx_NumByteToTest / ExecutionTime ; // ����д���ٶȣ���λ KB/S
	if( SPI_Status == SPI_W25Qxx_OK )
	{
		printf ("\r\nд��ɹ�,���ݴ�С��%d KB, ��ʱ: %d ms, д���ٶȣ�%.2f KB/s\r\n",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\nд�����!!!!!  �������:%d\r\n",SPI_Status);
		while (1);
	}		

// ��ȡ	>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
	
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms	
	SPI_Status				= SPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);	// ��ȡ����
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 					// �������ʱ�䣬��λms
	ExecutionSpeed = (float)W25Qxx_NumByteToTest/1024/1024 / ExecutionTime*1000 ; 	// �����ȡ�ٶȣ���λ MB/S 
	
	if( SPI_Status == SPI_W25Qxx_OK )
	{
		printf ("\r\n��ȡ�ɹ�,���ݴ�С��%d KB, ��ʱ: %d ms, ��ȡ�ٶȣ�%.2f MB/s \r\n",W25Qxx_NumByteToTest/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\n��ȡ����!!!!!  �������:%d\r\n",SPI_Status);
		while (1);
	}	

// ����У�� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   

	for(i=0;i<W25Qxx_NumByteToTest;i++)	//��֤�����������Ƿ����д�������
	{
		if( W25Qxx_WriteBuffer[i] != W25Qxx_ReadBuffer[i] )	//������ݲ���ȣ������ʧ��	
		{
			printf ("\r\n����У��ʧ��!!!!!��������λ��:%d\r\n",i);	

			while(1);
		}
	}		
	printf ("\r\nУ��ͨ��!!!!! SPI����W25Q64��������\r\n");		
	

// ��ȡ��ƬFlash�����ݣ����Բ����ٶ� >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	printf ("\r\n*****************************************************************************************************\r\n");		
	printf ("\r\n����Ĳ����У���ȡ�����ݱȽ�С����ʱ�̣ܶ���֮��������С��λΪms��������Ķ�ȡ�ٶ����ϴ�\r\n");		
	printf ("\r\n��������ȡ��Ƭflash���������Բ����ٶȣ������ó����ٶ����Ƚ�С\r\n");		
	printf ("\r\n��ʼ��ȡ>>>>\r\n");		
	ExecutionTime_Begin 	= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms		
	
	for(i=0;i<SPI_W25Qxx_FlashSize/(W25Qxx_NumByteToTest);i++)	// ÿ�ζ�ȡ W25Qxx_NumByteToTest �ֽڵ�����
	{
		SPI_Status		 = SPI_W25Qxx_ReadBuffer(W25Qxx_ReadBuffer,W25Qxx_TestAddr,W25Qxx_NumByteToTest);
		W25Qxx_TestAddr = W25Qxx_TestAddr + W25Qxx_NumByteToTest;		
	}
	ExecutionTime_End		= HAL_GetTick();	// ��ȡ systick ��ǰʱ�䣬��λms
	
	ExecutionTime  = ExecutionTime_End - ExecutionTime_Begin; 								// �������ʱ�䣬��λms
	ExecutionSpeed = (float)SPI_W25Qxx_FlashSize/1024/1024 / ExecutionTime*1000  ; 	// �����ȡ�ٶȣ���λ MB/S 

	if( SPI_Status == SPI_W25Qxx_OK )
	{
		printf ("\r\n��ȡ�ɹ�,���ݴ�С��%d MB, ��ʱ: %d ms, ��ȡ�ٶȣ�%.2f MB/s \r\n",SPI_W25Qxx_FlashSize/1024/1024,ExecutionTime,ExecutionSpeed);		
	}
	else
	{
		printf ("\r\n��ȡ����!!!!!  �������:%d\r\n",SPI_Status);
		while (1);
	}	
	
	return SPI_Status ;  	
}

/********************************************** �������� *******************************************/

void SystemClock_Config(void);

/***************************************************************************************************
*	�� �� ��: main
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: w25q64��д����
*	˵    ��: ��
****************************************************************************************************/

int main(void)
{	
	SCB_EnableICache();		// ʹ��ICache
	SCB_EnableDCache();		// ʹ��DCache
	HAL_Init();					// ��ʼ��HAL��	
	SystemClock_Config();	// ����ϵͳʱ�ӣ���Ƶ280MHz
	LED_Init();					// ��ʼ��LED����
	KEY_Init();					// ��ʼ����������
	USART1_Init();				// USART1��ʼ��	

	SPI_W25Qxx_Init();      // W25Qxx ��ʼ��

	SPI_W25Qxx_Test();		//Flash��д����

	while (1)
	{
		LED1_Toggle;
		HAL_Delay(100);
	}
}

/****************************************************************************************************/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 280000000 (CPU Clock)
  *            HCLK(Hz)                       = 280000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 1 (AHB  Clock  280MHz)
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  140MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  140MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  140MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  140MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 112
  *            PLL_P                          = 2
  *            PLL_Q                          = 2
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
/****************************************************************************************************/  
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 112;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
/***************************************** SPI6�ں�ʱ������ **************************************************/
/*** FANKE ***
>>>>> ����˵����
	*
	*	1.	SPI6 ��������������ʱ��Ϊ100M��������Բ��ģ�7B0�����ֲ� ��6.3.36.3С��  SPI interface characteristics
   *
	*	2. W25Q64JV��������������Ƶ����Ȼ��ֹ50MHz����ʹ�ö�ȡָ�� 0x03 �����������ٶ�ֻ�ܵ�50M
   *
	*	3. ���ｫ pll2_q_ck ����Ϊ 100M ��Ϊ SPI6 ���ں�ʱ�ӣ�Ȼ���پ���2��Ƶ�õ� 50M ��SCK����ʱ��
   *
*** FANKE ***/

   PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
  
   PeriphClkInitStruct.PLL2.PLL2M = 5;       // �����Ƶϵ��5(ע�����İ�ʹ�þ���Ϊ25M)
   PeriphClkInitStruct.PLL2.PLL2N = 100;     // ������Ԥ��Ƶ��ľ���ʱ�ӽ���100����Ƶ���õ�500Mʱ��
   PeriphClkInitStruct.PLL2.PLL2P = 2;       // ���ʱ���޹�SPI���û����������ú�ʹ��
   PeriphClkInitStruct.PLL2.PLL2R = 2;       // ���ʱ���޹�SPI���û����������ú�ʹ��
   PeriphClkInitStruct.PLL2.PLL2Q = 5;       // ����5��Ƶ���õ� 100M �� pll2_q_ck ʱ��
   PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
   PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
   PeriphClkInitStruct.PLL2.PLL2FRACN = 0;

   PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;	// �� pll2_q_ck ����ΪSPI6���ں�ʱ��
	
   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
   {
      Error_Handler();
   }  


}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
