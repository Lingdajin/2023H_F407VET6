/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/BSP/LCD/lcd.h"
// #include "arm_math.h"
#include "../../Drivers/BSP/KEY/key.h"
#include "../../Drivers/BSP/WAVEFORM/wave.h"
//#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 1000	//采样点
#define SAMPLE_RATE 1000000	//采样率
#define DAC_SIZE 200	//DAC波形缓冲区大小
#define DAC_RATE 500000	//DAC波形输出率
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//void fft_process(float32_t* fft_in, float32_t* mag, float32_t* freq);
//void filter_test();

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buffer[SAMPLE_SIZE];	//ADC采样数据缓冲区
uint16_t adc_buffer_enhanced[SAMPLE_SIZE * 2];	//ADC采样数据增强缓冲区

volatile uint8_t AdcConvEnd = 0;		//ADC采样完成标志
volatile uint8_t Dac1ConvEnd = 0;		//DAC1单周期完成标志
volatile uint8_t Dac2ConvEnd = 0;		//DAC1单周期完成标志

volatile uint8_t dac1_adjust_flag = 0;
volatile uint8_t dac2_adjust_flag = 0;


unsigned char frq[2];  //存放计算出来的两个频率
unsigned char wave_type[2];  //存放两频率波形种类（0为正弦波，1为三角波）
uint16_t wave_type_vote[2][2] = {{0,0},{0,0}}; //存放两频率波形种类投票，
uint16_t adjust_count = 0;
uint16_t dac1_buffer[200];	//存放dac1所需样本
uint16_t dac2_buffer[200];	//存放dac1所需样本

uint8_t debounce_count = 0;	//消抖用

uint8_t frq1_update = 0; //频率更新标志，告诉DMA回调函数需要更新频率
uint8_t frq2_update = 0; //频率更新标志，告诉DMA回调函数需要更新频率

volatile uint8_t phase_adjust_flag = 0; //A'B'相位调整标志（提高要求二）
volatile uint8_t phase_adjust_already_flag = 0; //A'B'相位调整已完成标志（提高要求二）
double temp_double_dac2 = 0; //用于存放dac2的延迟时间
uint16_t dac_phase = 0; //A'相对于B'的相位


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim6);
//  lcd_init();
//	printf("hELLO!\r\n");

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start_IT(&htim5); //开启定时器5中断，用于消抖
  HAL_DMA_Start(&hdma_dac1, (uint32_t)&DAC1->DHR12R1, (uint32_t)dac1_buffer, 0);                          /* 配置DMA传输参数 */
  HAL_DMA_Start(&hdma_dac2, (uint32_t)&DAC1->DHR12R1, (uint32_t)dac2_buffer, 0);                          /* 配置DMA传输参数 */

	cal_2frqs(frq); //计算两个信号频率
  cal_2type_enhance(); //计算两频率波形种类
  
 	  //      printf("frq1 = %dkHz, frq2 = %dkHz\r\n", (frq[0] + 2) * 5, (frq[1] + 2) * 5); //打印两个频率
 	    //初始产生两分离信号
 	for(int i = 0; i < DAC_SIZE; i++){
 		dac1_buffer[i] = DAC_SIN[(2 * i) % 200 + (200 * frq[0])]; //将第一个正弦波的200个点放入dac1_buffer
 	 	dac2_buffer[i] = DAC_SIN[(2 * i) % 200 + (200 * frq[1])]; //将第二个正弦波的200个点放入dac2_buffer
 	}
 	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
 	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac1_buffer, (200), DAC_ALIGN_12B_R);
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
 	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)dac2_buffer, (200), DAC_ALIGN_12B_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t key = key_scan(); //扫描按键
    if(key == WKUP_EVENT){
      // HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
      phase_adjust_flag = ~phase_adjust_flag; //设置A'B'相位调整标志（提高要求二）
      phase_adjust_already_flag = 0; //清除A'B'相位调整已完成标志（提高要求二）
    }else if(key == KEY0_EVENT){
      // HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
      dac_phase -= 5; //增加A'相对于B'的相位
      printf("dac_phase = %d°\r\n", dac_phase); //打印当前相位
    }else if(key == KEY1_EVENT){
      // HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
      dac_phase += 5; //减少A'相对于B'的相位
      printf("dac_phase = %d°\r\n", dac_phase); //打印当前相位
    }

	  if(adjust_count < 10){  //相位每调整十次，更新一次频率
      adjust_count++;
      Dac1ConvEnd = 0;	//清除DAC1单周期完成标志
      while(Dac1ConvEnd == 0) //等待DAC1单周期完成标志，即到达0相位点
        ;
      if(!phase_adjust_flag){
        dac1_phase_adjust();
      }
      // HAL_Delay(1);
//	HAL_GPIO_WritePin(TST_GPIO_Port, TST_Pin, GPIO_PIN_SET);	//dac1相位校正后管脚置1
      Dac2ConvEnd = 0;	//清除DAC2单周期完成标志
      while(Dac2ConvEnd == 0) //等待DAC2单周期完成标志，即到达0相位点
        ;
      dac2_phase_adjust();
      HAL_Delay(3);
    }
    else{
      adjust_count = 0;
	    cal_2frqs(frq); //计算两个信号频率
      cal_2type_enhance(); //计算两频率波形种类

      Dac1ConvEnd = 0;	//清除DAC1单周期完成标志
      while(Dac1ConvEnd == 0) //等待DAC1单周期完成标志，即到达0相位点，准备更新数据
        ;
      frq1_update = 1; //设置频率更新标志，告诉DMA回调函数需要更新频率
      HAL_Delay(1);

      Dac2ConvEnd = 0;	//清除DAC1单周期完成标志
      while(Dac2ConvEnd == 0) //等待DAC1单周期完成标志，即到达0相位点，准备更新数据
        ;
      frq2_update = 1; //设置频率更新标志，告诉DMA回调函数需要更新频率
      HAL_Delay(1);
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5)
	{
		debounce_count++;	//每1ms自增一次
    // frq_update = 1; //设置频率更新标志，告诉其他任务需要更新频率
	}


}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	Dac1ConvEnd = 1;
  if(frq1_update) {
    frq1_update = 0; //清除频率更新标志
    do_dac1(&dac1_buffer[DAC_SIZE / 2]); //更新dac1_buffer的后半部分数据
  }
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  if(frq1_update){
	  do_dac1(&dac1_buffer[0]); //更新dac1_buffer的前半部分数据
  }
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
	Dac2ConvEnd = 1;
  if(frq2_update) {
    frq2_update = 0; //清除频率更新标志
    do_dac2(&dac2_buffer[DAC_SIZE / 2]); //更新dac1_buffer的后半部分数据
  }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
  if(frq2_update){
	  do_dac2(&dac2_buffer[0]); //更新dac1_buffer的前半部分数据
  }
}

void do_dac1(uint16_t * dac_buffer){
  if(wave_type[0] == 0){ //如果第一个频率是正弦波
    for(int i = 0; i < (DAC_SIZE / 2); i++){
      dac_buffer[i] = DAC_SIN[(2 * i) % 200 + (200 * frq[0])];
    }
  } else if(wave_type[0] == 1){ //如果第一个频率是三角波
    switch (frq[0])
    {
    case 0:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_10k[(2 * i) % 200];
      }
      break;
    case 1:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_15k[(2 * i) % 200];
      }
      break;
    case 2:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_20k[(2 * i) % 200];
      }
      break;
    case 3:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_25k[(2 * i) % 200];
      }
      break;
    case 4:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_30k[(2 * i) % 200];
      }
      break;
    case 5:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_35k[(2 * i) % 200];
      }
      break;
    case 6:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_40k[(2 * i) % 200];
      }
      break;
    case 7:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_45k[(2 * i) % 200];
      }
      break;
    case 8:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_50k[(2 * i) % 200];
      }
      break;
    case 9:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_55k[(2 * i) % 200];
      }
      break;
    case 10:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_60k[(2 * i) % 200];
      }
      break;
    case 11:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_65k[(2 * i) % 200];
      }
      break;
    case 12:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_70k[(2 * i) % 200];
      }
      break;
    case 13:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_75k[(2 * i) % 200];
      }
      break;
    case 14:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_80k[(2 * i) % 200];
      }
      break;
    case 15:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_85k[(2 * i) % 200];
      }
      break;
    case 16:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_90k[(2 * i) % 200];
      }
      break;
    case 17:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_95k[(2 * i) % 200];
      }
      break;
    case 18:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_100k[(2 * i) % 200];
      }
      break;

    default:
      break;
    }
  }
}

void do_dac2(uint16_t * dac_buffer){
  if(wave_type[1] == 0){ //如果第一个频率是正弦波
    for(int i = 0; i < (DAC_SIZE / 2); i++){
      dac_buffer[i] = DAC_SIN[(2 * i) % 200 + (200 * frq[1])];
    }
  } else if(wave_type[1] == 1){ //如果第一个频率是三角波
    switch (frq[1])
    {
    case 0:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_10k[(2 * i) % 200];
      }
      break;
    case 1:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_15k[(2 * i) % 200];
      }
      break;
    case 2:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_20k[(2 * i) % 200];
      }
      break;
    case 3:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_25k[(2 * i) % 200];
      }
      break;
    case 4:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_30k[(2 * i) % 200];
      }
      break;
    case 5:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_35k[(2 * i) % 200];
      }
      break;
    case 6:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_40k[(2 * i) % 200];
      }
      break;
    case 7:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_45k[(2 * i) % 200];
      }
      break;
    case 8:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_50k[(2 * i) % 200];
      }
      break;
    case 9:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_55k[(2 * i) % 200];
      }
      break;
    case 10:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_60k[(2 * i) % 200];
      }
      break;
    case 11:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_65k[(2 * i) % 200];
      }
      break;
    case 12:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_70k[(2 * i) % 200];
      }
      break;
    case 13:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_75k[(2 * i) % 200];
      }
      break;
    case 14:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_80k[(2 * i) % 200];
      }
      break;
    case 15:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_85k[(2 * i) % 200];
      }
      break;
    case 16:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_90k[(2 * i) % 200];
      }
      break;
    case 17:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_95k[(2 * i) % 200];
      }
      break;
    case 18:
      for(int i = 0; i < (DAC_SIZE / 2); i++){
        dac_buffer[i] = dac_tri_100k[(2 * i) % 200];
      }
      break;

    default:
      break;
    }
  }
}

double corr1000_200(uint16_t* data, uint16_t* mask) {
  double res = 0;
  short i;
  for(i = 0; i < SAMPLE_SIZE ; i++)
    res = res + (data[i] - 2048) * (mask[i%200] - 2048);//注意模版要减去直流偏置
  return res;
}

double corr1000_200_enhance(uint16_t* data, uint16_t* mask) {
  double res = 0;
  short i;
  for(i = 0; i < SAMPLE_SIZE * 2; i++)
    res = res + (data[i] - 2048) * (mask[i%200] - 2048);//注意模版要减去直流偏置
  return res;
}

//计算IQ分量的模的长度
double modulus(double I, double Q){
  double temp_double;
  temp_double = sqrt(I*I + Q*Q);
  return temp_double;
}

void cal_2frqs(unsigned char* frq)//计算参与叠加的两个正弦信号的频率
{
    double corr_sin_double;//用于存放C与所有19种正弦相关计算的结果
    double corr_cos_double;//用于存放C与所有19种余弦相关计算的结果
    double mag[19];//用于存放输入信号与所有19种信号相关计算的模
    double temp_double[19],sort_temp;

    unsigned char i,j,temp_char;

    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, SAMPLE_SIZE); //让ADC1去采集1000个数，存放到adc_buff数组里
	  while (!AdcConvEnd)                                   //等待转换完毕
		  ;
	  AdcConvEnd = 0;

    for(i = 0; i < 19 ; i++)
    {
        corr_sin_double = corr1000_200(adc_buffer, DAC_SIN + 200*i);
        corr_cos_double = corr1000_200(adc_buffer, DAC_COS + 200*i);
        mag[i] = modulus(corr_sin_double, corr_cos_double);
        temp_double[i] = mag[i];
    }
    //冒泡法对大小进行排序
    for (i = 0; i < 19 - 1; i++)
    {
        for (j = 0; j < 19 - i - 1; j++){
                if (temp_double[j] < temp_double[j+1]){
                        sort_temp = temp_double[j];
                        temp_double[j] = temp_double[j+1];
                        temp_double[j+1] = sort_temp;
                }
            }
    }
    //寻找最大值序号
    for (i = 0; i < 19; i++){
        if(mag[i] == temp_double[0])//找到最大数
            frq[0] = i;
        if(mag[i] == temp_double[1])//找到第二大数
            frq[1] = i;
    }
    if(frq[0] > frq[1]) {//把频率低的放在第一个位置，如果颠倒就换一下
        temp_char = frq[0];
        frq[0] = frq[1];
        frq[1] = temp_char;
    }
}

void cal_2type_enhance(){
  for(int i = 0; i < 5; i++){
    cal_2type(frq, wave_type); //计算两个信号波形类型
    if(wave_type[0] == 0){
      wave_type_vote[0][0]++; //如果第一个频率是正弦波，则投票+1
    }else if(wave_type[0] == 1){
      wave_type_vote[0][1]++; //如果第一个频率是三角波，则投票+1
    }

    if(wave_type[1] == 0){
      wave_type_vote[1][0]++; //如果第二个频率是正弦波，则投票+1
    }else if(wave_type[1] == 1){
      wave_type_vote[1][1]++; //如果第二个频率是三角波
    }
  }
  if(wave_type_vote[0][0] >= wave_type_vote[0][1]){ //如果第一个频率的正弦波投票数大于三角波，则认为第一个频率是正弦波
    wave_type[0] = 0;
  }else if(wave_type_vote[0][0] < wave_type_vote[0][1]){ //如果第一个频率的三角波投票数大于正弦波，则认为第一个频率是三角波
    wave_type[0] = 1;
  }

  if(wave_type_vote[1][0] >= wave_type_vote[1][1]){ //如果第二个频率的正弦波投票数大于三角波，则认为第二个频率是正弦波
    wave_type[1] = 0;
  }else if(wave_type_vote[1][0] < wave_type_vote[1][1]){ //如果第二个频率的三角波投票数大于正弦波，则认为第二个频率是三角波
    wave_type[1] = 1;
  }
  wave_type_vote[0][0] = 0; //清除第一个频率的正弦波投票数
  wave_type_vote[0][1] = 0; //清除第一个频率的三角波投票数
  wave_type_vote[1][0] = 0; //清除第二个频率的正弦波投票数
  wave_type_vote[1][1] = 0; //清除第二个频率的三角波投票数

  // printf("frq0 = %d, frq1 = %d\r\n", frq[0], frq[1]); //打印两个频率
  // printf("wave_type0 = %d, wave_type1 = %d\r\n", wave_type[0], wave_type[1]); //打印两个频率波形类型
}

void cal_2type(unsigned char* frq, unsigned char* wave_type){
  //判断A'类型
  double corr_sin_double_A1_Three;//用于存放A'与其三次谐波正弦相关计算的结果
  double corr_cos_double_A1_Three;//用于存放A'与其三次谐波余弦相关计算的结果
  double mag_A1_Three;//用于存放相关计算的模

  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer_enhanced, SAMPLE_SIZE * 2); //让ADC1去采集1000个数，存放到adc_buff数组里
	while (!AdcConvEnd)                                   //等待转换完毕
	  ;
	AdcConvEnd = 0;

  switch (frq[0]) //根据基波频率对三次谐波做相关计算
  {
  case 0:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*4);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*4);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 1:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*7);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*7);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 2:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*10);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*10);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 3:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*13);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*13);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 4:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*16);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*16);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 5:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_105k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_105k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 6:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_120k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_120k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 7:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_135k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_135k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 8:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_150k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_150k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 9:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_165k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_165k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 10:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_180k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_180k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 11:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_195k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_195k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 12:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_210k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_210k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 13:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_225k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_225k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 14:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_240k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_240k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 15:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_255k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_255k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 16:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_270k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_270k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 17:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_285k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_285k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;
  case 18:
    corr_sin_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_300k);
    corr_cos_double_A1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_300k);
    mag_A1_Three = modulus(corr_sin_double_A1_Three, corr_cos_double_A1_Three);
    break;

  default:
    break;
  }

  double corr_sin_double_A1 = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*frq[0]);
  double corr_cos_double_A1 = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*frq[0]);
  double mag_A1 = modulus(corr_sin_double_A1, corr_cos_double_A1);
  // printf("frq0 = %dkHz, mag_A1 = %.3f, mag_A1_Three = %.3f, mag_A1/ mag_A1_Three = %.3f\r\n", (frq[0] + 2) * 5, mag_A1, mag_A1_Three, mag_A1 / mag_A1_Three);
  if(mag_A1_Three > mag_A1 / 9) { //如果三次谐波的模大于基波模的1/22，则认为是三角波
    wave_type[0] = 1; //三角波
    // printf("frq0 is triangle wave\r\n");
  }
  else {
    wave_type[0] = 0; //正弦波
    // printf("frq0 is sine wave\r\n");
  }

  //判断B'类型
  double corr_sin_double_B1_Three;//用于存放B'与其三次谐波正弦相关计算的结果
  double corr_cos_double_B1_Three;//用于存放B'与其三次谐波余弦相关计算的结果
  double mag_B1_Three;//用于存放相关计算的模
  
  switch (frq[1]) //根据基波频率对三次谐波做相关计算
  {
  case 0:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*4);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*4);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 1:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*7);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*7);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 2:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*10);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*10);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 3:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*13);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*13);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 4:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*16);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*16);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 5:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_105k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_105k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 6:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_120k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_120k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 7:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_135k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_135k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 8:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_150k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_150k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 9:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_165k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_165k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 10:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_180k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_180k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 11:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_195k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_195k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 12:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_210k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_210k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 13:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_225k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_225k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 14:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_240k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_240k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 15:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_255k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_255k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 16:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_270k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_270k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 17:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_285k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_285k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;
  case 18:
    corr_sin_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_300k);
    corr_cos_double_B1_Three = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_300k);
    mag_B1_Three = modulus(corr_sin_double_B1_Three, corr_cos_double_B1_Three);
    break;

  default:
    break;
  }

  double corr_sin_double_B1 = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*frq[1]);
  double corr_cos_double_B1 = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*frq[1]);
  double mag_B1 = modulus(corr_sin_double_B1, corr_cos_double_B1);
  // printf("frq1 = %dkHz, mag_B1 = %.3f, mag_B1_Three = %.3f, mag_B1 / mag_B1_Three = %.3f\r\n", (frq[1] + 2) * 5, mag_B1, mag_B1_Three, mag_B1 / mag_B1_Three);
  if(mag_B1_Three > mag_B1 / 9) { //如果三次谐波的模大于基波模的1/22，则认为是三角波
    wave_type[1] = 1; //三角波
    // printf("frq1 is triangle wave\r\n");
  }
  else {
    wave_type[1] = 0; //正弦波
    // printf("frq1 is sine wave\r\n");
  }

  //特殊情况特殊判断，根据五次谐波判断
  if(((frq[0] == 0)&&(frq[1] == 4))||((frq[0] == 1)&&(frq[1] == 7))||((frq[0] == 2)&&(frq[1] == 10))||((frq[0] == 3)&&(frq[1] == 13))||((frq[0] == 4)&&(frq[1] == 16))){
    double corr_sin_double_A1_Five;//用于存放A'与其五次谐波正弦相关计算的结果
    double corr_cos_double_A1_Five;//用于存放A'与其五次谐波余弦相关计算的结果
    double mag_A1_Five;//用于存放相关计算的模
    switch (frq[0]) //根据基波频率对五次谐波做相关计算
    {
    case 0:
      corr_sin_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*8);
      corr_cos_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*8);
      mag_A1_Five = modulus(corr_sin_double_A1_Five, corr_cos_double_A1_Five);
      break;
    case 1:
      corr_sin_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*13);
      corr_cos_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*13);
      mag_A1_Five = modulus(corr_sin_double_A1_Five, corr_cos_double_A1_Five);
      break;
    case 2:
      corr_sin_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, DAC_SIN + 200*18);
      corr_cos_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, DAC_COS + 200*18);
      mag_A1_Five = modulus(corr_sin_double_A1_Five, corr_cos_double_A1_Five);
      break;
    case 3:
      corr_sin_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_125k);
      corr_cos_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_125k);
      mag_A1_Five = modulus(corr_sin_double_A1_Five, corr_cos_double_A1_Five);
      break;
    case 4:
      corr_sin_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, dac_sin_150k);
      corr_cos_double_A1_Five = corr1000_200_enhance(adc_buffer_enhanced, dac_cos_150k);
      mag_A1_Five = modulus(corr_sin_double_A1_Five, corr_cos_double_A1_Five);
      break;

    default:
      break;
    }

    // printf("this is special circumstances!\r\nfrq0 = %dkHz, mag_A1 = %.3f, mag_A1_Five = %.3f, mag_A1 / mag_A1_Five = %.3f\r\n", (frq[0] + 2) * 5, mag_A1, mag_A1_Five, mag_A1 / mag_A1_Five);
    if(mag_A1_Five > mag_A1 / 27) { //如果五次谐波的模大于基波模的1/27，则认为是三角波
      wave_type[0] = 1; //三角波
      // printf("frq0 is triangle wave\r\n");
    }
    else {
      wave_type[0] = 0; //正弦波
      // printf("frq0 is sine wave\r\n");
    }

    //A'是三角波时，其三次谐波会影响到B'的判断，所以需要减去A'三次谐波后重新计算B'的基波模，并判断B'的类型
    if(wave_type[0] == 1){
      // printf("frq1 is affected by frq0, recalculating...\r\n");
      mag_B1 = mag_B1 - (mag_A1 / 9); //减去A'三次谐波的模
      if(mag_B1 < 0){
         mag_B1 = 0; //避免负数
      }
      if(mag_B1_Three > mag_B1 / 14) { //如果此时其三次谐波的模仍大于基波模的1/10，则认为是三角波
        wave_type[1] = 1; //三角波
        // printf("the real mag_B1 = %.3f, mag_B1_Three = %.3f, frq1 is triangle wave\r\n", mag_B1, mag_B1_Three);
      }
      else {
        wave_type[1] = 0; //正弦波
        // printf("the real mag_B1 = %.3f, mag_B1_Three = %.3f, frq1 is sine wave\r\n", mag_B1, mag_B1_Three);
      }
    }
  }
}

void dac1_phase_adjust(){
    //计算dac1需要延时的周期数
//	HAL_TIM_Base_Stop(&htim2);	//停止dac1传输定时器
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, SAMPLE_SIZE); //让ADC1去采集200个数，存放到adc_buff数组里
    while (!AdcConvEnd)                                   //等待转换完毕
    	;
    AdcConvEnd = 0;
    double temp_double_sin = corr1000_200(adc_buffer, DAC_SIN + 200*frq[0]);
    double temp_double_cos = corr1000_200(adc_buffer, DAC_COS + 200*frq[0]);
    double temp_double_angle = atan2(temp_double_cos, temp_double_sin);
    // int dac1_phase_adjust_clock = phase_adjust(temp_double_angle, 0);
    double pos_angle;//转换为延迟0-2pi，全部为整数的角度
	  uint16_t temp_short;
	  if(temp_double_angle < 0){//atan2函数计算得到的相位差在-pi到pi，但当其小于0时，DAC输出的信号无法提前相位，只能将所有相位全部延迟一个周期，即进行一个周期的求补操作
	      pos_angle = -temp_double_angle;
	  }
	  else{
	      pos_angle = 2*3.1415926 - temp_double_angle;
	  }
	  double temp_double = 4200*pos_angle / (3.1415926*((double)frq[0] + 2));
	  temp_short = temp_double + 0.5;//转换为整数的延迟时间，避免舍入误差

	  uint32_t temp_int = TIM2->CNT;
    // if(phase_adjust_flag){
    //   double temp_double_2 = 4200*(dac_phase + 215) / (180*((double)frq[1] + 2)) + temp_double;
    //   uint16_t temp_short_2 = temp_double_2 + 0.5;//转换为整数的延迟时间，避免舍入误差
    //   temp_int = temp_int + 4294967295 - temp_short_2;
    // }else{
      temp_int = temp_int + 4294967295 - temp_short;
    // }
	  
	  TIM2->CNT = temp_int;
//    TIM6->CNT = 65535 - temp_short;
//    HAL_TIM_Base_Start_IT(&htim6);	//开启延时控制定时器
}

void dac2_phase_adjust(){
    //计算dac2需要延时的周期数
//	  HAL_TIM_Base_Stop(&htim4);	//停止dac2传输定时器
    if(phase_adjust_flag){
      //重启dac1，强制其为0相位
      HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1); //停止dac1的DMA传输
      HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac1_buffer, (200), DAC_ALIGN_12B_R);
    }
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, SAMPLE_SIZE); //让ADC1去采集200个数，存放到adc_buff数组里
    while (!AdcConvEnd)                                   //等待转换完毕
    	;
    AdcConvEnd = 0;
    double temp_double_sin = corr1000_200(adc_buffer, DAC_SIN + 200*frq[1]);
    double temp_double_cos = corr1000_200(adc_buffer, DAC_COS + 200*frq[1]);
    double temp_double_angle = atan2(temp_double_cos, temp_double_sin);
    // int dac2_phase_adjust_clock = phase_adjust(temp_double_angle, 1);
    double pos_angle;//转换为延迟0-2pi，全部为整数的角度
    uint16_t temp_short;
	  if(temp_double_angle < 0){//atan2函数计算得到的相位差在-pi到pi，但当其小于0时，DAC输出的信号无法提前相位，只能将所有相位全部延迟一个周期，即进行一个周期的求补操作
	      pos_angle = -temp_double_angle;
	  }
	  else{
	      pos_angle = 2*3.1415926 - temp_double_angle;
	  }
	  double temp_double = 4200*pos_angle / (3.1415926*((double)frq[1] + 2));
	  temp_short = temp_double + 0.5;//转换为整数的延迟时间，避免舍入误差
	  uint32_t temp_int = TIM4->CNT;
	  temp_int = temp_int + 4294967295 - temp_short;
    TIM4->CNT = temp_int;
    if(phase_adjust_flag){ //如果需要进行相位调整，并且还没有进行过相位调整
      // temp_double_dac2 = temp_double; //保存dac2的延时
      double temp_double_2 = (4200*(dac_phase - 30) / (180*((double)frq[1] + 2))) + temp_double;
      uint16_t temp_short_2 = temp_double_2 + 0.5;//转换为整数的延迟时间，避免舍入误差
      uint32_t temp_int_2 = TIM2->CNT;
      temp_int_2 = temp_int_2 + 4294967295 - temp_short_2;
      TIM2->CNT = temp_int_2; //将dac1的延时也进行相应的调整
      phase_adjust_already_flag =1; //标记已经进行过相位调整
    } 
//    TIM7->CNT = 65535 - temp_short;
//    HAL_TIM_Base_Start_IT(&htim7);	//开启延时控制定时器
}

void dac1_phase_adjust_enhance(){
  //计算dac1需要延时的周期数
  
}




/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
