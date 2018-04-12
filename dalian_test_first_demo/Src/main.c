/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ADT7420.h"
#include "Communication.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

__IO uint16_t uhADCxConvertedValue[3];

/* Buffer used for transmission */
char s[3];
char Temp[300];
char Temp1[300];
char aTxBuffer[300];
char buffer[100];

/*************  本地变量声明	**************/
#define Buf2_Max 600					  //串口2缓存长度
char Uart2_Buf[Buf2_Max];
uint8_t First_Int = 0;
double x;
double y;
double z;
/*************	本地函数声明	**************/
void CLR_Buf1(void);
void CLR_Buf2(void);
void CLR_Buf3(void);
void CLR_Buf4(void);
void CLR_Buf5(void);
void Set_ATE0(void);
void Set_ATE1(void);
uint8_t Find(char *a);
void UART2_SendString(char* s);
void Second_AT_Command(char *b, char *a, uint8_t wait_time);
void NBSet_Init(void);
void Wait_CSQ(void);
void Wait_CEREG(void);
void Connect_Server(void);
void USART2_Receive(void);
void Second_1A_AT(char *b, char *a, uint8_t wait_time);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle);
/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

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
    MX_USART3_UART_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(100);
    float temp[1];
    ADT7420_Init();
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if(HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    //若开启DMA中断的话，由于其请求频率太快，会阻塞其它的中断
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, 3) != HAL_OK)
    {
        Error_Handler();
    }

    NBSet_Init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_GPIO_WritePin(GPIOA, LD4_Pin, GPIO_PIN_SET);
        //为温度传感器提供3.3V的电源
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

        //获取三轴加速度值
        x = (((double)uhADCxConvertedValue[0] / 4096) * 3.3 - 1.6332) / 0.0065 - 1;
        y = (((double)uhADCxConvertedValue[1] / 4096) * 3.3 - 1.6332) / 0.0065;
        z = (((double)uhADCxConvertedValue[2] / 4096) * 3.3 - 1.6332) / 0.0065;

        //发送加速度信息给串口调试助手
        sprintf(buffer, "\nX axis value is %lfg\t", x);
        UART2_SendString(buffer);
        CLR_Buf2();
        sprintf(buffer, " Y axis value is %lfg\t", y);
        UART2_SendString(buffer);
        CLR_Buf2();
        sprintf(buffer, " Z axis value is %lfg\n", z);
        UART2_SendString(buffer);
        CLR_Buf2();

        //发送传感器数据给服务器
        ADT7420_GetTemperature(temp);
        sprintf(Temp, "Temperature:%5.3f", temp[0]);
        strcpy(aTxBuffer, "AT+NMGS=17,");
        for(int i = 0; i < 17; i++)
        {
            sprintf(s, "%x", Temp[i]);
            strcpy(aTxBuffer + 11 + i * 2, s);
        }
        UART2_SendString(aTxBuffer);
        UART2_SendLR();
        Second_AT_Command(aTxBuffer, "SENT", 3);
				HAL_Delay(1000);
        CLR_Buf4();
        CLR_Buf1();

        while(1)
        {
            CLR_Buf4();							//清除接收缓存区
            UART_SendString("AT+CSQ");				//发送AT+CSQ指令,查询信号强度
            UART_SendLR();
            HAL_UART_Receive(&huart3, (uint8_t*)Temp, 20, 4000);
            if(strstr(Temp, "+CSQ:") != NULL)	//如果不是+CSQ:99,99，并且收到+CSQ:代表有信号
            {
                strncpy(Temp1, Temp + 2, 7);
                break;
            }
            else
            {
                //Do Nothing
            }
        }
        strcpy(aTxBuffer, "AT+NMGS=7,");
        for(int i = 0; i < 7; i++)
        {
            sprintf(s, "%x", Temp1[i]);
            strcpy(aTxBuffer + 10 + i * 2, s);
        }
        UART2_SendString(Temp1);
        UART2_SendLR();
        UART2_SendString(aTxBuffer);
        UART2_SendLR();
        Second_AT_Command(aTxBuffer, "SENT", 3);
				HAL_Delay(1000);
        CLR_Buf5();
        CLR_Buf4();
        CLR_Buf1();

        sprintf(Temp, "x_axis:%8.3fg, y_axis:%8.3fg, z_axis:%8.3fg", x, y, z);
        strcpy(aTxBuffer, "AT+NMGS=52,");
        for(int j = 0; j < 52; j++)
        {
            sprintf(s, "%x", Temp[j]);
            strcpy(aTxBuffer + 11 + j * 2, s);
        }
        UART2_SendString(aTxBuffer);
        UART2_SendLR();
        Second_AT_Command(aTxBuffer, "SENT", 3);
        HAL_Delay(1000);
        CLR_Buf4();
        CLR_Buf1();


        HAL_GPIO_WritePin(GPIOA, LD4_Pin, GPIO_PIN_RESET);
        //重启一下温度传感器
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_Delay(2000);
    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3
                                         | RCC_PERIPHCLK_ADC;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

    ADC_ChannelConfTypeDef sConfig;

    /**Common config
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.NbrOfConversion = 3;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion = 1;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    //HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LD4_Pin | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PC3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : LD4_Pin */
    GPIO_InitStruct.Pin = LD4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PA9 PA10 */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*******************************************************************************
* 函数名  : USART2_Receiver
* 描述    : 串口1接收程序
* 输入    : 无
* 返回    : 无
* 说明    :
*******************************************************************************/
void USART2_Receive(void)
{
    uint8_t Res = 0;
    int i;
    HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 20, 5000);
    for(; i < 20; i++)
    {
        if(Uart2_Buf[i] == '\0')
        {
            break;
        }
        Res++;
    }
    Uart2_Buf[First_Int] = Res;  	  //将接收到的字符串存到缓存中
    First_Int = First_Int + Res;    //缓存指针向后移动
}


/*******************************************************************************
* 函数名 : Find
* 描述   : 判断缓存中是否含有指定的字符串
* 输入   :
* 输出   :
* 返回   : unsigned char:1 找到指定字符，0 未找到指定字符
* 注意   :
*******************************************************************************/

uint8_t Find(char *a)
{
    if(strstr(Uart2_Buf, a) != NULL)
        return 1;
    else
        return 0;
}

/*******************************************************************************
* 函数名 : Set_ATE0
* 描述   : 取消回显
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void Set_ATE0(void)
{
    Second_AT_Command("ATE0", "OK", 3);								//取消回显
}

/*******************************************************************************
* 函数名 : Set_ATE0
* 描述   : 开启回显
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void Set_ATE1(void)
{
    Second_AT_Command("ATE1", "OK", 3);								//取消回显
}

/*******************************************************************************
* 函数名 : CLR_Buf5
* 描述   :
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void CLR_Buf5(void)
{
    int k;
    for(k = 0; k < 300; k++) //将缓存内容清零
    {
        Temp1[k] = 0x00;
    }
}

/*******************************************************************************
* 函数名 : CLR_Buf4
* 描述   :
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void CLR_Buf4(void)
{
    int k;
    for(k = 0; k < 300; k++) //将缓存内容清零
    {
        Temp[k] = 0x00;
    }
}

/*******************************************************************************
* 函数名 : CLR_Buf3
* 描述   :
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void CLR_Buf3(void)
{
    uint16_t k;
    for(k = 0; k < Buf2_Max; k++) //将缓存内容清零
    {
        Uart2_Buf[k] = 0x00;
    }
    First_Int = 0;              //接收字符串的起始存储位置
}

/*******************************************************************************
* 函数名 : CLR_Buf2
* 描述   :
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void CLR_Buf2(void)
{
    int k;
    for(k = 0; k < 100; k++) //将缓存内容清零
    {
        buffer[k] = 0x00;
    }
}

/*******************************************************************************
* 函数名 : CLR_Buf1
* 描述   :
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void CLR_Buf1(void)
{
    int k;
    for(k = 0; k < 300; k++) //将缓存内容清零
    {
        aTxBuffer[k] = 0x00;
    }
}
/*******************************************************************************
* 函数名 : Second_AT_Command
* 描述   : 发送AT指令函数
* 输入   : 发送数据的指针、发送等待时间(单位：S)
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/

void Second_AT_Command(char *b, char *a, uint8_t wait_time)
{
    uint8_t i;
    char *c;
    c = b;
    CLR_Buf3();
    i = 0;
    while(i == 0)
    {
        if(!Find(a))
        {
            b = c;
            UART_SendString(b);
            UART_SendLR();
            HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 30, 3000);
            HAL_Delay(wait_time * 1000);
        }
        else
        {
            i = 1;
        }
    }
    CLR_Buf3();
}


/*******************************************************************************
* 函数名 : Second_AT_Command
* 描述   : 发送AT指令函数
* 输入   : 发送数据的指针、发送等待时间(单位：S)
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/

void Second_1A_AT(char *b, char *a, uint8_t wait_time)
{
    uint8_t i;
    char *c;
    c = b;
    CLR_Buf3();
    i = 0;
    while(i == 0)
    {
        if(!Find(a))
        {
            b = c;
            UART_SendString(b);
            UART_SendLR();
            HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 600, 3000);
            HAL_Delay(wait_time * 1000);
        }
        else
        {
            i = 1;
        }
    }
    CLR_Buf3();
}

/*******************************************************************************
* 函数名  : UART_SendString
* 描述    : USART3发送字符串
* 输入    : *s字符串指针
* 输出    : 无
* 返回    : 无
* 说明    : 无
*******************************************************************************/
void UART_SendString(char* s)
{
    while(*s)
    {
        if(HAL_UART_Transmit(&huart3, (uint8_t*)s++, 1, 5000) != HAL_OK) //发送当前字符
        {
            Error_Handler();
        }
    }
}


/*******************************************************************************
* 函数名  : UART2_SendString
* 描述    : USART2发送字符串
* 输入    : *s字符串指针
* 输出    : 无
* 返回    : 无
* 说明    : 无
*******************************************************************************/
void UART2_SendString(char* s)
{
    while(*s)
    {
        if(HAL_UART_Transmit(&huart2, (uint8_t*)s++, 1, 100) != HAL_OK) //发送当前字符
        {
            Error_Handler();
        }
    }
}

/*******************************************************************************
* 函数名 : Wait_CSQ
* 描述   : 等待模块收到信号
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void Wait_CSQ(void)
{
    while(1)
    {
        CLR_Buf3();							//清除接收缓存区
        UART_SendString("AT+CSQ");				//发送AT+CSQ指令,查询信号强度
        UART_SendLR();
        HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 20, 4000);
        if(strstr(Uart2_Buf, "+CSQ:99,99") != NULL)	//返回99代表没有信号
        {
            continue;
        }
        else if(strstr(Uart2_Buf, "+CSQ:") != NULL)	//如果不是+CSQ:99,99，并且收到+CSQ:代表有信号
        {
            break;
        }
        else
        {
            //Do Nothing
        }
    }

}

/*******************************************************************************
* 函数名 : Wait_CREG
* 描述   : 等待模块注册成功
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void Wait_CEREG(void)
{
    uint8_t i = 0;
    int k;
    //CLR_Buf3();
    /*1
    while(i == 0)
    {
    	  CLR_Buf2();
    	  UART_SendString("AT+CREG?");
      UART_SendLR();
    	  HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 20, 5000);
    	  HAL_Delay(2000);
        for(k=0;k<Buf2_Max;k++)
    	{
    		   if(Uart2_Buf[k] == ':')
    		   {
    			    if((Uart2_Buf[k+4] == '1')||(Uart2_Buf[k+4] == '5'))
    			    {
    				     i = 1;
    			       break;
    			    }
    		   }
    	  }
    	//UART_SendString("Register.....\r\n");
    }
    */

    /*2
    while(i == 0)
    {
    	  CLR_Buf3();
    	  UART_SendString("AT+CEREG?");
      UART_SendLR();
    	  HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 20, 5000);
        for(k=0;k<Buf2_Max;k++)
    	{
    			    if((Uart2_Buf[k] == '1')||(Uart2_Buf[k] == '5'))
    			    {
    				     i = 1;
    			       break;
    			    }
    	  }
    	//UART_SendString("Register.....\r\n");
    }
    */

    while(i == 0)
    {
        CLR_Buf3();
        UART_SendString("AT+CEREG?");			//发送AT+CEREG?指令,查询是否注册到网络上
        UART_SendLR();
        HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 20, 4000);
        HAL_Delay(1000);
        for(k = 0; k < Buf2_Max; k++)
        {
            if((Uart2_Buf[k] == '1') || (Uart2_Buf[k] == '5'))
            {
                i = 1;
                break;
            }
        }
    }
}

/*******************************************************************************
* 函数名 : Connect_Server
* 描述   : GPRS连接服务器函数
* 输入   :
* 输出   :
* 返回   :
* 注意   :
*******************************************************************************/
void NBSet_Init(void)
{
    CLR_Buf3();
    UART2_SendString("Begin\r\n");
    //Second_AT_Command("AT","OK",1);
    //Second_AT_Command("AT+CFUN=0","OK",1);
    //Second_AT_Command("AT+NCDP=120.76.136.124","OK",1);
    //UART_SendString("AT+NRB");
    //HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 20, 2000);
    //HAL_UART_Receive(&huart3, (uint8_t*)Uart2_Buf, 20, 4000);
    //HAL_Delay(10000);
    Second_AT_Command("AT", "OK", 1);
    Wait_CSQ();
    Wait_CEREG();
    UART2_SendString("Register Scucess\n");
    Second_AT_Command("AT+NSMI=1", "OK", 1);
    Second_AT_Command("AT+NNMI=1", "OK", 1);
    UART2_SendString("NB-IoT Configure successfully!");
    CLR_Buf3();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
