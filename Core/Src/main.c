/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

IWDG_HandleTypeDef hiwdg1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
__IO float semiusDelayBase;
static uint8_t pul_sta=1,dir_sta=1,ena_sta=1;
static uint32_t set_speed = 700;//700r/min motor output without reducer
static uint32_t delay_time = 1000000;

static uint8_t Gate1_seed_num = 1, Gate2_seed_num = 1;
static uint32_t receive_seed_num = 0b0;
static uint8_t receive_usart1[USART_REC_LEN];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t *TxData;

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t *RxData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
void PY_semiusDelayTest(void);
void PY_Delay_semius_t(uint32_t Delay);
void PY_semiusDelayOptimize(void);
void PY_Delay_semius(uint32_t Delay);

void motor_init();

uint32_t speed2delay_sus(uint32_t speed);//speed r/min

void dif_fluoresent_seed(uint8_t num_Light_Gate2);

uint8_t FDCAN1_Send_Msg(uint8_t * msg,uint32_t len);
uint8_t FDCAN1_Receive_Msg(uint8_t *buf);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define LED0RED(n) (n ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)) //led0
#define LED1GREEN(n) (n ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET)) //led1

#define KEY_UP HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) //KEY_UP  PA0
#define KEY0 HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_3) //KEY0  PH3
#define KEY1 HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2) //KEY1  PH2
#define KEY2 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) //KEY1  PC13

#define Light_Gate1 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_6) //Light_Gate1  PF6
#define Light_Gate2 HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7) //Light_Gate2  PF7

#define PUL_minus(n) (n ? HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_5,GPIO_PIN_RESET)) // Pulse signal
#define DIR_minus(n) (n ? HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET)) // Direction signal
#define ENA_minus(n) (n ? HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET): \
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_RESET)) // Enable signal

//#define select(n) ((n) > 1000 ? 1000 : (n))
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
  RetargetInit(&huart1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_IWDG1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  motor_init();
  PY_semiusDelayTest();
  PY_semiusDelayOptimize();
  printf("Motor control, pul_sta = %d, dir_sta = %d, ena_sta = %d\r\n",pul_sta,dir_sta,ena_sta);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_usart1, USART_REC_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_IWDG_Refresh(&hiwdg1);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_usart1, USART_REC_LEN);
//    pul_sta = !pul_sta;
//    PUL_minus(pul_sta);
//    PY_Delay_semius(speed2delay_sus(set_speed));
    uint8_t receive_can_data[8];
    if(FDCAN1_Receive_Msg(receive_can_data))
    {
        printf("CAN Receive data: %s\r\n",receive_can_data);
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_IT_TX_COMPLETE);
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{

  /* USER CODE BEGIN IWDG1_Init 0 */

  /* USER CODE END IWDG1_Init 0 */

  /* USER CODE BEGIN IWDG1_Init 1 */

  /* USER CODE END IWDG1_Init 1 */
  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg1.Init.Window = 4095;
  hiwdg1.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG1_Init 2 */

  /* USER CODE END IWDG1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)receive_usart1, USART_REC_LEN);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF6 PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PH2 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PI5 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_0: {//key_up high voltage enable
            if (KEY_UP == 1) {
            }
            break;
        }
        case GPIO_PIN_2: {//key1
            if (KEY1 == 0) {// plus velocity
               set_speed += 70;
                printf("speed up to %lu!\r\n",set_speed);
            }
            break;
        }
        case GPIO_PIN_3: {//key0
            if (KEY0 == 0) {// minus velocity
                if(set_speed > 70)
                    set_speed -= 70;
                else
                    set_speed = 0;
                printf("speed down to %lu!\r\n",set_speed);
            }
            break;
        }
        case GPIO_PIN_6: {
            if (Light_Gate1 == 0) {
                printf("No %d passed! \r\n",Gate1_seed_num);
                ++Gate1_seed_num;
                if(Gate1_seed_num > 32)
                {
                    Gate1_seed_num = 1;
                }
            }
            break;
        }case GPIO_PIN_7: {
            if (Light_Gate2 == 0) {
                printf("No %d passed\r\n",Gate2_seed_num);
                if(Gate2_seed_num > Gate1_seed_num){
                    printf("Error, Gate2_seed_num > Gate1_seed_num\r\n");
                    break;
                }
                dif_fluoresent_seed(Gate2_seed_num);
                ++Gate2_seed_num;
                if(Gate2_seed_num > 32)
                {
                    Gate2_seed_num = 1;
                    receive_seed_num=0b0;
                }
            }
            break;
        }
        case GPIO_PIN_13: {//key2
            if (KEY2 == 0) {// start/pause the motor,disable
                printf("key2 is touched, motor status changes\r\n");
                ena_sta = !ena_sta;
                ENA_minus(ena_sta);
                LED0RED(ena_sta);
                LED1GREEN(!ena_sta);
                printf("ena_sta is %d\r\n",ena_sta);
            }
            break;
        }
        default:
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance==USART1)//如果是串口1
    {
        uint8_t temp_num=0;
        if(receive_usart1[2] == 0xd && receive_usart1[0]>='0' && receive_usart1[0]<='9'
            &&receive_usart1[1]>= '0' && receive_usart1[1]<='9')//如果接收到的数据是回车
        {
            receive_usart1[2] = '\0';
            printf("receive_usart1 = %s\r\n",receive_usart1);
            temp_num = (receive_usart1[0]-'0')*10 + (receive_usart1[1]-'0');

            if(temp_num<=32 && temp_num>0)
            {
                if(FDCAN1_Send_Msg(receive_usart1,USART_REC_LEN))
                {
                    printf("send data to CAN1 success!\r\n");
                } else
                {
                    printf("send data to CAN1 failed!\r\n");
                }
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)receive_usart1, USART_REC_LEN);
    }

}
uint32_t speed2delay_sus(uint32_t speed)//speed 1r/min == 1/60000r/ms, 1ms = 2 * 1000 sus
{
    if(speed==0)
    {
        return 1000000;
    }
    uint16_t step = 400; //check figure, current status is sw5 off, sw6 on, sw7 on, sw8 on.
    delay_time =(2000.0 * 60000.0/(step*speed) + 0.5);
    if(delay_time<3)
        delay_time = 3;
    return delay_time;
}

void motor_init()
{
    PUL_minus(pul_sta);
    DIR_minus(dir_sta);
    ENA_minus(ena_sta);
}

void PY_semiusDelayTest(void)
{
    __IO uint32_t firstms, secondms;
    __IO uint32_t counter = 0;

    firstms = HAL_GetTick()+1;
    secondms = firstms+1;

    while(uwTick!=firstms) ;

    while(uwTick!=secondms) counter++;

    semiusDelayBase = ((float)counter)/2000;
}

void PY_Delay_semius_t(uint32_t Delay)
{
    __IO uint32_t delayReg;
    __IO uint32_t semiusNum = (uint32_t)(Delay*semiusDelayBase);

    delayReg = 0;
    while(delayReg!=semiusNum) delayReg++;
}

void PY_semiusDelayOptimize(void)
{
    __IO uint32_t firstms, secondms;
    __IO float coe = 1.0;

    firstms = HAL_GetTick();
    PY_Delay_semius_t(2000000) ;
    secondms = HAL_GetTick();

    coe = ((float)1000)/(secondms-firstms);
    semiusDelayBase = coe*semiusDelayBase;
}

void PY_Delay_semius(uint32_t Delay)
{
    __IO uint32_t delayReg;

    __IO uint32_t msNum = Delay/2000;
    __IO uint32_t semiusNum = (uint32_t)((Delay%2000)*semiusDelayBase);

    if(msNum>0) HAL_Delay(msNum);

    delayReg = 0;
    while(delayReg!=semiusNum) delayReg++;
}
void dif_fluoresent_seed(uint8_t num_Light_Gate2)
{
    uint8_t temp = 0b1;
    temp = temp<<num_Light_Gate2;
    if(receive_seed_num&temp)
    {
        printf("Match successfully!\r\n");//use Spray valve here
    }
    else printf("Fail to match seed!\r\n");
}
uint8_t FDCAN1_Send_Msg(uint8_t * msg,uint32_t len)
{
    TxHeader.Identifier=0x12;                           //32位ID
    TxHeader.IdType=FDCAN_STANDARD_ID;                  //标准ID
    TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    TxHeader.DataLength=len;                            //数据长度
    TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    TxHeader.MessageMarker=0;

    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader,msg)!=HAL_OK) return 1;//发送
    return 0;
}
uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
{
    if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,buf)!=HAL_OK)return 0;//接收数据
    return RxHeader.DataLength>>16;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1) {
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
            Error_Handler();
        }
        uint8_t buf[8];
        uint8_t len;
        len = FDCAN1_Receive_Msg(buf);
        if (len) {
            printf("receive can success\r\n");
            uint8_t temp_num = 0;
            if (buf[2] == 0xd && buf[0] >= '0' && buf[0] <= '9'
                && buf[1] >= '0' && buf[1] <= '9')//如果接收到的数据是回车
            {
                buf[2] = '\0';
                printf("receive_can = %s\r\n", buf);
                temp_num = (buf[0] - '0') * 10 + (buf[1] - '0');

                if (temp_num <= 32 && temp_num > 0) {
                    receive_seed_num |= (0b1 << (temp_num - 1));
                    printf("receive_seed_num = %lu\r\n", receive_seed_num);
                }
            }
        }
    }
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
