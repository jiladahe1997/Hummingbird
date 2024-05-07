/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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
 SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
uint8_t spi_dma_buffer[1024] = {0};
volatile bool spi_dma_tx_cmp_flag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define WORK_GPIO_GROUP           GPIOA
#define WORK_GPIO_PIN             GPIO_PIN_9
#define LCD_RES_GPIO_GROUP        GPIOA         //PA1 LCD 复位控制 
#define LCD_RES_GPIO_PIN          GPIO_PIN_1
#define LCD_DC_GPIO_GROUP         GPIOA         //PA2 LCD DC引脚，高发发送数据，低发送命令
#define LCD_DC_GPIO_PIN           GPIO_PIN_2   
#define LCD_SPI_CS_GPIO_GROUP     GPIOA         //PA3 LCD SPI CS引脚
#define LCD_SPI_CS_GPIO_PIN       GPIO_PIN_3    
#define LCD_BCKLIGHT_GPIO_GROUP   GPIOA         //PA4 LCD 背光控制
#define LCD_BCKLIGHT_GPIO_PIN     GPIO_PIN_4
#define LCD_POWER_CTL_GPIO_GROUP  GPIOA
#define LCD_POWER_CTL_GPIO_PIN    GPIO_PIN_8


//常用颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40
#define BRRED 			     0XFC07
#define GRAY  			     0X8430
#define DARKBLUE      	 0X01CF
#define LIGHTBLUE      	 0X7D7C 
#define GRAYBLUE       	 0X5458
#define LIGHTGREEN     	 0X841F
#define LGRAY 			     0XC618
#define LGRAYBLUE        0XA651 
#define LBBLUE           0X2B12

enum LCD_WRITE_TYPE {
  LCD_WRITE_TYPE_COMMAND,
  LCD_WRITE_TYPE_DATA,
};
int lcd_write(uint8_t* data, uint16_t size, enum LCD_WRITE_TYPE type);
int utils_lcd_write_multiple_times(uint8_t* data, uint16_t size);
int lcd_set_address(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
extern const unsigned char gImage_1[40960];
extern const unsigned char gImage_2[40960];
extern const unsigned char gImage_3[40960];
extern const unsigned char gImage_4[40960];
extern const unsigned char gImage_5[40960];
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化LCD */

  /* 先随便发送一串数据，使得SCLK引脚拉高高 */
  uint8_t test=0x11;
  lcd_write(&test, 1, LCD_WRITE_TYPE_COMMAND);


  HAL_GPIO_WritePin(LCD_POWER_CTL_GPIO_GROUP, LCD_POWER_CTL_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_SPI_CS_GPIO_GROUP, LCD_SPI_CS_GPIO_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RES_GPIO_GROUP, LCD_RES_GPIO_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RES_GPIO_GROUP, LCD_RES_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_BCKLIGHT_GPIO_GROUP, LCD_BCKLIGHT_GPIO_PIN, GPIO_PIN_SET);
  HAL_Delay(100);

  //TODO 校验一下lcd_write的返回值
  // int ret;
  /* step 1:退出休眠 */
  uint8_t sleep_out=0x11;
  lcd_write(&sleep_out, 1, LCD_WRITE_TYPE_COMMAND);
  HAL_Delay(120);


  /* step 2:设置FRMCTR1 (B1h): Frame Rate Control (In normal mode/ Full colors) */
  uint8_t FRMCTR1 = 0xB1;
  uint8_t FRMCTR1_data1 = 0x05;
  uint8_t FRMCTR1_data2 = 0x3C;
  uint8_t FRMCTR1_data3 = 0x3C;
  lcd_write(&FRMCTR1, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&FRMCTR1_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCTR1_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCTR1_data3, 1, LCD_WRITE_TYPE_DATA);

  /* step 3:设置FRMCTR2 (B2h): Frame Rate Control (In Idle mode/ 8-colors) */
  uint8_t FRMCRT2 = 0xB2;
  uint8_t FRMCRT2_data1 = 0x05;
  uint8_t FRMCRT2_data2 = 0x3C;
  uint8_t FRMCRT2_data3 = 0x3C;
  lcd_write(&FRMCRT2, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&FRMCRT2_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCRT2_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCRT2_data3, 1, LCD_WRITE_TYPE_DATA);

  /* step 4:设置FRMCTR3 (B3h): Frame Rate Control (In Partial mode/ full colors)  */
  uint8_t FRMCRT3 = 0xB3;
  uint8_t FRMCRT3_data1 = 0x05;
  uint8_t FRMCRT3_data2 = 0x3C;
  uint8_t FRMCRT3_data3 = 0x3C;
  uint8_t FRMCRT3_data4 = 0x05;
  uint8_t FRMCRT3_data5 = 0x3C;
  uint8_t FRMCRT3_data6 = 0x3C;
  lcd_write(&FRMCRT3, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&FRMCRT3_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCRT3_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCRT3_data3, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCRT3_data4, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCRT3_data5, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&FRMCRT3_data6, 1, LCD_WRITE_TYPE_DATA);

  /* step 5:设置 INVCTR (B4h): Display Inversion Control */
  uint8_t INVCTR = 0xB4;
  uint8_t INVCTR_data1 = 0x03;
  lcd_write(&INVCTR, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&INVCTR_data1, 1, LCD_WRITE_TYPE_DATA);

  /* step 6:设置PWCTR1 (C0h): Power Control 1 */
  uint8_t PWCTR1 = 0xC0;
  uint8_t PWCTR1_data1 = 0x28;
  uint8_t PWCTR1_data2 = 0x08;
  uint8_t PWCTR1_data3 = 0x04;
  lcd_write(&PWCTR1, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&PWCTR1_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&PWCTR1_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&PWCTR1_data3, 1, LCD_WRITE_TYPE_DATA);

  /* step 7:设置PWCTR2 (C1h): Power Control 2 */
  uint8_t PWCTR2 = 0xC1;
  uint8_t PWCTR2_data1 = 0xC0;
  lcd_write(&PWCTR2, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&PWCTR2_data1, 1, LCD_WRITE_TYPE_DATA);

  /* step 8:设置PWCTR3 (C2h): Power Control 3 (in Normal mode/ Full colors) */
  uint8_t PWCTR3 = 0xC2;
  uint8_t PWCTR3_data1 = 0x0D;
  uint8_t PWCTR3_data2 = 0x00;
  lcd_write(&PWCTR3, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&PWCTR3_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&PWCTR3_data2, 1, LCD_WRITE_TYPE_DATA);

  /* step 9:设置 PWCTR4 (C3h): Power Control 4 (in Idle mode/ 8-colors)  */
  uint8_t PWCTR4 = 0xC3;
  uint8_t PWCTR4_data1 = 0x8D;
  uint8_t PWCTR4_data2 = 0x2A;
  lcd_write(&PWCTR4, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&PWCTR4_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&PWCTR4_data2, 1, LCD_WRITE_TYPE_DATA);

  /* step 10:设置 PWCTR5 (C4h): Power Control 5 (in Partial mode/ full-colors)  */
  uint8_t PWCTR5 = 0xC4;
  uint8_t PWCTR5_data1 = 0x8D;
  uint8_t PWCTR5_data2 = 0xEE;
  lcd_write(&PWCTR5, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&PWCTR5_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&PWCTR5_data2, 1, LCD_WRITE_TYPE_DATA);

  /* step 10:设置 VMCTR1 (C5h): VCOM Control 1   */
  uint8_t VMCTR1 = 0xC5;
  uint8_t VMCTR1_data1 = 0x1A;
  lcd_write(&VMCTR1, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&VMCTR1_data1, 1, LCD_WRITE_TYPE_DATA);

  /* step 11:设置 MADCTL (36h): Memory Data Access Control  */
  uint8_t MADCTL = 0x36;
  uint8_t MADCTL_data1 = 0x70;
  lcd_write(&MADCTL, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&MADCTL_data1, 1, LCD_WRITE_TYPE_DATA);

  /* step 12:设置 GMCTRP1 (E0h): Gamma (�?+’polarity) Correction Characteristics Setting */
  uint8_t GMCTRP1 = 0xE0;
  uint8_t GMCTRP1_data1 =  0x04;
  uint8_t GMCTRP1_data2 =  0x22;
  uint8_t GMCTRP1_data3 =  0x07;
  uint8_t GMCTRP1_data4 =  0x0A;
  uint8_t GMCTRP1_data5 =  0x2E;
  uint8_t GMCTRP1_data6 =  0x30;
  uint8_t GMCTRP1_data7 =  0x25;
  uint8_t GMCTRP1_data8 =  0x2A;
  uint8_t GMCTRP1_data9 =  0x28;
  uint8_t GMCTRP1_data10 = 0x26;
  uint8_t GMCTRP1_data11 = 0x2E;
  uint8_t GMCTRP1_data12 = 0x3A;
  uint8_t GMCTRP1_data13 = 0x00;
  uint8_t GMCTRP1_data14 = 0x01;
  uint8_t GMCTRP1_data15 = 0x03;
  uint8_t GMCTRP1_data16 = 0x13;
  lcd_write(&GMCTRP1, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&GMCTRP1_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data3, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data4, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data5, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data6, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data7, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data8, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data9, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data10, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data11, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data12, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data13, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data14, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data15, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP1_data16, 1, LCD_WRITE_TYPE_DATA);

  /* step 13:设置 GMCTRN1 (E1h): Gamma �?-’polarity Correction Characteristics Setting */
  uint8_t GMCTRP2 = 0xE1;
  uint8_t GMCTRP2_data1 =  0x04;
  uint8_t GMCTRP2_data2 =  0x16;
  uint8_t GMCTRP2_data3 =  0x06;
  uint8_t GMCTRP2_data4 =  0x0D;
  uint8_t GMCTRP2_data5 =  0x2D;
  uint8_t GMCTRP2_data6 =  0x26;
  uint8_t GMCTRP2_data7 =  0x23;
  uint8_t GMCTRP2_data8 =  0x27;
  uint8_t GMCTRP2_data9 =  0x27;
  uint8_t GMCTRP2_data10 = 0x25;
  uint8_t GMCTRP2_data11 = 0x2D;
  uint8_t GMCTRP2_data12 = 0x3B;
  uint8_t GMCTRP2_data13 = 0x00;
  uint8_t GMCTRP2_data14 = 0x01;
  uint8_t GMCTRP2_data15 = 0x04;
  uint8_t GMCTRP2_data16 = 0x13;
  lcd_write(&GMCTRP2, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&GMCTRP2_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data3, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data4, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data5, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data6, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data7, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data8, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data9, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data10, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data11, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data12, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data13, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data14, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data15, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&GMCTRP2_data16, 1, LCD_WRITE_TYPE_DATA);

  /* step 14:设置 COLMOD (3Ah): Interface Pixel Format */
  uint8_t COLMOD = 0x3A;
  uint8_t COLMOD_data1 = 0x05;
  lcd_write(&COLMOD, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&COLMOD_data1, 1, LCD_WRITE_TYPE_DATA);

  /* step 15:设置 DISPON (29h): Display On */
  uint8_t DISPON=0x29;
  lcd_write(&DISPON, 1, LCD_WRITE_TYPE_COMMAND);

  /* step 16:清屏 */
  // lcd_set_address(0,0,160,128);
  // uint16_t* fill_data = malloc(128*160*sizeof(uint16_t)); //宽128像素 高160像素 每个像素16位/2字节颜色
  // if(fill_data==NULL)
  //   Error_Handler();
  
  // for(int i=0;i<128*160;i++){
  //   fill_data[i] = RED; //注意STM32是小端存储，低位存在高字节，所以这里需要转换一下
  //   uint8_t temp = fill_data[i] >> 8;
  //   fill_data[i] = (fill_data[i] << 8) | temp;
  // }
  // utils_lcd_write_multiple_times((uint8_t*)fill_data, 128*160*sizeof(uint16_t));
  // //HAL_Delay(200);
  // for(int i=0;i<128*160;i++){
  //   fill_data[i] = YELLOW;//注意STM32是小端存储，低位存在高字节，所以这里需要转换一下
  //   uint8_t temp = fill_data[i] >> 8;
  //   fill_data[i] = (fill_data[i] << 8) | temp;
  // }
  // utils_lcd_write_multiple_times((uint8_t*)fill_data, 128*160*sizeof(uint16_t));
  // //HAL_Delay(200);
  // for(int i=0;i<128*160;i++){
  //   fill_data[i] = BLUE;//注意STM32是小端存储，低位存在高字节，所以这里需要转换一下
  //   uint8_t temp = fill_data[i] >> 8;
  //   fill_data[i] = (fill_data[i] << 8) | temp;
  // }
  // utils_lcd_write_multiple_times((uint8_t*)fill_data, 128*160*sizeof(uint16_t));
  // //HAL_Delay(200);
  // for(int i=0;i<128*160;i++){
  //   fill_data[i] = WHITE;//注意STM32是小端存储，低位存在高字节，所以这里需要转换一下
  //   uint8_t temp = fill_data[i] >> 8;
  //   fill_data[i] = (fill_data[i] << 8) | temp;
  // }
  // utils_lcd_write_multiple_times((uint8_t*)fill_data, 128*160*sizeof(uint16_t));
  // free(fill_data);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count=0;
  while (1)
  {
    /* 工作指示 500ms 常闪 */
    HAL_GPIO_TogglePin(WORK_GPIO_GROUP, WORK_GPIO_PIN);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 渲染图片 */
    count++;   
    if(count <= 2){
      lcd_set_address(0,0,160,128);
      utils_lcd_write_multiple_times((uint8_t*)gImage_1, sizeof(gImage_1));
    }
    else if(count <= 4){
      lcd_set_address(0,0,160,128);
      utils_lcd_write_multiple_times((uint8_t*)gImage_2, sizeof(gImage_2));
    }
    else if(count <= 6){
      lcd_set_address(0,0,160,128);
      utils_lcd_write_multiple_times((uint8_t*)gImage_3, sizeof(gImage_3));
    }
    else if(count <= 8){
      lcd_set_address(0,0,160,128);
      utils_lcd_write_multiple_times((uint8_t*)gImage_4, sizeof(gImage_4));
    }
    else if(count <= 10){
      lcd_set_address(0,0,160,128);
      utils_lcd_write_multiple_times((uint8_t*)gImage_5, sizeof(gImage_5));
    }
    if(count >= 10) {
      count=0;
      /* Check and Clear the Wakeup flag */
      if (__HAL_PWR_GET_FLAG(PWR_FLAG_WU) != RESET)
      {
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
      }
      HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
      HAL_PWR_EnterSTANDBYMode();
    }
    HAL_Delay(500);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief 通过SPI接口，向LCD屏发送数�?
 * @param data 数据缓冲区，uint8指针类型
 * @param size 数据数量，不能超过DMA�?大长�?1024
 * @param type 类型，根据LCD定义，分为发送命令和数据
 * @return int 0代表成功，其他代表失�?
 */
int lcd_write(uint8_t* data, uint16_t size, enum LCD_WRITE_TYPE type){
  HAL_StatusTypeDef ret;


  // check param
  if(size > sizeof(spi_dma_buffer)){
    return -1;
  }

  memcpy(spi_dma_buffer, data, size);
  HAL_GPIO_WritePin(LCD_SPI_CS_GPIO_GROUP, LCD_SPI_CS_GPIO_PIN, GPIO_PIN_RESET);

  if(type == LCD_WRITE_TYPE_COMMAND)
    HAL_GPIO_WritePin(LCD_DC_GPIO_GROUP, LCD_DC_GPIO_PIN, GPIO_PIN_RESET);
  else if(type == LCD_WRITE_TYPE_DATA)
    HAL_GPIO_WritePin(LCD_DC_GPIO_GROUP, LCD_DC_GPIO_PIN, GPIO_PIN_SET);
  else 
    return -1;


  ret = HAL_SPI_Transmit_DMA(&hspi1, spi_dma_buffer, size);
  if(ret != HAL_OK) return -1;

  while(spi_dma_tx_cmp_flag==false);
  spi_dma_tx_cmp_flag=false;

  HAL_GPIO_WritePin(LCD_DC_GPIO_GROUP, LCD_DC_GPIO_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_SPI_CS_GPIO_GROUP, LCD_SPI_CS_GPIO_PIN, GPIO_PIN_SET);

  //HAL_Delay(1);
  

  return 0;
}

/**
 * @brief 设置LCD渲染的起始地址
 *        设置完成后，直接往LCD发送数据即可渲染屏幕
 * 
 * @TODO  错误处理没做
 */ 
int lcd_set_address(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2){
  uint8_t CASET=0x2A;
  uint8_t CASET_data1 = 0;
  uint8_t CASET_data2 = x1+1;
  uint8_t CASET_data3 = 0;
  uint8_t CASET_data4 = x2-1+1;
  uint8_t RASET=0X2B;
  uint8_t RASET_data1= 0;
  uint8_t RASET_data2= y1+2;
  uint8_t RASET_data3= 0;
  uint8_t RASET_data4= y2-1+2;
  uint8_t RAMWR=0X2C;
  lcd_write(&CASET, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&CASET_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&CASET_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&CASET_data3, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&CASET_data4, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&RASET, 1, LCD_WRITE_TYPE_COMMAND);
  lcd_write(&RASET_data1, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&RASET_data2, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&RASET_data3, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&RASET_data4, 1, LCD_WRITE_TYPE_DATA);
  lcd_write(&RAMWR, 1, LCD_WRITE_TYPE_COMMAND);
  return 0;
}

/**
 *  @brief 渲染一个大数组
 *         多次调用lcd_write
 *  @note  由于dma限制一次只能发送1024Byte的数据，所以大数据要分多次发送
 *  
 *  @return 0发送成功，其他发送失败
 */ 
int utils_lcd_write_multiple_times(uint8_t* data, uint16_t size){
  int send_len=0;
  int dma_len = sizeof(spi_dma_buffer);

  for(;send_len<size;){
    int ready_to_send_len=size-send_len > dma_len ? dma_len : size-send_len;
    if(lcd_write((uint8_t*)data+send_len, ready_to_send_len, LCD_WRITE_TYPE_DATA)!=0){
      return -1;
    }
    send_len+=ready_to_send_len;
  }

  return 0;
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
