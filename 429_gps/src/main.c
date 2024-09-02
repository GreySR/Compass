#include "stm32f429xx.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_i2c_ex.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_flash_ex.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "arm_math.h"

#define SIZE 170
#define RAD_TO_DEG (180.0 / PI)
#define DEG_TO_RAD (PI / 180.0)

float32_t angle = 0;
uint8_t pData[SIZE];
uint8_t heading[6];

char str[20];
uint8_t R = 81, Ox = 117, Oy = 108, l90 = 15, l30 = 10, l_arrow = 75, l_tail = 55;
uint8_t Time[]    = "Time:05:05:05";
uint8_t Lat[]     = "La:0000.00000N";
uint8_t Lon[]     = "Lo:0000.00000E";


I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
GPIO_InitTypeDef GPIO_InitStruct;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart1_rx;

static void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    ///* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* The voltage scaling allows optimizing the power consumption when the device is
    clocked below the maximum system frequency, to update the voltage scaling value
    regarding system frequency refer to product datasheet. */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    /* Activate the Over-Drive mode */
    HAL_PWREx_EnableOverDrive();
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
    clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;//RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;//RCC_CFGR_PPRE1_DIV8;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void MX_GPIO_Init(void) {  
  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();  
  __GPIOF_CLK_ENABLE();
}

void MX_USART_Init() { 
    __USART1_CLK_ENABLE();
    __USART5_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = (uint8_t)0x07;  
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = (uint8_t)0x07; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
    
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = (uint8_t)0x08; //GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = (uint8_t)0x08; //GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 9600;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart5);       
    // void LL_USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t OverSampling,
    //                                      uint32_t BaudRate)
    // внутри ф-ии USARTx->BRR = (uint16_t)(__LL_USART_DIV_SAMPLING16(PeriphClk, BaudRate));
    // ((((((uint32_t)((((uint64_t)(((PeriphClk))))*25)/(4*((uint64_t)(((BaudRate)))))))/100) << 4) +
    // (((((((uint32_t)((((uint64_t)(((PeriphClk))))*25)/(4*((uint64_t)(((BaudRate))))))) - 
    // ((((uint32_t)((((uint64_t)((((PeriphClk)))))*25)/
    // (4*((uint64_t)((((BaudRate))))))))/100) * 100)) * 16) + 50) / 100) & 0xF0)) +
    // (((((((uint32_t)((((uint64_t)(((PeriphClk))))*25)/(4*((uint64_t)(((BaudRate))))))) -
    // ((((uint32_t)((((uint64_t)((((PeriphClk)))))*25)/
    // (4*((uint64_t)((((BaudRate))))))))/100) * 100)) * 16) + 50) / 100) & 0x0F))
    
    //USART1->BRR = 9375; //9600 45.0 
    USART1->BRR = 781; // 1152000 
    UART5->BRR  = 4687; //9600 22.5

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart5_rx);

    __HAL_LINKDMA(&huart5, hdmarx, hdma_uart5_rx);    


    /* UART1 DMA Init */
    /* UART1_RX Init */
    hdma_uart1_rx.Instance = DMA2_Stream2;
    hdma_uart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart1_rx);

    __HAL_LINKDMA(&huart1, hdmarx, hdma_uart1_rx); 
}

void MX_TIM2_Init() {
    __TIM2_CLK_ENABLE();    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = (uint8_t)0x01; 
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 199;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999;//4599;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; //TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void MX_DMA_Init() {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}


void drawCompas() {
    BSP_LCD_DrawCircle(Ox, Oy, R); // 2 окружности, для толщины линии
    BSP_LCD_DrawCircle(Ox, Oy, R - 1);
    BSP_LCD_FillCircle(Ox, Oy, 3); // точка-центр

    BSP_LCD_DrawLine(Ox, Oy + R, Ox, Oy + R + l90);
    BSP_LCD_DrawLine(Ox, Oy - R, Ox, Oy - R - l90);
    BSP_LCD_DrawLine(Ox + R, Oy, Ox + R + l90, Oy);
    BSP_LCD_DrawLine(Ox - R, Oy, Ox - R - l90, Oy);

    BSP_LCD_DisplayChar(Ox + 3,  Oy - R - l90 - 8, 'N');
    BSP_LCD_DisplayChar(Ox + R + 3,  Oy + 3, 'E');
    BSP_LCD_DisplayChar(Ox - 18,  Oy + R + l90 - 10, 'S');
    BSP_LCD_DisplayChar(Ox - R - 19,  Oy - 24, 'W');

    // 30 60
    BSP_LCD_FillCircle(Ox + 40.5, Oy + 70.1, 3);
    BSP_LCD_FillCircle(Ox - 40.5, Oy - 70.1, 3);
    BSP_LCD_FillCircle(Ox + 40.5, Oy - 70.1, 3);
    BSP_LCD_FillCircle(Ox - 40.5, Oy + 70.1, 3);   
    BSP_LCD_FillCircle(Ox + 70.1, Oy + 40.5, 3);
    BSP_LCD_FillCircle(Ox - 70.1, Oy - 40.5, 3);
    BSP_LCD_FillCircle(Ox + 70.1, Oy - 40.5, 3);
    BSP_LCD_FillCircle(Ox - 70.1, Oy + 40.5, 3);

    // 45
    BSP_LCD_FillCircle(Ox + 57.27, Oy + 57.27, 2);
    BSP_LCD_FillCircle(Ox - 57.27, Oy - 57.27, 2);
    BSP_LCD_FillCircle(Ox + 57.27, Oy - 57.27, 2);
    BSP_LCD_FillCircle(Ox - 57.27, Oy + 57.27, 2);

    // 15 75
    BSP_LCD_FillCircle(Ox + 20.95, Oy + 78.24, 2);
    BSP_LCD_FillCircle(Ox - 20.95, Oy - 78.24, 2);
    BSP_LCD_FillCircle(Ox + 20.95, Oy - 78.24, 2);
    BSP_LCD_FillCircle(Ox - 20.95, Oy + 78.24, 2);
    BSP_LCD_FillCircle(Ox + 78.24, Oy + 20.95, 2);
    BSP_LCD_FillCircle(Ox - 78.24, Oy - 20.95, 2);
    BSP_LCD_FillCircle(Ox + 78.24, Oy - 20.95, 2);
    BSP_LCD_FillCircle(Ox - 78.24, Oy + 20.95, 2);
}

void drawNarrow() {
    float32_t x = arm_sin_f32(-angle);
    float32_t y = arm_cos_f32(-angle); 
    BSP_LCD_DrawLine(Ox, Oy, Ox - l_arrow * x, Oy - l_arrow * y);
    BSP_LCD_DrawLine(Ox, Oy, Ox + l_tail  * x, Oy + l_tail  * y);
    angle = atof((unsigned char *)&heading) * DEG_TO_RAD;
}

void setTime() {
    BSP_LCD_DisplayStringAtLine(10, Time);
}

void setGPS() {    
    BSP_LCD_DisplayStringAtLine(11, Lat);
    BSP_LCD_DisplayStringAtLine(12, Lon);
}

void setHeading() {
    BSP_LCD_DisplayStringAtLine(9, heading);
}

char* string_relace(char* source, char* substring, char* with) {
    char* substring_source = strstr(source, substring);
    if(substring_source == NULL) {
        return NULL;
    }
    memmove(
        substring_source + strlen(with),
        substring_source + strlen(substring),
        strlen(substring_source) - strlen(substring) + 1
        );
        
    memcpy(substring_source, with, strlen(with));
    return substring_source + strlen(with);
}

// Example of string
// "...$GPVTG,,T,,M,0.115,N,0.214,K,A*21$GPGGA,184641.00,N,07809.54978,E,1,06,3.35,257.2,M,15.9,M,,*55$GPGSA,A,3,20,22,15,13,05,14,,,,,,,4.90,3.35,3.57*0F..."
void gps_parsing() {
    uint8_t len_substr = 50;

    uint8_t* ptr;
    ptr = strstr(pData, "$GPGGA");    
    if(ptr == NULL)
        return;
    
    uint8_t pos = ptr - pData;
    if(pos + len_substr >= sizeof(pData))
        return;   
    
    uint8_t s[len_substr];
    strncpy(s, pData + pos, len_substr);    
    
    while(string_relace(s, ",,", ",x,"));
    
    uint8_t i = 0;
    char *ch;
    ch = strtok(s, ",");
    while (ch != NULL) {
        ++i; 
        ch = strtok(NULL, ","); 
               
        if(i == 1) {
            if(*ch == 'x')
                continue; 
            else {
                Time[5]  = ch[0];
                Time[6]  = ch[1];
                Time[8]  = ch[2];
                Time[9]  = ch[3];
                Time[11] = ch[4];
                Time[12] = ch[5];
            }                
        } 
        else if(i == 2) {
            if(*ch == 'x')
                continue; 
            else {
                char s_tmp[15] = "La:";
                strcat(s_tmp, ch);
                s_tmp[13] = 'N';
                s_tmp[14] = '\0';
                if(strlen(s_tmp) < 10)
                    continue;
                strcpy(Lat, s_tmp);
            }                           
        }       
        else if(i == 4) {
            if(*ch == 'x')
                continue; 
            else {
                char s_tmp[15] = "Lo:";
                strcat(s_tmp, ch);
                s_tmp[13] = 'E';
                s_tmp[14] = '\0';
                if(strlen(s_tmp) < 10)
                    continue;
                strcpy(Lon, s_tmp);
            }                
        }
    }    
}

void TIM2_IRQHandler() {    
    TIM2->SR &= ~TIM_SR_UIF;       
    BSP_LCD_Clear(LCD_COLOR_GREEN); // Clear display
    drawCompas();
    drawNarrow();
    setHeading();
    gps_parsing();
    setTime();
    setGPS();
}

void DMA1_Stream0_IRQHandler() {
    
}

void DMA2_Stream2_IRQHandler() {    
   
}

int main() {    
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();    
    MX_DMA_Init();
    MX_USART_Init();
    MX_TIM2_Init();  

    BSP_LCD_Init(); 
    BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
    BSP_LCD_DisplayOn();
    BSP_LCD_Clear(LCD_COLOR_GREEN); // Clear display
    BSP_LCD_SetBackColor(LCD_COLOR_GREEN); // Choose background color
    BSP_LCD_SetTextColor(LCD_COLOR_RED); // Set work color, not only for text
    drawCompas();
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_UART_Receive_DMA(&huart5, pData, SIZE); 
    HAL_UART_Receive_DMA(&huart1, heading, 6);
    while(1) {
        
    }
}