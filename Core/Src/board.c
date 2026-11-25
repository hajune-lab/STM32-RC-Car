/* Split from the working REBUILD_16_BT main(3).c; driving settings unchanged. */
#include "board.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#define DIRECTION_PINS (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

static void ApplicationGPIOInit(void){
    GPIO_InitTypeDef g={0};
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,DIRECTION_PINS,GPIO_PIN_RESET);
    g.Mode=GPIO_MODE_OUTPUT_PP;g.Pull=GPIO_NOPULL;g.Speed=GPIO_SPEED_FREQ_LOW;
    g.Pin=GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=DIRECTION_PINS;HAL_GPIO_Init(GPIOB,&g);
    g.Mode=GPIO_MODE_INPUT;g.Pin=GPIO_PIN_1|GPIO_PIN_6;HAL_GPIO_Init(GPIOA,&g);
    g.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOB,&g);g.Pin=GPIO_PIN_13;HAL_GPIO_Init(GPIOC,&g);
    g.Mode=GPIO_MODE_AF_PP;g.Alternate=GPIO_AF2_TIM4;g.Pin=GPIO_PIN_8|GPIO_PIN_9;HAL_GPIO_Init(GPIOB,&g);
}

void Board_Init(void){
    HAL_Init();SystemClock_Config();MX_TIM4_Init();
    MX_USART1_UART_Init();MX_USART2_UART_Init();ApplicationGPIOInit();
}

void SystemClock_Config(void){
    RCC_OscInitTypeDef osc={0};RCC_ClkInitTypeDef clk={0};
    __HAL_RCC_PWR_CLK_ENABLE();__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSI;osc.HSIState=RCC_HSI_ON;
    osc.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;osc.PLL.PLLState=RCC_PLL_ON;
    osc.PLL.PLLSource=RCC_PLLSOURCE_HSI;osc.PLL.PLLM=16;osc.PLL.PLLN=200;
    osc.PLL.PLLP=RCC_PLLP_DIV2;osc.PLL.PLLQ=4;
    if(HAL_RCC_OscConfig(&osc)!=HAL_OK)Error_Handler();
    clk.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV2;clk.APB2CLKDivider=RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_3)!=HAL_OK)Error_Handler();
}

void Error_Handler(void){
    if(htim4.Instance==TIM4){__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0U);__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0U);}
    __disable_irq();while(1){}
}

