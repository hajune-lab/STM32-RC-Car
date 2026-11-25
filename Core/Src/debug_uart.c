/* Split from the working REBUILD_16_BT main(3).c; driving settings unchanged. */
#include "debug_uart.h"
#include "usart.h"
#include "app_hooks.h"

static char tx[DEBUG_UART_BUFFER_SIZE];
static unsigned tx_len, tx_pos;
void DebugUart_Init(void){
    huart2.Init.BaudRate=115200;huart2.Init.WordLength=UART_WORDLENGTH_8B;
    huart2.Init.StopBits=UART_STOPBITS_1;huart2.Init.Parity=UART_PARITY_NONE;
    huart2.Init.Mode=UART_MODE_TX_RX;huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    if(HAL_UART_Init(&huart2)!=HAL_OK)Error_Handler();
}
bool DebugUart_IsBusy(void){return tx_pos<tx_len;}
char *DebugUart_Buffer(void){return tx;}
void DebugUart_Commit(unsigned length){
    if(length<sizeof(tx)&&!DebugUart_IsBusy()){tx_len=length;tx_pos=0;}
}
void DebugUart_Poll(uint32_t now){
#ifndef RC_HOST_TEST
    /* No blocking transmit, interrupts, DMA or CubeMX changes required. */
    uint32_t sr=huart2.Instance->SR;
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE)){
        uint32_t data=huart2.Instance->DR;
        if(sr&USART_SR_RXNE)App_DebugCommand((char)data,now);
    }
    if(tx_pos<tx_len&&(huart2.Instance->SR&USART_SR_TXE))huart2.Instance->DR=(uint8_t)tx[tx_pos++];
#else
    (void)now;tx_pos=tx_len;
#endif
}
