/* Split from the working REBUILD_16_BT main(3).c; driving settings unchanged. */
#include "bt.h"
#include "usart.h"
#include "app_hooks.h"
#include "rc_config.h"

static uint32_t bluetooth_errors;
void BT_HandleCommand(char c,uint32_t now){
    if(c>='a'&&c<='z')c=(char)(c-'a'+'A');
    if(c=='S'){App_StopWithReason(now,"BT_STOP");return;}
    if(!App_IsManual())return;
    int l=0,r=0;
    switch(c){
        case 'F':l=MANUAL_DRIVE_PWM;r=MANUAL_DRIVE_PWM;break;
        case 'B':l=-MANUAL_DRIVE_PWM;r=-MANUAL_DRIVE_PWM;break;
        case 'L':l=-MANUAL_TURN_PWM;r=MANUAL_TURN_PWM;break;
        case 'R':l=MANUAL_TURN_PWM;r=-MANUAL_TURN_PWM;break;
        default:return;
    }
    App_ManualDrive(l,r,now);
}
void BT_HandleFault(uint32_t now){
    ++bluetooth_errors;
    if(App_IsManual())App_StopWithReason(now,"BT_RX_ERROR");
}
uint32_t BT_ErrorCount(void){return bluetooth_errors;}
void BT_Poll(uint32_t now){
#ifndef RC_HOST_TEST
    /* Poll USART1 without arming Receive_IT: existing USART1 IRQ may remain
       enabled in NVIC, but RXNEIE stays disabled. SR then DR clears RX errors.
       Preserve MX_USART1_UART_Init() baud rate from the working BT project. */
    uint32_t bt_sr=huart1.Instance->SR;
    if(bt_sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){
        uint32_t data=huart1.Instance->DR;
        if(bt_sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))BT_HandleFault(now);
        else if(bt_sr&USART_SR_RXNE)BT_HandleCommand((char)data,now);
    }
#else
    (void)now;
#endif
}
