# 하드웨어 연결

핀 정보는 현재 소스와 원본 적용 안내를 기준으로 정리하였다.

| 대상 | 신호 | STM32 핀 / 기능 |
| --- | --- | --- |
| 정면 초음파 | Trig / Echo | PA7 / PA6 |
| 왼쪽 초음파 | Trig / Echo | PA0 / PA1 |
| 오른쪽 초음파 | Trig / Echo | PA4 / PB0 |
| L298N | IN1 / IN2 / IN3 / IN4 | PB12 / PB13 / PB14 / PB15 |
| L298N | ENA / ENB | PB8 TIM4_CH3 / PB9 TIM4_CH4 |
| HC-06 | RX ← MCU TX / TX → MCU RX | PB6 USART1_TX / PB7 USART1_RX |
| 디버그 UART | TX / RX | PA2 / PA3, USART2 |
| 버튼 / LED | B1 / LD2 | PC13 / PA5 |

## 기존 프로젝트에서 필요한 설정

- MX_TIM4_Init(), MX_USART1_UART_Init(), MX_USART2_UART_Init()와 관련 핸들이 필요하다.
- TIM4 CH3/CH4를 PWM 출력으로 초기화한다. 실행 시 PSC=0, ARR=4999로 설정한다. 현재 클럭에서 TIM4 입력이 100 MHz이면 PWM은 20 kHz이다.
- USART1은 기존 생성 코드의 통신 속도를 유지한다. HC-06 모듈 설정과 일치해야 한다.
- USART2는 코드에서 115200 bps, 8N1, 흐름 제어 없음으로 설정한다.
- 수신은 BT_Poll()과 DebugUart_Poll()이 레지스터를 폴링한다. 별도의 HAL_UART_Receive_IT() 경로를 중복으로 시작하지 않는다.
- 초음파 시간 측정에는 DWT 사이클 카운터를 사용한다.

## 모터 방향

L298N ENA·ENB 점퍼를 제거하고 PWM 핀에 연결한다. 기본 논리 왼쪽은 ENA, 오른쪽은 ENB이다. 양수 출력은 각각 PB13/PB15를 HIGH로 설정하며 실제 전진 방향은 모터 배선에 따라 달라진다.

rc_config.h의 MOTOR_SWAP_SIDES, ENA_FORWARD_SIGN, ENB_FORWARD_SIGN으로 좌우·극성을 조정한다. LEFT_TRIM, RIGHT_TRIM은 출력 보정값이다.

## 전원 구성 기록

제작 시 외부 배터리를 L298N에 연결하고 L298N 측에서 보드로 전원을 공급하였다. 보드·드라이버·센서의 신호 기준에는 공통 GND가 필요하다.

현재 자료는 보드 전원 입력 단자와 레귤레이터 배선까지 확정한 회로도가 아니다. 배터리 전압을 보드의 5V/3.3V 핀에 직접 연결하는 안내로 사용하지 않는다. 초음파 Echo 신호 레벨과 사용 핀의 허용 전압도 실제 모듈·회로에서 확인해야 한다.
