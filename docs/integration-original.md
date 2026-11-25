# RC_Project_ULTRASONIC 파일 분리 버전

첨부한 **main(3).c / REBUILD_16_BT**를 기준으로 파일만 분리했습니다.
주행 임계값, PWM, 핀, 측정 간격, 블루투스 명령, 파란 버튼 기능은 원본과 같습니다.
원본 파일은 보관해 두고, 아래 파일을 기존 프로젝트에 적용하세요.

## 1. 적용 순서

1. 기존 프로젝트를 백업합니다.
2. 압축 안의 `Core/Src`에 있는 **7개 .c 파일**을 프로젝트의 `Core/Src`에 복사합니다. 이 중 `main.c`는 기존 내용을 교체합니다.
3. 압축 안의 `Core/Inc`에 있는 **9개 .h 파일**을 프로젝트의 `Core/Inc`에 복사합니다.
4. 기존 `main.h`, `tim.c/.h`, `usart.c/.h`, `gpio.c/.h`, HAL 드라이버, 인터럽트 파일과 `.ioc`는 그대로 유지합니다. 압축 파일은 전체 CubeIDE 프로젝트가 아니라 교체·추가할 소스 묶음입니다.
5. CubeIDE에서 프로젝트를 선택하고 **F5(Refresh)**를 누릅니다.
6. **Project → Clean** 후 **Build Project**를 실행합니다. 새 `.c` 파일이 빌드 대상에서 제외되지 않았는지 확인합니다.
7. 보드에 다운로드합니다. 기존처럼 수동 모드로 시작하며, 파란 버튼으로 자율주행 모드와 전환합니다.

**이번에는 main.c만 바꾸면 안 됩니다. .c 7개와 .h 9개를 모두 넣어야 합니다.**
파일 분리 때문에 MX 설정이나 배선을 바꿀 필요는 없으며, Generate Code도 누를 필요가 없습니다.
기존의 큰 코드를 `main_old.c` 또는 `main(3).c`라는 이름으로 Core/Src에 함께 두지 마세요.
같은 main/SystemClock_Config/Error_Handler가 중복 정의될 수 있으므로 백업은 프로젝트 밖에 보관하세요.

## 2. 파일별 역할

| 위치/파일 | 역할 |
| --- | --- |
| Core/Src/main.c | App_Init() 한 번 호출, App_Task() 반복 호출 |
| Core/Src/app.c | 주행 상태 머신, 중앙 보정, 코너·후진·복구, 파란 버튼, 로그 내용 |
| Core/Src/bt.c | USART1 수신, F/B/L/R/S 해석, 수신 오류 처리 |
| Core/Src/ultrasonic.c | 3개 센서 순차 측정, Trig/Echo 처리, 타임아웃, 필터·측정 상태 |
| Core/Src/motor.c | L298N 방향, TIM4 PWM, 가감속, 정·역전 전환 대기시간 |
| Core/Src/debug_uart.c | USART2 명령 수신, MoSerial 로그의 비차단 송신 |
| Core/Src/board.c | 기존 클럭·GPIO 초기화와 Error_Handler |
| Core/Inc/app.h | main.c에서 사용하는 App_Init/App_Task 선언 |
| Core/Inc/bt.h | 블루투스 함수 선언 |
| Core/Inc/ultrasonic.h | 센서 ID·측정값 구조체·센서 함수 선언 |
| Core/Inc/motor.h | 모터 제어 함수 선언 |
| Core/Inc/debug_uart.h | 로그 송신 버퍼와 USART2 함수 선언 |
| Core/Inc/board.h | 보드 초기화 함수 선언 |
| Core/Inc/rc_config.h | 속도·거리·시간·센서 보정 등 모든 튜닝 상수 |
| Core/Inc/rc_math.h | 범위 제한·부호·시간 비교 공통 함수 |
| Core/Inc/app_hooks.h | 센서/통신 모듈이 주행 프로그램과 연결되는 내부 함수 선언 |

`.h`는 다른 파일에서 사용할 함수와 자료형을 알리는 인터페이스이고, `.c`에는 실제 실행 코드가 있습니다.
**다른 .c 파일을 #include하지 않습니다.** 각 .c는 따로 컴파일되고, 링커가 함수들을 연결합니다.
모터의 실제 상태, 초음파 측정 순서, 통신 송신 버퍼 등은 각 모듈의 `static` 변수로 숨겼습니다.

app.c는 주행 판단을 담당합니다. 그래서 bt.c는 명령을 해석한 뒤 app.c에 전달하며,
모터를 별도 경로로 직접 제어하지 않습니다. 기존 수동/자율 전환 순서를 유지하기 위한 구조입니다.
센서의 코너 접근 변화 판단과 회전 후 측면값 갱신 조건은 주행 상태와 관련되어 있으므로
app_hooks.h의 함수를 통해 연결했습니다. 단순히 파일마다 전역변수를 중복 선언하지 않습니다.
Ultrasonic_GetRanges()는 원본에서 주행부도 조정하던 측정 카운터를 보존하기 위해
현재 측정 배열에 접근하도록 합니다. 임의의 다른 모듈에서 측정값을 변경하지 마세요.

## 3. 이제 main.c는 이 정도입니다

```c
#include "main.h"
#include "app.h"

int main(void)
{
    App_Init();

    while (1)
    {
        App_Task();
    }
}
```

App_Init은 보드 → 디버그 UART → 모터 → 초음파를 초기화하고 모터를 정지시킵니다.
App_Task는 기존과 같은 순서로 센서 → 버튼 → 블루투스/디버그 UART → 주행 판단 → 모터 → 로그/LED를 처리합니다.
Trig 펄스 생성 중에는 센서 처리를 우선하는 기존 동작도 유지했습니다.
RTOS나 새 인터럽트, DMA를 추가하지 않았습니다.

## 4. 어디를 수정하면 되나요?

- 속도·거리·게인·센서 간격 조정: **rc_config.h**
- 회전·후진·코너 판단 변경: **app.c**
- 블루투스 명령 문자 변경: **bt.c**
- 모터 방향/PWM 구현 변경: **motor.c**
- 초음파 측정·필터 구현 변경: **ultrasonic.c**
- 로그에 출력할 항목 변경: **app.c의 LogTask**
- 클럭·GPIO 초기화 변경: **board.c**

## 5. 그대로 유지한 주요 값과 핀

- 최고속도 허용: 정면 65cm, 좌우 각각 12cm, 좌우 불균형 35%, 새 정면 측정 2회.
- 수동 전후진 PWM 100, 수동 회전 PWM 85. 기존 벽 근접 감속과 자동 회전 출력 유지.
- 정면 초음파: Trig PA7 / Echo PA6.
- 왼쪽 초음파: Trig PA0 / Echo PA1.
- 오른쪽 초음파: Trig PA4 / Echo PB0.
- L298N IN1/IN2/IN3/IN4: PB12/PB13/PB14/PB15.
- ENA/ENB: PB8 TIM4_CH3 / PB9 TIM4_CH4.
- 블루투스 USART1: PB6 TX / PB7 RX, 기존 generated baud rate 유지.
- MoSerial USART2: PA2 TX / PA3 RX, 115200 8N1.
- 버튼 PC13 / LED PA5.

## 6. 검증 범위

- PC의 HAL 대체 환경에서 7개 .c 파일을 각각 컴파일한 뒤 하나의 실행 파일로 링크했습니다.
  실제 UART 레지스터 폴링 부분도 컴파일 대상으로 확인했습니다.
- 기존 센서·직진·회전·후진 복구·가속·블루투스 모의 검사 91개 항목을 통과했습니다.
- 원본 main(3).c와 분리본에 같은 가상 거리·실패·명령 입력을 넣고,
  3개 시드 × 20,000단계 = 60,000단계의 상태·목표 PWM·출력 PWM·코너 방향 기억·필터값·판단 이유를 비교했으며 일치했습니다.
- 이것은 실제 트랙 물리 시뮬레이션이 아닙니다. STM32용 실제 CubeIDE 빌드와 보드 주행은 여기서 실행하지 못했습니다.
