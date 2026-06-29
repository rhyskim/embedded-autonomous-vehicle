# Embedded Autonomous Vehicle

> **STM32F429 + FreeRTOS** 플랫폼 위에서 동작하는 미로 탈출 자율 주행 로봇 프로젝트입니다.  
> 초음파·적외선 센서 융합과 PD 제어 알고리즘을 이용해 벽면을 탐색하고, 동적 장애물(로봇청소기)을 실시간으로 회피합니다.

---

## 프로젝트 개요

| 항목 | 내용 |
|---|---|
| **MCU** | STM32F429ZI (ARM Cortex-M4, 180 MHz) |
| **RTOS** | FreeRTOS v9 (CMSIS-RTOS API) |
| **개발 환경** | Keil MDK-ARM (µVision 5) |
| **드라이버** | STM32 HAL (STM32F4xx_HAL_Driver) |
| **보드** | STM32F429I-EVAL |

---

## 주요 알고리즘

### 1. 방위각(Azimuth) 기반 경로 탐색

로봇은 절대 방위각(북/동/남/서)을 추적하며 `get_direction_score()` 함수를 통해 **북쪽 우선 점수 기반** 최적 경로를 선택합니다.

```
북(3점) > 동·서(2점) > 남(1점)
```

장애물이 탐지되면 다음 순서로 회전 방향을 결정합니다:

- **오른쪽만 열림** → 우회전
- **왼쪽만 열림** → 좌회전
- **양쪽 열림** → 방위각 점수가 높은 방향으로 회전
- **막힌 골목** → 180° U턴

### 2. PD 기반 조향 보정 (IR 45° 센서 융합)

좌·우 45도 방향의 적외선 센서 값 차이를 PD 제어기에 입력하여 **직진 경로를 미세 보정**합니다.

```
PD_output = Kp * error + Kd * (error - prev_error)
Left_PWM  = BASE_PULSE + PD_output
Right_PWM = BASE_PULSE - PD_output
```

| 파라미터 | 값 | 설명 |
|---|---|---|
| `KP` | 4 | 비례 게인 |
| `KD` | 8 | 미분 게인 |
| `BASE_PULSE` | 19000 | 기본 PWM 듀티 |
| `IR_DEADZONE` | 100 | 오차 데드존 (진동 방지) |

### 3. 동적 장애물 실시간 감지

정면 초음파 센서가 반응했을 때, IR 센서의 **변화율(미분)**을 50회(약 500 ms) 동안 추적합니다.  
변화량이 임계값(`IR_DELTA_TH = 200`)을 초과하면 동적 장애물로 판정하여 **급발진 회피**를 실행합니다.

```c
if (delta_left > IR_DELTA_TH || delta_right > IR_DELTA_TH)
    is_dynamic_obstacle = 1;  // 로봇청소기 확정!
```

### 4. 틱 카운터 기반 정밀 회전

엔코더 인터럽트로 누적된 모터 틱을 기반으로 **90° / 180° 회전**을 정밀하게 제어합니다.

| 상수 | 값 | 의미 |
|---|---|---|
| `TURN_90_TICKS` | 900 | 90도 회전 목표 틱 |
| `TURN_180_TICKS` | 1800 | 180도 회전 목표 틱 |
| `TURN_BASE_PULSE` | 15000 | 회전 시 PWM |

---

## 프로젝트 구조

```
Lab8_RTOS/
├── project/
│   └── RTOS/
│       ├── Src/
│       │   ├── main.c                  ← 핵심 알고리즘 (미로 탐색 + 동적 장애물 감지)
│       │   ├── stm32f4xx_hal_msp.c     ← HAL MSP 초기화 (GPIO, UART, TIM 핀 설정)
│       │   ├── stm32f4xx_it.c          ← 인터럽트 핸들러
│       │   └── system_stm32f4xx.c      ← 시스템 클럭 설정
│       ├── Inc/
│       │   ├── main.h                  ← 핀 정의, 매크로
│       │   ├── stm32f4xx_hal_conf.h    ← HAL 모듈 활성화 설정
│       │   └── stm32f4xx_it.h          ← 인터럽트 핸들러 선언
│       └── MDK-ARM/
│           ├── Project.uvprojx         ← Keil 프로젝트 파일
│           └── startup_stm32f429xx.s   ← 스타트업 어셈블리
├── Drivers/
│   ├── BSP/                            ← 보드 지원 패키지 (stm324x9i_eval)
│   ├── CMSIS/                          ← ARM CMSIS 코어 헤더
│   └── STM32F4xx_HAL_Driver/           ← STM32 HAL 드라이버
└── Middlewares/
    └── Third_Party/
        └── FreeRTOS/
            └── Source/                 ← FreeRTOS 커널 소스
```

---

## 하드웨어 구성

| 센서 / 액추에이터 | 핀 / 인터페이스 | 용도 |
|---|---|---|
| 초음파 (우) | TIM3_CH2 (Input Capture) | 우측 거리 측정 |
| 초음파 (전) | TIM3_CH3 (Input Capture) | 정면 거리 측정 |
| 초음파 (좌) | TIM3_CH4 (Input Capture) | 좌측 거리 측정 |
| IR 좌 45° | ADC3 CH11 | 좌측 벽 PD 보정 |
| IR 우 45° | ADC2 CH14 | 우측 벽 PD 보정 |
| 모터 (우) | TIM8_CH1/CH2 (PWM) | 우측 구동 모터 |
| 모터 (좌) | TIM4_CH1/CH2 (PWM) | 좌측 구동 모터 |
| 엔코더 (우) | PA15 (EXTI) | 우측 회전 틱 계수 |
| 엔코더 (좌) | PB4 (EXTI) | 좌측 회전 틱 계수 |
| UART1 | UART (115200 bps, 8N1) | 디버그 콘솔 출력 |
| 초음파 트리거 | TIM10_CH1 (PWM) | 초음파 트리거 펄스 생성 |

---

## FreeRTOS 태스크 구조

```
vTaskStartScheduler()
│
├── Task: Detect_obstacle  (Priority: 2, Stack: 128 words)
│     └── 60 ms 주기로 초음파 거리 계산
│         → obs_forward / obs_left / obs_right 전역 플래그 갱신
│
└── Task: Motor_control    (Priority: 1, Stack: 128 words)
      ├── [직진] IR 45° PD 조향 → Motor_Forward()
      ├── [T자 교차로] 북쪽 샛길 감지 → 전진 후 회전
      ├── [정면 장애물] 동적 장애물 판별 스캔 (500 ms)
      │     ├── 동적(로봇청소기) → 통과 대기 → 급발진 탈출
      │     └── 정적(벽) → 방위각 점수 기반 회전 결정
      └── Execute_Tick_Turn() → 엔코더 틱 기반 정밀 회전
```

---

## 빌드 및 플래싱 방법

### 요구사항

- Keil MDK-ARM 5.x 이상
- STM32F4 Device Family Pack 설치
- ST-Link V2 디버거

### 빌드 절차

1. `Lab8_RTOS/project/RTOS/MDK-ARM/Project.uvprojx` 파일을 Keil에서 엽니다.
2. **Build** (`F7`)을 눌러 빌드합니다.
3. 빌드 성공 후 **Download** (`F8`)으로 보드에 플래싱합니다.

### 디버그 콘솔 출력 예시

UART1 (115200 bps, 8N1)에 시리얼 터미널을 연결하면 실시간 상태 로그를 확인할 수 있습니다.

```
[SYS] Ultrasonic Scanner Initialized
[SYS] Forward-First Azimuth Navigation Initialized
[ROUTE] Snapshot F:1 L:0 R:1 / LS:3 RS:-1
[ROUTE] Open Left -> Turn Left
[ALIGN] Secondary Fine Tuning...
[SYS] Turn Completed. Direction: 3
[ROBOT CLEANER] Moving Obstacle Confirmed!
[DODGE] Waiting for Cleaner to Pass Away...
[DODGE] DASH!!!
```

---

## 주요 튜닝 파라미터

`Src/main.c` 상단의 `#define` 값을 조정하여 주행 특성을 튜닝할 수 있습니다.

```c
#define KP                    4       // IR PD 비례 게인
#define KD                    8       // IR PD 미분 게인
#define BASE_PULSE            19000   // 기본 전진 PWM 듀티
#define ULTRASONIC_FRONT_TH   15      // 정면 장애물 임계 거리 (cm)
#define ULTRASONIC_SIDE_TH    25      // 측면 장애물 임계 거리 (cm)
#define IR_VALID_MIN          220     // IR 센서 유효 최솟값 (ADC raw)
#define TURN_90_TICKS         900     // 90° 회전 목표 엔코더 틱
#define TURN_180_TICKS        1800    // 180° 회전 목표 엔코더 틱
#define IR_DELTA_TH           200     // 동적 장애물 IR 변화량 임계값
#define DASH_TIME_MS          5000    // 동적 장애물 회피 후 급발진 시간 (ms)
```

---

## 라이선스

- **FreeRTOS** — MIT License (Copyright Amazon.com, Inc. or its affiliates)
- **STM32 HAL / BSP** — BSD 3-Clause License (Copyright STMicroelectronics)
