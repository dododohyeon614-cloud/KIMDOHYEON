// Arduino motor/steering controller [PS2 우선권 보장판]
//
// 변경 요약 (이전 delay 제거판 대비)
// 1. PS2 read 동안 Serial RX 인터럽트만 임시 차단 (UCSR0B의 RXCIE0 비트 토글)
//    → ROS 통신 트래픽이 PS2X bit-bang 타이밍을 깨지 못하도록 보호
//    → MsTimer2 조향 PID와 Serial TX는 그대로 동작 (안전)
// 2. PS2 폴링 주기 20ms → 30ms (다른 작업과 시간 다툼 완화)
// 3. PS2 통신 실패 자동 감지 + 재설정 시도 (con_error 0이어도 stale 상태 회복)
// 4. [핵심] 리모컨 Override 윈도우: L1/R1 누른 순간부터 300ms간
//    ROS 명령을 완전 무시하고 리모컨 입력 우선 유지
//    → 한 번의 PS2 read 실패가 ROS 끼어들기로 이어지지 않음
//    → 사람이 위급 상황에 리모컨을 잡으면 즉시 시스템 장악
//
// 단위/통신 체계 (이전과 동일)
// - Serial 수신: C,velocity,steer  /  S,0|1
// - velocity 단위: tick/10ms, 부호 포함
// - Serial 송신: E,d_val  (DIAG_DT=1일 때 E,d_val,dt_us)

#include <MsTimer2.h>
#include <PS2X_lib.h>
#include <SPI.h>
#include <math.h>

#define DIAG_DT 0

// ── 전역 명령 변수 ─────────────────────────────────────────────────────────
int  velocity    = 0;
int  steer_angle = 0;
bool state       = false;

// ── Serial 수신 버퍼 ────────────────────────────────────────────────────────
static char    rx_line[64];
static uint8_t rx_idx = 0;

#define MAX_VEL 255

// ── 구동 모터 ───────────────────────────────────────────────────────────────
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4
#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7

void motor_control(int pwm) {
  if (pwm > 0) {
    digitalWrite(MOTOR1_ENA, HIGH); digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, pwm);
    digitalWrite(MOTOR2_ENA, HIGH); digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, HIGH); analogWrite(MOTOR1_PWM, -pwm);
    digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, HIGH); analogWrite(MOTOR2_PWM, -pwm);
  } else {
    digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, 0);
    digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, 0);
  }
}

// ── 엔코더 (LS7366R SPI) ────────────────────────────────────────────────────
#define ENC1_ADD 22
#define ENC2_ADD 23

void initEncoders() {
  pinMode(ENC1_ADD, OUTPUT); pinMode(ENC2_ADD, OUTPUT);
  digitalWrite(ENC1_ADD, HIGH); digitalWrite(ENC2_ADD, HIGH);
  SPI.begin();
  digitalWrite(ENC1_ADD, LOW);  SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, LOW);  SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC2_ADD, HIGH);
}

long readEncoder(int no) {
  unsigned int c1, c2, c3, c4;
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x60);
  c1 = SPI.transfer(0); c2 = SPI.transfer(0);
  c3 = SPI.transfer(0); c4 = SPI.transfer(0);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  return ((long)c1<<24)|((long)c2<<16)|((long)c3<<8)|(long)c4;
}

void clearEncoderCount(int no) {
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x98); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  delayMicroseconds(100);
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0xE0);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
}

// ── 속도 PID ────────────────────────────────────────────────────────────────
#define velo_Kp  12
#define velo_Ki  0.25
#define velo_Kd  15

#define VELO_PID_PERIOD_US  10000UL
#define SERIAL_TX_PERIOD_US 10000UL

long          Curr_val = 0, old_val = 0;
signed long   d_val = 0;
signed long   d_val_raw = 0;
unsigned long last_pid_us = 0;
unsigned long last_dt_us  = VELO_PID_PERIOD_US;

int           velo_val = 0;
float         Error = 0, d_Error = 0, old_Error = 0, Sum_Error = 0;
int           PWM = 0;

void Velo_PID_Control() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - last_pid_us;

  if (elapsed_us < VELO_PID_PERIOD_US) return;

  Curr_val  = -1L * readEncoder(1);
  d_val_raw = Curr_val - old_val;
  old_val   = Curr_val;

  float scale = (float)VELO_PID_PERIOD_US / (float)elapsed_us;
  d_val = (signed long)((float)d_val_raw * scale);

  last_dt_us = elapsed_us;

  Error = (float)velo_val - (float)d_val;

  float dt_ratio = (float)elapsed_us / (float)VELO_PID_PERIOD_US;
  Sum_Error += Error * dt_ratio;
  d_Error = (Error - old_Error) / dt_ratio;

  if (velo_val == 0) {
    PWM = 0;
    Sum_Error = 0;
    old_Error = 0;
  } else {
    PWM = (int)(velo_Kp * Error + velo_Ki * Sum_Error + velo_Kd * d_Error);
    if (PWM > 205) {
      PWM = 205;
      Sum_Error -= Error * dt_ratio;
    }
    if (PWM < -205) {
      PWM = -205;
      Sum_Error -= Error * dt_ratio;
    }
  }

  motor_control(PWM);

  if (elapsed_us > 2 * VELO_PID_PERIOD_US) {
    last_pid_us = now_us;
  } else {
    last_pid_us += VELO_PID_PERIOD_US;
  }

  old_Error = Error;
}

// ── 조향 PID (MsTimer2 20ms - 기존 유지) ────────────────────────────────
#define Steering_Sensor  A15
#define NEURAL_ANGLE     0
#define LEFT_STEER_ANGLE  -21
#define RIGHT_STEER_ANGLE  21
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

const int   AD_MIN = -460;
const int   AD_MAX =  423;

float Kp = 5.0, Ki_s = 1.8, Kd_s = 2.5;
const float STEER_DEADBAND = 1.0;
const float I_CLAMP   = 40.0;
const int   PWM_LIMIT = 255;
const int   PWM_SLEW  = 11;
const float STEER_LPF_FC = 12.0f;

double error_s = 0.0, error_old_s = 0.0, error_i_s = 0.0;
int    pwm_prev_s = 0, sensorValue = 0;
float  sensorValue_f = 0.0f;
int    Steer_Angle_Measure = 0, Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int pwm) {
  if (sensorValue >= AD_MAX || sensorValue <= AD_MIN) {
    digitalWrite(MOTOR3_ENA, LOW); digitalWrite(MOTOR3_ENB, LOW); analogWrite(MOTOR3_PWM, 0);
    return;
  }
  if (pwm > 0) {
    digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, HIGH); analogWrite(MOTOR3_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(MOTOR3_ENA, HIGH); digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, -pwm);
  } else {
    digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, 0);
  }
}

void Steer_PID_Control(float dt_s) {
  error_s = (double)Steering_Angle - (double)Steer_Angle_Measure;
  if (fabs(error_s) <= STEER_DEADBAND) {
    steer_motor_control(0); error_i_s = 0.0; pwm_prev_s = 0; error_old_s = error_s; return;
  }
  double p  = Kp * error_s;
  double d  = Kd_s * ((error_s - error_old_s) / dt_s);
  double ic = error_i_s + (Ki_s * error_s * dt_s);
  double u  = p + ic + d;
  double u0 = u;
  if (u >  PWM_LIMIT) u =  PWM_LIMIT;
  if (u < -PWM_LIMIT) u = -PWM_LIMIT;
  bool sat = (u != u0);
  if (!sat || ((u>0&&error_s<0)||(u<0&&error_s>0))) {
    error_i_s = ic;
    if (error_i_s >  I_CLAMP) error_i_s =  I_CLAMP;
    if (error_i_s < -I_CLAMP) error_i_s = -I_CLAMP;
  }
  int uc = (int)round(u);
  int du = uc - pwm_prev_s;
  if (du >  PWM_SLEW) uc = pwm_prev_s + PWM_SLEW;
  if (du < -PWM_SLEW) uc = pwm_prev_s - PWM_SLEW;
  steer_motor_control(uc);
  pwm_prev_s = uc; error_old_s = error_s;
}

void control_callback() {
  static unsigned long last_ms = 0;
  unsigned long now = millis();
  float dt_s = (now - last_ms) * 0.001f;
  if (dt_s <= 0.0f) dt_s = 0.02f;
  last_ms = now;

  int raw = analogRead(Steering_Sensor) - 512;
  float RC    = 1.0f / (6.283185f * STEER_LPF_FC);
  float alpha = dt_s / (RC + dt_s);
  sensorValue_f += alpha * ((float)raw - sensorValue_f);
  sensorValue    = (int)round(sensorValue_f);

  int sens = sensorValue;
  if (sens < AD_MIN) sens = AD_MIN;
  if (sens > AD_MAX) sens = AD_MAX;
  Steer_Angle_Measure = (int)round(
    ((double)(sens-AD_MIN)*(RIGHT_STEER_ANGLE-LEFT_STEER_ANGLE)/(double)(AD_MAX-AD_MIN))
    + LEFT_STEER_ANGLE);

  Steering_Angle = NEURAL_ANGLE + steer_angle;
  if (Steering_Angle < LEFT_STEER_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE;
  if (Steering_Angle > RIGHT_STEER_ANGLE) Steering_Angle = RIGHT_STEER_ANGLE;
  Steer_PID_Control(dt_s);
}

// ══════════════════════════════════════════════════════════════════════════
// PS2 컨트롤러 - 우선권 보장 핵심 영역
// ══════════════════════════════════════════════════════════════════════════
#define PS2_DAT 17
#define PS2_CMD 16
#define PS2_SEL 15
#define PS2_CLK 14
PS2X ps2x;
int  con_error = 1;
bool controller_true = false;
int  con_sp = 0;
#define pressures false
#define rumble    false

// PS2 폴링 주기: 30ms (50→33Hz)
// 다른 작업과 시간 다툼을 완화. 사람이 느끼는 응답성은 동일 수준.
#define PS2_POLL_PERIOD_US 30000UL
unsigned long last_ps2_us = 0;

// PS2 재설정 자동 시도 주기 (con_error != 0일 때)
#define PS2_RECONFIG_INTERVAL_MS 5000UL
unsigned long last_ps2_recfg_ms = 0;

// ── [핵심] 리모컨 우선권 윈도우 ────────────────────────────────────────
// L1/R1을 누르거나 스틱을 크게 움직인 시점부터 OVERRIDE_DURATION_MS 동안
// ROS 명령을 완전히 무시하고 리모컨 입력을 강제 유지.
// 한 번의 PS2 read 실패로 controller_true가 잠깐 false로 떨어져도
// 이 윈도우 안에서는 ROS가 갑자기 끼어들지 않는다.
#define OVERRIDE_DURATION_MS 300UL
#define STICK_ACTIVE_THRESHOLD 30   // 128(중립)에서 ±30 이상 벗어나면 활성
unsigned long remote_override_until_ms = 0;
int last_remote_sp    = 0;
int last_remote_steer = 0;

// PS2 통신 stale 감지: read_gamepad가 실패해도 con_error는 0인 채로 남는
// 라이브러리 버그 대응. 일정 시간 이상 버튼이 모두 0이고 스틱도 정확히
// 0 또는 0xFF만 나오면 통신 이상으로 간주.
unsigned long last_ps2_valid_ms = 0;

// ── PS2 read 동안 Serial RX 인터럽트만 임시 차단 ───────────────────────
// MsTimer2(조향 PID)와 Serial TX는 그대로 동작 → 안전
// UCSR0B의 RXCIE0(bit 7)만 토글한다. (Arduino Mega는 Serial=USART0)
static inline void ps2_safe_read() {
  uint8_t saved = UCSR0B;
  UCSR0B &= ~(1 << RXCIE0);   // RX 인터럽트 OFF
  ps2x.read_gamepad(false, 0);
  UCSR0B = saved;             // 복원
  // 차단 동안 HW RX 버퍼에 쌓인 바이트는 다음 RX 인터럽트에서 그대로 읽힘.
  // 즉 명령 손실 없음 (HW UART 자체는 동작).
}

void controller() {
  unsigned long now_ms = millis();
  unsigned long now_us = micros();

  // ── PS2 미연결 시 주기적으로 재설정 시도 ──
  if (con_error != 0) {
    if (now_ms - last_ps2_recfg_ms >= PS2_RECONFIG_INTERVAL_MS) {
      con_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
      last_ps2_recfg_ms = now_ms;
      if (con_error == 0) {
        last_ps2_valid_ms = now_ms;
      }
    }
    controller_true = false;
    return;
  }

  // ── 폴링 주기 도달했을 때만 ──
  if (now_us - last_ps2_us < PS2_POLL_PERIOD_US) {
    // 폴링 안 하는 사이에도 override 윈도우는 유지된다
    if (now_ms < remote_override_until_ms) {
      controller_true = true;
      // 이전 read 값을 그대로 유지 (last_remote_sp/steer는 변경 안 함)
    }
    return;
  }
  last_ps2_us = now_us;

  // ── PS2 read (Serial RX 인터럽트만 임시 차단) ──
  ps2_safe_read();

  // ── 입력 분석 ──
  bool l1r1     = ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1);
  int  ly       = ps2x.Analog(PSS_LY);
  int  rx_stick = ps2x.Analog(PSS_RX);

  // 스틱이 의미 있게 움직였는지 검사 (중립 128 기준)
  bool stick_active =
      (abs(ly - 128) >= STICK_ACTIVE_THRESHOLD) ||
      (abs(rx_stick - 128) >= STICK_ACTIVE_THRESHOLD);

  // PS2 read 유효성 휴리스틱:
  // L1/R1, 다른 버튼, 또는 스틱이 정상 범위(예외값 0/255 아닌)이면 통신 정상
  bool ps2_likely_alive =
      l1r1 ||
      ps2x.Button(PSB_PAD_UP)    || ps2x.Button(PSB_PAD_DOWN) ||
      ps2x.Button(PSB_PAD_LEFT)  || ps2x.Button(PSB_PAD_RIGHT) ||
      ps2x.Button(PSB_CROSS)     || ps2x.Button(PSB_CIRCLE) ||
      ps2x.Button(PSB_SQUARE)    || ps2x.Button(PSB_TRIANGLE) ||
      (ly       > 5 && ly       < 250) ||
      (rx_stick > 5 && rx_stick < 250);

  if (ps2_likely_alive) {
    last_ps2_valid_ms = now_ms;
  } else if (now_ms - last_ps2_valid_ms > 2000) {
    // 2초간 유효 신호 없으면 통신 stale로 판단하고 재설정
    con_error = 1;
    last_ps2_recfg_ms = now_ms - PS2_RECONFIG_INTERVAL_MS;  // 즉시 재시도
    controller_true = false;
    return;
  }

  // ── 리모컨 활성 판정 ──
  // L1/R1 누름 OR 스틱이 크게 움직인 상태
  if (l1r1 || stick_active) {
    // 새 명령 산출
    last_remote_sp    = -(map(ly, 0, 255, -10, 10)) * 10;
    last_remote_steer = -map(rx_stick, 0, 255, -21, 21);
    controller_true   = true;
    // [핵심] override 윈도우 갱신
    remote_override_until_ms = now_ms + OVERRIDE_DURATION_MS;
  } else {
    // L1/R1도 안 누르고 스틱도 중립
    // → override 윈도우가 끝났으면 ROS에 제어 양도
    if (now_ms >= remote_override_until_ms) {
      controller_true   = false;
      last_remote_sp    = 0;
      last_remote_steer = 0;
    }
    // 윈도우 안이면 controller_true=true 유지 (last_remote_*도 유지)
  }
}

// ── Serial 명령 파싱 ─────────────────────────────────────────────────────────
void process_rx_line(const char* line) {
  if (line[0]=='C' && line[1]==',') {
    const char* p  = line + 2;
    int v = atoi(p);
    const char* c2 = strchr(p, ',');
    if (!c2) return;
    int s = atoi(c2 + 1);

    if (v >  MAX_VEL) v =  MAX_VEL;
    if (v < -MAX_VEL) v = -MAX_VEL;
    if (s >  21) s =  21;
    if (s < -21) s = -21;

    velocity    = v;
    // ROS의 steer 명령은 리모컨이 활성이 아닐 때만 반영
    // (리모컨 활성 중에는 controller()에서 last_remote_steer를 steer_angle에 직접 씀)
    if (millis() >= remote_override_until_ms) {
      steer_angle = s;
    }
    return;
  }

  if (line[0]=='S' && line[1]==',') {
    int st = atoi(line + 2);
    state = (st != 0);
    if (!state) {
      velocity  = 0;
      Sum_Error = 0;
      motor_control(0);
    }
    return;
  }
}

void serial_rx_poll() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rx_line[rx_idx] = '\0';
      if (rx_idx > 0) process_rx_line(rx_line);
      rx_idx = 0;
      continue;
    }
    if (rx_idx < sizeof(rx_line)-1) rx_line[rx_idx++] = ch;
    else rx_idx = 0;
  }
}

// ── Serial 송신 ─────────────────────────────────────────────────────────
unsigned long last_tx_us = 0;

void serial_tx_poll() {
  unsigned long now_us = micros();
  if (now_us - last_tx_us < SERIAL_TX_PERIOD_US) return;
  if (Serial.availableForWrite() < 32) return;

  last_tx_us = now_us;

  Serial.print("E,");
#if DIAG_DT
  Serial.print(d_val);
  Serial.print(",");
  Serial.println(last_dt_us);
#else
  Serial.println(d_val);
#endif
}

// ── 감속 슬루율 ─────────────────────────────────────────────────────────
#define DECEL_STEP_PERIOD_US 50000UL
unsigned long last_decel_us = 0;

void decel_slew_update() {
  if (controller_true || state) {
    last_decel_us = micros();
    return;
  }
  unsigned long now_us = micros();
  if (now_us - last_decel_us < DECEL_STEP_PERIOD_US) return;
  last_decel_us = now_us;

  if (velo_val > 0) {
    velo_val--;
  } else if (velo_val < 0) {
    velo_val++;
  }
  if (velo_val == 0) {
    Sum_Error = 0;
  }
}

// ── setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    con_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    if (con_error == 0) break;
    delay(200);   // setup 단계 허용
  }

  pinMode(13, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_ENA, OUTPUT); pinMode(MOTOR1_ENB, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_ENA, OUTPUT); pinMode(MOTOR2_ENB, OUTPUT);
  initEncoders(); clearEncoderCount(1); clearEncoderCount(2);
  pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_ENA, OUTPUT); pinMode(MOTOR3_ENB, OUTPUT);

  Error = Sum_Error = d_Error = 0;
  old_val = 0; error_s = error_i_s = error_old_s = 0.0; pwm_prev_s = 0;

  MsTimer2::set(20, control_callback);
  MsTimer2::start();

  unsigned long now_us = micros();
  last_pid_us   = now_us;
  last_tx_us    = now_us;
  last_ps2_us   = now_us;
  last_decel_us = now_us;

  unsigned long now_ms = millis();
  last_ps2_recfg_ms        = now_ms;
  last_ps2_valid_ms        = now_ms;
  remote_override_until_ms = 0;

  Serial.println("Arduino Ready. PS2 priority mode, RX-IRQ guard, override 300ms");
}

// ── loop ────────────────────────────────────────────────────────────────────
void loop() {
  // 1) PS2 컨트롤러 폴링 (30ms 주기)
  controller();

  // 2) Serial 수신 처리
  //    controller()가 먼저 호출되어 remote_override_until_ms가 갱신된 뒤
  //    process_rx_line에서 그 윈도우를 확인해 ROS steer 무시 여부 결정
  serial_rx_poll();

  // 3) velo_val 결정 - 리모컨 우선
  unsigned long now_ms = millis();
  bool override_active = (now_ms < remote_override_until_ms);

  if (override_active || controller_true) {
    // ── 리모컨 우선권 발동 ──
    // ROS 명령을 완전히 무시하고 리모컨 직접 제어.
    // override 윈도우 안에서는 controller_true가 잠깐 false로 떨어져도
    // 직전 last_remote_* 값이 유지되어 ROS가 끼어들지 못함.
    motor_control(last_remote_sp);
    velo_val    = 0;
    Sum_Error   = 0;
    steer_angle = last_remote_steer;   // 조향도 리모컨 직접
  } else if (state) {
    // ROS2 자율주행
    velo_val = velocity;
  } else {
    // 정지 명령: 시간 기반 슬루 감속
    decel_slew_update();
  }

  // 4) 속도 PID 발동 (10ms 주기)
  Velo_PID_Control();

  // 5) Serial 송신 (10ms 주기)
  serial_tx_poll();
}
