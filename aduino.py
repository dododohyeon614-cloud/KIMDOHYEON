// 하네스 T8 PID Control v6 (로스백 분석 기반 버그수정)
//
// ═══════════════════════════════════════════════════
// ★ v6 변경 사항 (로스백 분석으로 확인된 버그 수정)
// ═══════════════════════════════════════════════════
//
// [변경 1] MAX_VEL : 71 → 255 (상한 없음)
//   - 기존 71: ROS에서 83틱 명령 시 클램프→71 전달
//     → PID 오차 = 71 - 4틱 = 67틱 → Sum_Error 폭주
//     → PWM_raw > 205 → PWM = 60 급감 → 차 멈춤
//   - 수정: 아두이노는 255까지 허용, 상한은 ROS(driving.py)에서 71틱으로 관리
//   - 실제 명령은 driving.py publish_cmd가 71틱 이하로 보장
//
// [변경 2] PWM 안티 와인드업 클램프 방식 수정 (★ 핵심 버그)
//   - 기존: if(PWM > 205) PWM = 60;  ← PWM을 60으로 급감 → 모터 갑자기 약해짐
//   - 수정: if(PWM > 205) PWM = 205; ← 포화만 시킴 (급감 없음)
//           Sum_Error도 와인드업 방지용 역산으로 클램프
//
// [v5 유지] 클램프 버그 수정(±MAX_VEL), E-Stop 적분 초기화,
//           d_val 부호 포함 발행 로직 동일
//
// ═══════════════════════════════════════════════════
// 속도 대조표
//   35틱 = 1.485 m/s
//   47틱 = 1.993 m/s
//   59틱 = 2.501 m/s
//   71틱 = 3.011 m/s  ← ROS 측 상한 (driving.py에서 제한)
//   255틱 = 아두이노 하드웨어 상한 (실제 도달 불가)
// ═══════════════════════════════════════════════════

#include <MsTimer2.h>
#include <PS2X_lib.h>
#include <SPI.h>
#include <math.h>

// ── 전역 명령 변수 ─────────────────────────────────────────────────────────
int  velocity    = 0;   // 틱/20ms (부호 포함)
int  steer_angle = 0;   // 도 (-21 ~ +21)
bool state       = false;

// ── Serial 수신 버퍼 ────────────────────────────────────────────────────────
static char    rx_line[64];
static uint8_t rx_idx = 0;

// ── ★ [변경 1] 속도 상한 255 = 상한 없음 (ROS 측 driving.py에서 71틱으로 제한)
#define MAX_VEL 255

// ── 구동 모터 (Motor1, Motor2: 후륜 구동) ───────────────────────────────────
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

long          Curr_val = 0, old_val = 0;
signed long   d_val = 0;       // ★ [변경 4] 부호 포함 (후진=음수)
int           velo_val = 0;
unsigned long t_start = 0;
float         Error = 0, d_Error = 0, old_Error = 0, Sum_Error = 0;
int           PWM = 0;

void velo_control_callback() {
  Curr_val = -1L * readEncoder(1);   // 모터 방향에 맞게 부호 반전
  d_val    = Curr_val - old_val;
  old_val  = Curr_val;
}

void Velo_PID_Control() {
  unsigned long t_now = millis();
  if ((t_now - t_start) >= 10) {
    velo_control_callback();
    Error     = velo_val - d_val;
    d_Error   = Error - old_Error;
    Sum_Error += Error;

    if (velo_val == 0) {
      // ★ [변경 3] 정지 명령 시 PID 완전 초기화
      PWM       = 0;
      Sum_Error = 0;
      old_Error = 0;
    } else {
      PWM = (int)(velo_Kp * Error + velo_Ki * Sum_Error + velo_Kd * d_Error);
      // ★ [변경 2] 안티 와인드업: PWM 포화 클램프 (기존 60 급감 버그 수정)
      // 기존: if(PWM > 205) PWM = 60; → PWM 급감으로 모터 갑자기 약해지는 버그
      // 수정: 포화만 시키고, Sum_Error도 역산 클램프로 와인드업 방지
      if (PWM >  205) {
        PWM = 205;
        Sum_Error -= Error;   // 와인드업 방지: 이번 Error 적분 취소
      }
      if (PWM < -205) {
        PWM = -205;
        Sum_Error -= Error;   // 와인드업 방지: 이번 Error 적분 취소
      }
    }

    motor_control(PWM);
    t_start   = t_now;
    old_Error = Error;
  }
}

// ── 조향 PID ────────────────────────────────────────────────────────────────
#define Steering_Sensor  A15
#define NEURAL_ANGLE     0
#define LEFT_STEER_ANGLE  -21
#define RIGHT_STEER_ANGLE  21
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

const int   AD_MIN = -460;
const int   AD_MAX =  423;
float Kp = 6.5, Ki_s = 3.0, Kd_s = 1.0;
const float STEER_DEADBAND = 0.8;
const float I_CLAMP = 80.0;
const int   PWM_LIMIT = 255;
const int   PWM_SLEW  = 15;
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

// ── 20ms 조향 타이머 ────────────────────────────────────────────────────────
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

// ── PS2 컨트롤러 ────────────────────────────────────────────────────────────
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

void controller() {
  if (con_error != 0) return;
  ps2x.read_gamepad(false, 0);
  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
    controller_true = true;
    con_sp      = -(map(ps2x.Analog(PSS_LY), 0, 255, -10, 10)) * 10;
    steer_angle = -map(ps2x.Analog(PSS_RX), 0, 255, -21, 21);
  } else {
    controller_true = false;
    con_sp      = 0;
  }
}

// ── Serial 명령 파싱 ─────────────────────────────────────────────────────────
void process_rx_line(const char* line) {
  // C,vel,steer
  if (line[0]=='C' && line[1]==',') {
    const char* p  = line + 2;
    int v = atoi(p);
    const char* c2 = strchr(p, ',');
    if (!c2) return;
    int s = atoi(c2 + 1);

    // ★ [변경 2] 클램프 버그 수정: ±MAX_VEL 기준
    if (v >  MAX_VEL) v =  MAX_VEL;
    if (v < -MAX_VEL) v = -MAX_VEL;
    if (s >  21) s =  21;
    if (s < -21) s = -21;

    velocity    = v;
    steer_angle = s;
    return;
  }

  // S,0/1
  if (line[0]=='S' && line[1]==',') {
    int st = atoi(line + 2);
    state = (st != 0);
    // ★ [변경 3] E-Stop 시 적분 즉시 초기화
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

// ── setup / loop ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    con_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    if (con_error == 0) break;
    delay(200);
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
  t_start = millis();

  Serial.println("Arduino v6 Ready. MAX_VEL=255 (ROS manages 71-tick limit)");
}

void loop() {
  controller();
  serial_rx_poll();
  Velo_PID_Control();

  if (controller_true) {
    // 조종기 우선 (PS2)
    motor_control(con_sp);
    velo_val  = 0;
    Sum_Error = 0;
  } else if (state) {
    // ROS2 자율주행
    velo_val = velocity;
  } else {
    // E-Stop
    velo_val = 0;
  }

  // ★ [변경 4] d_val 부호 포함 발행 (후진=음수)
  Serial.print("E,");
  Serial.println(d_val);

  delay(10);   // CPU 과부하 방지 (delay(20) → delay(10))
}
