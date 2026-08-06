// 하네스 T8 PID Control Vr.1 (ROS2 Humble 호환: Raw Serial Protocol 적용)
// - ROS1/rosserial 제거
// - PC->Arduino: "C,velocity,steer\n" / "S,0|1\n"
// - Arduino->PC: "E,d_val,sensorValue,PWM,Steering_Angle,state\n"
// - 나머지 제어 로직(PID/모터/PS2)은 변경하지 않음 (통신부만 변경)

#include <MsTimer2.h>
#include <PS2X_lib.h> // for v1.6
#include <SPI.h>
#include <math.h>

// ===================== ROS I/O (Raw Serial Protocol) =====================
int velocity = 0;
int steer_angle = 0;
bool state = false;

// ======= [추가] Serial RX 파서 버퍼 =======
static char rx_line[64];
static uint8_t rx_idx = 0;

// ===================== Front Motor Drive (구동) =====================
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7

void motor_control(int motor_pwm_output)
{
  if (motor_pwm_output > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor_pwm_output);

    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, motor_pwm_output);
  }
  else if (motor_pwm_output < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor_pwm_output);

    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, -motor_pwm_output);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);

    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }
}

// ===================== Encoders (SPI) =====================
#define ENC1_ADD 22
#define ENC2_ADD 23
signed long encoder1count = 0;
signed long encoder2count = 0;

void initEncoders() {
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT);
  digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, HIGH);
  SPI.begin();

  // Encoder 1 init
  digitalWrite(ENC1_ADD, LOW);
  SPI.transfer(0x88);  // write MDR0
  SPI.transfer(0x03);  // 4-byte mode, x4 quad
  digitalWrite(ENC1_ADD, HIGH);

  // Encoder 2 init
  digitalWrite(ENC2_ADD, LOW);
  SPI.transfer(0x88);
  SPI.transfer(0x03);
  digitalWrite(ENC2_ADD, HIGH);
}

long readEncoder(int encoder_no)
{
  unsigned int c1, c2, c3, c4;
  long v;
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);
  SPI.transfer(0x60); // read CNTR
  c1 = SPI.transfer(0x00);
  c2 = SPI.transfer(0x00);
  c3 = SPI.transfer(0x00);
  c4 = SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);
  v = ((long)c1 << 24) + ((long)c2 << 16) + ((long)c3 << 8) + (long)c4;
  return v;
}

void clearEncoderCount(int encoder_no) {
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);
  SPI.transfer(0x98);  // write DTR
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);

  delayMicroseconds(100);

  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);
  SPI.transfer(0xE0);  // load DTR -> CNTR
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);
}

// ===================== Motor Velocity PID =====================
#define velo_Kp  12
#define velo_Ki  0.25
#define velo_Kd  15

long dt_ms = 0;
long Curr_val, old_val, d_val;  // d_val: encoder 증가량(속도 proxy)
int  velo_val = 0;

unsigned long t_start;
float Error, d_Error, old_Error, Sum_Error;
int PWM = 0;

void velo_control_callback() {
  Curr_val = -1 * readEncoder(1);
  d_val = Curr_val - old_val;

  // [중요] 기존 디버그 출력은 프로토콜을 깨므로 제거
  // Serial.print(velo_val); Serial.print(",");
  // Serial.print(d_val);    Serial.print(",");

  old_val = Curr_val;
}

void Velo_PID_Control() {
  unsigned long t_now = millis();
  if ((t_now - t_start) >= 10) { // 100Hz
    velo_control_callback();

    Error = velo_val - d_val;
    d_Error = Error - old_Error;
    Sum_Error += Error;

    dt_ms = (long)(t_now - t_start);

    PWM = (int)(velo_Kp * Error + velo_Ki * Sum_Error + velo_Kd * d_Error);

    // [중요] 기존 디버그 출력은 프로토콜을 깨므로 제거
    // Serial.print(PWM);
    // Serial.println(",#");

    if (velo_val == 0) {
      PWM = 0;
    } else {
      if (PWM >= 205)  PWM = 60;
      if (PWM <= -205) PWM = -60;
    }

    motor_control(PWM);

    t_start = t_now;
    old_Error = Error;
  }
}

// ===================== Steering PID (안정화 개정) =====================
#define Steering_Sensor A15
#define NEURAL_ANGLE 0
#define LEFT_STEER_ANGLE  -21
#define RIGHT_STEER_ANGLE  21
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

const int AD_MIN = -460;
const int AD_MAX =  423;

float Kp = 6.5;
float Ki = 3.0;
float Kd = 1.0;

const float STEER_DEADBAND_DEG = 0.8;
const float I_CLAMP = 80.0;
const int   PWM_LIMIT = 255;
const int   PWM_SLEW_PER_TICK = 15;
const float STEER_LPF_FC = 12.0f;

double error, error_old = 0.0;
double error_s = 0.0;
int pwm_output_prev = 0;

int sensorValue = 0;
float sensorValue_f = 0.0f;
int Steer_Angle_Measure = 0;
int Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int motor_pwm)
{
  if ( (sensorValue >= AD_MAX) || (sensorValue <= AD_MIN) )
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
    return;
  }

  if (motor_pwm > 0) {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void Steer_PID_Control(float dt_s)
{
  error = (double)Steering_Angle - (double)Steer_Angle_Measure;

  if (fabs(error) <= STEER_DEADBAND_DEG) {
    steer_motor_control(0);
    error_s = 0.0;
    pwm_output_prev = 0;
    error_old = error;
    return;
  }

  double p_term = Kp * error;
  double d_term = Kd * ((error - error_old) / dt_s);

  double i_candidate = error_s + (Ki * error * dt_s);

  double u = p_term + i_candidate + d_term;
  double u_unsat = u;

  if (u > PWM_LIMIT) u = PWM_LIMIT;
  if (u < -PWM_LIMIT) u = -PWM_LIMIT;

  bool saturated = (u != u_unsat);
  if (!saturated || ((u > 0 && error < 0) || (u < 0 && error > 0))) {
    error_s = i_candidate;
    if (error_s > I_CLAMP)  error_s = I_CLAMP;
    if (error_s < -I_CLAMP) error_s = -I_CLAMP;
  }

  int u_cmd = (int)round(u);
  int du = u_cmd - pwm_output_prev;
  if (du >  PWM_SLEW_PER_TICK) u_cmd = pwm_output_prev + PWM_SLEW_PER_TICK;
  if (du < -PWM_SLEW_PER_TICK) u_cmd = pwm_output_prev - PWM_SLEW_PER_TICK;

  steer_motor_control(u_cmd);
  pwm_output_prev = u_cmd;
  error_old = error;
}

void steer_velo_control(float dt_s)
{
  if (Steering_Angle <= LEFT_STEER_ANGLE + NEURAL_ANGLE)
    Steering_Angle = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE + NEURAL_ANGLE)
    Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;

  Steer_PID_Control(dt_s);
}

void control_callback()
{
  static boolean led = HIGH;
  digitalWrite(0, led);
  led = !led;

  static unsigned long last_ms = millis();
  unsigned long now = millis();
  float dt_s = (now - last_ms) * 0.001f;
  if (dt_s <= 0.0f) dt_s = 0.02f;
  last_ms = now;

  int raw = analogRead(Steering_Sensor) - 512;
  float RC = 1.0f / (6.283185f * STEER_LPF_FC);
  float alpha = dt_s / (RC + dt_s);
  sensorValue_f = sensorValue_f + alpha * ((float)raw - sensorValue_f);
  sensorValue = (int)round(sensorValue_f);

  int sens = sensorValue;
  if (sens < AD_MIN) sens = AD_MIN;
  if (sens > AD_MAX) sens = AD_MAX;

  Steer_Angle_Measure = (int)round(
                          ( (double)(sens - AD_MIN) * (RIGHT_STEER_ANGLE - LEFT_STEER_ANGLE) / (double)(AD_MAX - AD_MIN) )
                          + LEFT_STEER_ANGLE
                        );

  Steering_Angle = NEURAL_ANGLE + steer_angle;

  steer_velo_control(dt_s);
}

// ===================== Controller (PS2) =====================
#define PS2_DAT 17
#define PS2_CMD 16
#define PS2_SEL 15
#define PS2_CLK 14

PS2X ps2x;
byte type = 0;
byte vibrate = 0;
uint8_t spd = 0;
bool old_1 = 0;
int con_st = 0;
int con_sp = 0;
int con_error = 0;
bool controller_true = false;

#define pressures   false
#define rumble      false

void controller() {
  ps2x.read_gamepad(false, vibrate);

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
    controller_true = true;
    con_sp = - (map(ps2x.Analog(PSS_LY), 0, 255, -10, 10)) * 10;
    con_st = map(ps2x.Analog(PSS_RX), 0, 255, -21, 21);

    steer_angle = -con_st;
  } else {
    controller_true = false;
    con_sp = 0;
    con_st = 0;
  }
}

// ===================== [추가] Raw Serial 수신 파싱 =====================
// 수신 라인: "C,vel,steer" 또는 "S,0|1"
void process_rx_line(const char* line)
{
  // C 프레임
  if (line[0] == 'C' && line[1] == ',') {
    // C,<vel>,<steer>
    const char* p = line + 2;
    int v = atoi(p);

    const char* c2 = strchr(p, ',');
    if (c2 == NULL) return;
    int s = atoi(c2 + 1);

    velocity = v;
    steer_angle = s;

    // 기존 제한 로직 유지
    if (velocity >= 120) velocity = 10;
    if (velocity <= -120) velocity = -10;

    return;
  }

  // S 프레임
  if (line[0] == 'S' && line[1] == ',') {
    int st = atoi(line + 2);
    state = (st != 0);
    return;
  }
}

// 바이트 단위로 읽어서 '\n'에서 라인 완성
void serial_rx_poll()
{
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == '\r') continue;

    if (ch == '\n') {
      rx_line[rx_idx] = '\0';
      if (rx_idx > 0) {
        process_rx_line(rx_line);
      }
      rx_idx = 0;
      continue;
    }

    if (rx_idx < (sizeof(rx_line) - 1)) {
      rx_line[rx_idx++] = ch;
    } else {
      // overflow -> reset
      rx_idx = 0;
    }
  }
}

// ===================== Arduino setup/loop =====================
void setup() {
  Serial.begin(57600);

  con_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  if (con_error == 0) {
    // 기존 출력은 유지 가능하나, driver.py 파싱 혼선 방지 위해 권장: 제거
    // Serial.println("Controller OK");
  }

  type = ps2x.readType();
  // 기존 타입 출력도 driver.py에 혼선을 줄 수 있으므로 권장: 제거
  // switch (type) { ... }

  pinMode(13, OUTPUT);

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR1_ENB, OUTPUT);

  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);

  initEncoders();
  clearEncoderCount(1);
  clearEncoderCount(2);

  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);
  pinMode(MOTOR3_ENB, OUTPUT);

  Error = Sum_Error = d_Error = old_val = 0;
  error = error_s = error_old = 0.0;
  pwm_output_prev = 0;

  MsTimer2::set(20, control_callback);
  MsTimer2::start();

  t_start = millis();
}

void loop() {
  controller();

  // [중요] PC에서 오는 Serial 명령을 항상 반영
  serial_rx_poll();

  Velo_PID_Control();

  if (controller_true == true) {
    motor_control(con_sp);
    velo_val = 0;
  } else {
    velo_val = velocity;
  }

  // [중요] Arduino -> PC 텔레메트리: E 프레임만 송신
  // driver.py는 E만 읽습니다.
  Serial.print("E,");
  Serial.print(d_val);
  Serial.print(",");
  Serial.print(sensorValue);
  Serial.print(",");
  Serial.print(PWM);
  Serial.print(",");
  Serial.print(Steering_Angle);
  Serial.print(",");
  Serial.println(state ? 1 : 0);

  delay(20);
}