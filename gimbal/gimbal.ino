#include <Studuino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

Studuino board;
const int SAMPLING_PERIOD = 50; // milli sec
uint32_t next_sampling_time = 0;
const float RAD2DEG = 57.2958; // 180/pi


float phi, theta, phi_ref, theta_ref;
float ie_phi = 0.0;
float ie_theta = 0.0;

void calc_phi_theta_from_accel(float* phi_a, float* theta_a) {
  // 加速度センサ値を取得してオイラー角phi, thetaを計算
  float ax = board.GetGyroscopeValue(X_AXIS) / 16384.0; // センサ出力を分解能で割って加速度(G)に変換
  float ay = board.GetGyroscopeValue(Y_AXIS) / 16384.0;
  float az = board.GetGyroscopeValue(Z_AXIS) / 16384.0;

  *phi_a = atan2(ay, az);
  *theta_a = atan2(- ax, sqrt(ay * ay + az * az));
}

void calc_dphi_dtheta_from_gyro(float* phi_dot, float* theta_dot, float phi_cur, float theta_cur) {
  // ジャイロセンサ値を取得してオイラー角速度phi_dot, theta_dotを計算
  // phi_cur, theta_curは現在のオイラー角
  float wx = board.GetGyroscopeValue(GX_AXIS) / 131.0 / RAD2DEG; // 分解能で割って角速度(rad per sec)に変換する
  float wy = board.GetGyroscopeValue(GY_AXIS) / 131.0 / RAD2DEG;
  float wz = board.GetGyroscopeValue(GZ_AXIS) / 131.0 / RAD2DEG;

  *phi_dot = wx + sin(phi_cur) * tan(theta_cur) * wy + cos(phi_cur) * tan(theta_cur) * wz;
  *theta_dot = cos(phi_cur) * wy - sin(phi_cur) * wz;
}


void setup() {
  board.InitI2CPort(PIDGYROSCOPE); // 加速度ジャイロセンサの初期化
  board.InitDCMotorPort(PORT_M1); // DC モーターが接続されている M1 ポートを初期化
  board.InitDCMotorPort(PORT_M2);
  calc_phi_theta_from_accel(&phi, &theta); // オイラー角度のグローバル変数に初期値を入れる
  Serial.begin(115200);
  next_sampling_time = millis();
}

void loop() {
  if (millis() >= next_sampling_time) {
    // 加速度センサ値からオイラー角算出
    float phi_a, theta_a;
    calc_phi_theta_from_accel(&phi_a, &theta_a);

    // ジャイロセンサ値からオイラー角速度算出
    float phi_dot, theta_dot;
    calc_dphi_dtheta_from_gyro(&phi_dot, &theta_dot, phi, theta);

    // 相補フィルタに通す
    float tau = 0.6; // cut-off frequency
    float a = tau / (tau + SAMPLING_PERIOD * 0.001);
    phi = a * phi + (1 - a) * (phi_a + tau * phi_dot);
    theta = a * theta + (1 - a) * (theta_a + tau * theta_dot);

    float u, e;
    // PID control of phi
    float kp = 450.0;
    float ki = 0.0;
    float kd = 30.0;
    e = phi_ref - phi;
    ie_phi += e * (SAMPLING_PERIOD * 0.001);

    float u1 = kp * e + ki * ie_phi + kd * (-phi_dot);
    if (u1 >= 0) {
      board.DCMotorControl(PORT_M1, NORMAL);
      u = u1;
    } else {
      board.DCMotorControl(PORT_M1, REVERSE);
      u = -u1;
    }
    // saturation
    if (u > 255) {
      u = 255;
    }
    board.DCMotorPower(PORT_M1, (byte)u);

    // PID control of phi
    kp = 400.0;
    ki = 0.0;
    kd = 30.0;
    e = theta_ref - theta;
    ie_theta += e * (SAMPLING_PERIOD * 0.001);

    float u2 = kp * e + ki * ie_theta + kd * (-theta_dot);
    if (u2 >= 0) {
      board.DCMotorControl(PORT_M2, NORMAL);
      u = u2;
    } else {
      board.DCMotorControl(PORT_M2, REVERSE);
      u = -u2;
    }
    // saturation
    if (u > 255) {
      u = 255;
    }
    board.DCMotorPower(PORT_M2, (byte)u);

    // print
    Serial.print(phi * RAD2DEG);
    Serial.print(",");
    Serial.print(theta * RAD2DEG);
    Serial.print(",");
    Serial.print(u1);
    Serial.print(",");
    Serial.println(u2);

    next_sampling_time += SAMPLING_PERIOD;
  }
}
