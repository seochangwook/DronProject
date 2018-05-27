#include<Wire.h>

#define THROTTLE_MAX 255
#define THROTTLE_MIN 0

//MSP프로토콜 설정//
enum{
  HEAD1,
  HEAD2,
  HEAD3,
  DATASIZE,
  CMD,
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX,
  CRC,
  PACKETSIZE
};
uint8_t mspPacket[PACKETSIZE];

const int MPU_addr = 0x68; //MPU Address//
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //가속도 센서값, 자이로 센서값//

//Roll, Pitch, Yaw 각도를 저장하기 위한 변수, 상보필터 저장 변수//
float accel_angle_x, accel_angle_y, accel_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;

//메시지 출력을 위한 알려주는 변수. extern으로 선언 시 선언된 변수가 어딘가에 있다는 것을 아두이노 소프트웨어에 알릴 때 사용//
extern float roll_output, pitch_output, yaw_output;
extern float motorA_speed, motorB_speed, motorC_speed, motorD_speed;

//보정형 변수//
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;

//시간관련 값//
float dt;
unsigned long t_now;
unsigned long t_prev;

//자이로 센서를 이용한 각도구하기//
float gyro_x, gyro_y, gyro_z;

//이중루프PID에서 제어할 변수들(Roll, Pitch, Yaw)//
float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp = 1;
float roll_stabilize_ki = 0;
float roll_rate_kp = 1;
float roll_rate_ki = 0;
float roll_stabilize_iterm;
float roll_rate_iterm;
float roll_output;

float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 1;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 1;
float pitch_rate_ki = 0;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp = 1;
float yaw_stabilize_ki = 0;
float yaw_rate_kp = 1;
float yaw_rate_ki = 0;
float yaw_stabilize_iterm;
float yaw_rate_iterm;
float yaw_output;

float base_roll_target_angle;
float base_pitch_target_angle;
float base_yaw_target_angle;

extern float roll_target_angle;
extern float pitch_target_angle;
extern float yaw_target_angle;

//모터의 속도제어를 위한 값//
float throttle = 0;
float motorA_speed, motorB_speed, motorC_speed, motorD_speed;

//드론 프롭이 연결되어 있는 핀 정보//
int motorA_pin = 6;
int motorB_pin = 10;
int motorC_pin = 9;
int motorD_pin = 5;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200); //HM-10(블루투스)로 부터 데이터를 받기 위해서 선언//
  initMPU6050(); //MPU6050초기화//
  calibAccelGyro(); //가속도 자이로 센서의 초기 평균값을 구한다.//
  initDT(); //시간 간격 초기화//
  initYPR(); //Roll, Pitch, Yaw의 초기각도 값을 설정(평균을 구해 초기 각도로 설정, 호버링을 위한 목표 각도로 사용)//
  initMotorSpeed(); //모터의 속도를 초기화//
}
///////////////////////
void initMPU6050(){
  //MPU6050 초기화//
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true); //I2C의 제어권을 반환//
}
/////////////////////////
void calibAccelGyro(){
  float sumAcX = 0;
  float sumAcY = 0;
  float sumAcZ = 0;
  float sumGyX = 0;
  float sumGyY = 0;
  float sumGyZ = 0;

  readAccelGyro();

  //초기 보정값은 10번의 가속도 자이로 센서의 값을 받아 해당 평균값을 가진다.//
  for(int i=0; i<10; i++){
    readAccelGyro();

    sumAcX += AcX, sumAcY += AcY, sumAcZ += AcZ;
    sumGyX += GyX, sumGyY += GyY, sumGyZ += GyZ;

    delay(100);
  }

  baseAcX = sumAcX / 10;
  baseAcY = sumAcY / 10;
  baseAcZ = sumAcZ / 10;

  baseGyX = sumGyX / 10;
  baseGyY = sumGyY / 10;
  baseGyZ = sumGyZ / 10;
}
////////////////////////
void initDT(){
  t_prev = micros(); //초기 t_prev값은 근사값//
}
////////////////////////
void initYPR(){
  //초기 호버링의 각도를 잡아주기 위해서 Roll, Pitch, Yaw 상보필터 구하는 과정을 10번 반복한다.//
  for(int i=0; i<10; i++){
    readAccelGyro();
    calcDT();
    calcAccelYPR();
    calcGyroYPR();
    calcFilteredYPR();

    base_roll_target_angle += filtered_angle_y;
    base_pitch_target_angle += filtered_angle_x;
    base_yaw_target_angle += filtered_angle_z;

    delay(100);
  }

  //평균값을 구한다.//
  base_roll_target_angle /= 10;
  base_pitch_target_angle /= 10;
  base_yaw_target_angle /= 10;

  //초기 타겟 각도를 잡아준다.//
  roll_target_angle = base_roll_target_angle;
  pitch_target_angle = base_pitch_target_angle;
  yaw_target_angle = base_yaw_target_angle;
}
///////////////////////
void initMotorSpeed(){
  analogWrite(motorA_pin, THROTTLE_MIN);
  delay(1000);
  analogWrite(motorB_pin, THROTTLE_MIN);
  delay(1000);
  analogWrite(motorC_pin, THROTTLE_MIN);
  delay(1000);
  analogWrite(motorD_pin, THROTTLE_MIN);
  delay(1000);
}
////////////////////////
void loop() {
  // put your main code here, to run repeatedly:
  readAccelGyro();
  
  calcDT();
  
  calcAccelYPR(); //가속도 센서 Roll, Pitch, Yaw의 각도를 구하는 루틴//
  calcGyroYPR(); //자이로 센서 Roll, Pitch, Yaw의 각도를 구하는 루틴//
  calcFilteredYPR(); //상보필터를 적용해 Roll, Pitch, Yaw의 각도를 구하는 루틴//

  calcYPRtoDualPID(); //이중루프PID구현//
  calcMotorSpeed(); //PID출력값을 구한것을 기준으로 모터의 속도를 계산한다.//
  
  checkMspPacket();
  updateMotorSpeed();
}
///////////////////////
void readAccelGyro(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);  //I2C의 제어권을 이어간다//
  Wire.requestFrom(MPU_addr, 14, true);

  //가속도, 자이로 센서의 값을 읽어온다.//
  AcX = Wire.read() << 8|Wire.read();
  AcY = Wire.read() << 8|Wire.read();
  AcZ = Wire.read() << 8|Wire.read();
  Tmp = Wire.read() << 8|Wire.read();
  GyX = Wire.read() << 8|Wire.read();
  GyY = Wire.read() << 8|Wire.read();
  GyZ = Wire.read() << 8|Wire.read();
}
///////////////////////
void calcDT(){
  t_now = micros();
  dt = (t_now - t_prev) / 1000000.0;
  t_prev = t_now;
}
////////////////////////
void calcAccelYPR(){
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / 3.14159;

  accel_x = AcX - baseAcX;
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);

  //accel_angle_y는 Roll각을 의미//
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;

  //accel_angle_x는 Pitch값을 의미//
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_angle_z = 0; //중력 가속도(g)의 방향과 정반대의 방향을 가리키므로 가속도 센서를 이용해서는 회전각을 계산할 수 없다.//
}
/////////////////////////
void calcGyroYPR(){
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;

  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
}
/////////////////////////
void calcFilteredYPR(){
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + gyro_x * dt;
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  //상보필터 값 구하기(가속도, 자이로 센서의 절충)//
  filtered_angle_x = ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0-ALPHA) * accel_angle_y;
  filtered_angle_z = tmp_angle_z;
}
//////////////////////
void dualPID(float target_angle,
             float angle_in,
             float rate_in,
             float stabilize_kp,
             float stabilize_ki,
             float rate_kp,
             float rate_ki,
             float &stabilize_iterm,
             float &rate_iterm,
             float &output
){
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;

  //이중루프PID알고리즘//
  angle_error = target_angle - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;
  stabilize_iterm += stabilize_ki * angle_error * dt; //안정화 적분항//

  desired_rate = stabilize_pterm;

  rate_error = desired_rate - rate_in;

  rate_pterm = rate_kp * rate_error; //각속도 비례항//
  rate_iterm += rate_ki * rate_error * dt; //각속도 적분항//

  output = rate_pterm + rate_iterm + stabilize_iterm; //최종 출력 : 각속도 비례항 + 각속도 적분항 + 안정화 적분항//
}
///////////////////////
void calcYPRtoDualPID(){
  roll_angle_in = filtered_angle_y;
  roll_rate_in = gyro_y;

  dualPID(roll_target_angle,
          roll_angle_in,
          roll_rate_in,
          roll_stabilize_kp,
          roll_stabilize_ki,
          roll_rate_kp,
          roll_rate_ki,
          roll_stabilize_iterm,
          roll_rate_iterm,
          roll_output
  );

  pitch_angle_in = filtered_angle_x;
  pitch_rate_in = gyro_x;

  dualPID(pitch_target_angle,
          pitch_angle_in,
          pitch_rate_in,
          pitch_stabilize_kp,
          pitch_stabilize_ki,
          pitch_rate_kp,
          pitch_rate_ki,
          pitch_stabilize_iterm,
          pitch_rate_iterm,
          pitch_output
  );

  yaw_angle_in = filtered_angle_z;
  yaw_rate_in = gyro_z;

  dualPID(yaw_target_angle,
          yaw_angle_in,
          yaw_rate_in,
          yaw_stabilize_kp,
          yaw_stabilize_ki,
          yaw_rate_kp,
          yaw_rate_ki,
          yaw_stabilize_iterm,
          yaw_rate_iterm,
          yaw_output
  );
}
///////////////////////
void calcMotorSpeed(){

  //모터 속도 보정(호버링)//
  motorA_speed = (throttle == 0) ? 0 : throttle + yaw_output + roll_output + pitch_output + 20;
  motorB_speed = (throttle == 0) ? 0 : throttle - yaw_output - roll_output + pitch_output + 20;
  motorC_speed = (throttle == 0) ? 0 : throttle + yaw_output - roll_output - pitch_output + 45;
  motorD_speed = (throttle == 0) ? 0 : throttle - yaw_output + roll_output - pitch_output + 45;

  //아날로그의 PWM값은 0~255이므로 각 경계값마다의 보정작업//
  if(motorA_speed < 0){
    motorA_speed = 0;
  } if(motorA_speed > 255){
    motorA_speed = 255;
  } if(motorB_speed < 0){
    motorB_speed = 0;
  } if(motorB_speed > 255){
    motorB_speed = 255;
  } if(motorC_speed < 0){
    motorC_speed = 0;
  } if(motorC_speed > 255){
    motorC_speed = 255;
  } if(motorD_speed < 0){
    motorD_speed = 0;
  } if(motorD_speed > 255){
    motorD_speed = 255;
  }
}
/////////////////////////
void checkMspPacket(){
  static uint32_t cnt;

  if(Serial1.available() > 0){
    while(Serial1.available() > 0){
      uint8_t mspData = Serial1.read();

      if(mspData == '$'){
        cnt = HEAD1;
      } else{
        cnt++;
      }

      mspPacket[cnt] = mspData;

      if(cnt == CRC){
        if(mspPacket[CMD] == 150){
          throttle = mspPacket[THROTTLE]; //모터의 속도값을 가져온다//

          //roll, pitch, yaw 컨트롤//
          roll_target_angle = base_roll_target_angle;
          pitch_target_angle = base_pitch_target_angle;
          yaw_target_angle = base_yaw_target_angle;

          //기준값이 0이므로 전송받는 기본값인 125를 뺀다.Pitch, Yaw는 부호가 반대여야지 정상적인 컨트롤이 가능
          roll_target_angle += mspPacket[ROLL]-125;
          pitch_target_angle += -(mspPacket[PITCH]-125);
          yaw_target_angle += -(mspPacket[YAW]-125);
        }
      }
    }
  }
}
///////////////////////////
void updateMotorSpeed(){
  //어느 한 프롭의 속도를 움직이지 않으면 회전한다.//
  analogWrite(motorA_pin, motorA_speed);
  analogWrite(motorB_pin, motorB_speed);
  analogWrite(motorC_pin, motorC_speed);
  analogWrite(motorD_pin, motorD_speed);
}
