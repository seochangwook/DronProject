#include<Wire.h>

const int MPU_addr = 0x68; //MPU Address//
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //가속도 센서값, 자이로 센서값//

//세가지 형태의 Roll, Pitch, Yaw 각도를 저장하기 위한 변수//
float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;

//보정형 변수//
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;

//시간관련 값//
unsigned long t_now;
unsigned long t_prev;

//자이로 센서를 이용한 각도구하기//
float gyro_x, gyro_y, gyro_z;

void setup() {
  // put your setup code here, to run once:
  initMPU6050(); //MPU6050초기화//
  Serial.begin(115200);
  calibAccelGyro(); //가속도 자이로 센서의 초기 평균값을 구한다.//
  initDT(); //시간 간격 초기화//
  accelNoiseTest();
}

void loop() {
  // put your main code here, to run repeatedly:
  readAccelGyro();
  calcDT();
  
  calcAccelYPR(); //가속도 센서 Roll, Pitch, Yaw의 각도를 구하는 루틴//
  calcGyroYPR(); //자이로 센서 Roll, Pitch, Yaw의 각도를 구하는 루틴//
  calcFilteredYPR(); //상보필터를 적용해 Roll, Pitch, Yaw의 각도를 구하는 루틴//

  static int cnt;
  cnt++;

  if(cnt%2 == 0){
    SendDataToProcessing(); //프로세싱으로 Roll, Pitch, Yaw값을 전송//
  }
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
////////////////////////
void SendDataToProcessing(){
  Serial.print(F("DEL:"));
  Serial.print(dt, DEC);
  Serial.print(F("#ACC:"));
  Serial.print(accel_angle_x, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_y, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_z, 2);
  Serial.print(F("#GYR:"));
  Serial.print(gyro_angle_x, 2);
  Serial.print(F(","));
  Serial.print(gyro_angle_y, 2);
  Serial.print(F(","));
  Serial.print(gyro_angle_z, 2);
  Serial.print(F("#FIL:"));
  Serial.print(filtered_angle_x);
  Serial.print(F(","));
  Serial.print(filtered_angle_y);
  Serial.print(F(","));
  Serial.print(filtered_angle_z);
  Serial.print(F("\n"));

  delay(5);
}
//////////////////////////
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

  gyro_angle_x += gyro_x * dt;
  gyro_angle_y += gyro_y * dt;
  gyro_angle_z += gyro_z * dt;
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
/////////////////////////
void accelNoiseTest(){
  analogWrite(6, 40);
  analogWrite(10, 40);
  analogWrite(9,40);
  analogWrite(5, 40);
}

