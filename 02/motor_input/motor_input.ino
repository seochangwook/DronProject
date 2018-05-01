/* 4개의 모터 돌려보기 (4개의 모터는 아두이노 프로 마이크로의 5,6,9,10 번에 연결)
* 듀티 =  10
*/
/*void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(6,10);
  delay(1000);
  analogWrite(10,10);
  delay(1000);
  analogWrite(9,10);
  delay(1000);
  analogWrite(5,10);
  delay(1000);

  analogWrite(6,0);
  analogWrite(10, 0);
  analogWrite(9, 0);
  analogWrite(5,0);

  delay(2000);
}*/

/* 명령전송으로 모터회전 (앱에서 제어에 따른 속도변화)
* 모터의 속도조절 : 0~255단계로 조절가능(analogWrite)
* throttle : 모터의 속도 조절값
*/
#define THROTTLE_MAX 255
#define THROTTLE_MIN 0
#define THROTTLE_INI 5

enum{
  HEAD1,
  HEAD2,
  HEAD3,
  DATASIZE,
  CMD,
  ROLL,
  PICTH,
  YAW,
  THROTTLE,
  AUX,
  CRC,
  PACKETSIZE
};

uint8_t mspPacket[PACKETSIZE];

int throttle = THROTTLE_MIN;

//드론 프롭이 연결되어 있는 핀 정보//
int motorA_pin = 6;
int motorB_pin = 10;
int motorC_pin = 9;
int motorD_pin = 5;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200); //HM-10 BLE 모듈을 통해 드론의 모터 속도를 제어//

  initMotorSpeed(); //모터 속도 초기화//
}

void loop() {
  // put your main code here, to run repeatedly:
  controlMotorSpeed(); //모터 속도 제어//
}
//////////////////////////
void initMotorSpeed(){
  analogWrite(motorA_pin, THROTTLE_INI);
  delay(1000);
  analogWrite(motorB_pin, THROTTLE_INI);
  delay(1000);
  analogWrite(motorC_pin, THROTTLE_INI);
  delay(1000);
  analogWrite(motorD_pin, THROTTLE_INI);
  delay(1000);
}
//////////////////////////
void controlMotorSpeed(){
  checkMspPacket(); //MSP패킷 검사(블루투스 통신 값 받아오기)
  updateMotorSpeed(); //새로운 값으로 모터 속도 제어
}
//////////////////////////
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
        }
      }
    }
  }
}
///////////////////////////
void updateMotorSpeed(){
  analogWrite(motorA_pin, throttle);
  analogWrite(motorB_pin, throttle);
  analogWrite(motorC_pin, throttle);
  analogWrite(motorD_pin, throttle);
}

/* 명령전송으로 모터회전 (앱에서 제어에 따른 속도변화, 수평 회전 테스트)
* 모터의 속도조절 : 0~255단계로 조절가능(analogWrite)
* throttle : 모터의 속도 조절값
*/
/*#define THROTTLE_MAX 255
#define THROTTLE_MIN 0
#define THROTTLE_INI 5

enum{
  HEAD1,
  HEAD2,
  HEAD3,
  DATASIZE,
  CMD,
  ROLL,
  PICTH,
  YAW,
  THROTTLE,
  AUX,
  CRC,
  PACKETSIZE
};

uint8_t mspPacket[PACKETSIZE];

int throttle = THROTTLE_MIN;

//드론 프롭이 연결되어 있는 핀 정보//
int motorA_pin = 6;
int motorB_pin = 10;
int motorC_pin = 9;
int motorD_pin = 5;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200); //HM-10 BLE 모듈을 통해 드론의 모터 속도를 제어//

  initMotorSpeed(); //모터 속도 초기화//
}

void loop() {
  // put your main code here, to run repeatedly:
  controlMotorSpeed(); //모터 속도 제어//
}
//////////////////////////
void initMotorSpeed(){
  analogWrite(motorA_pin, THROTTLE_INI);
  delay(1000);
  analogWrite(motorB_pin, THROTTLE_INI);
  delay(1000);
  analogWrite(motorC_pin, THROTTLE_INI);
  delay(1000);
  analogWrite(motorD_pin, THROTTLE_INI);
  delay(1000);
}
//////////////////////////
void controlMotorSpeed(){
  checkMspPacket(); //MSP패킷 검사(블루투스 통신 값 받아오기)
  updateMotorSpeed(); //새로운 값으로 모터 속도 제어
}
//////////////////////////
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
        }
      }
    }
  }
}
///////////////////////////
void updateMotorSpeed(){
  //어느 한 프롭의 속도를 움직이지 않으면 회전한다.//
  analogWrite(motorA_pin, THROTTLE_MIN);
  analogWrite(motorB_pin, throttle);
  analogWrite(motorC_pin, THROTTLE_MIN);
  analogWrite(motorD_pin, throttle);
}*/
