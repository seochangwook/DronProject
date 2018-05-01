/*
 * HM-10은 아두이노 프로 마이크로의 TX0, RX1핀과 연결. HM-10의 TX, RX 핀은 각각 아두이노 프로 마이크로의 RX1, TX0 핀과 연결
 * Serial1 : 아두이노 프로 마이크로에서 사용하는 TX0, RX1 핀에 접근가능 (HM-10 블루투스 모듈로 연결해 사용)
 * Serial : USB로 연결된 가상 시리얼 포트를 접근가능.
 * 아두이노 프로 마이크로의 칩은 아두이노 레오나르도 보드가 사용하는 ATmega32u4를 사용
 */
/*void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200); //프로세싱과의 연결을 위해 시리얼을 사용//
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t userInput;

  if(Serial1.available() > 0){
    userInput = Serial1.read();

    Serial.println((char)userInput); //입력값을 출력
  }
}*/

/*
 * MSP 프로토콜
 * MSP 통신방식을 통해 드론에 메시지 전송
 * MSP : MultiWii Serial Protocol의 약자로 MultiWii flight controller(FC)와의 표준 통신 방법 제공
 * 3가지 구성 : command(명령), request(요청), response(응답)
 * - command : 안드로이드 앱에서 아두이노 드론으로 전달되는 명령 메시지 (header, size, type, payload, crc), (모터의 속도, 드론의 수평 회전각 
 * ,드론의 좌우 기울기, 전후 기울기 전달)
 * - request : 안드로이드 앱에서 아두이노 드론으로 전달되는 요청 메시지(Emergency와 Lock, Unlock이 사용), (header, size, type, crc)
 * - response : 아디우노 드론에서 안드로이드 앱으로 전달되는 응답 메시지 (header, size, type, payload, crc)
 */
/*void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200); //프로세싱과의 연결을 위해 시리얼을 사용//
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t mspData;

  if(Serial1.available() > 0){
    mspData = Serial1.read();

    Serial.print((char)mspData);
    Serial.print('\t');
    Serial.println(mspData, DEC); //10진수로 표현//
  }
}*/

/* MSP 패킷 명령 메시지 전송 주기 */
/*void setup(){
  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop(){
  static uint32_t t_now, t_prev;
  static uint32_t msp_period;

  if(Serial1.available()){
    uint8_t mspData = Serial1.read();

    if(mspData == '$'){
      t_now = micros();
      msp_period = t_now - t_prev;
      t_prev = t_now; //다음 시간을 측정하기 위해서 이전시간을 현재시간을 변경

      Serial.println(msp_period, DEC);
    }
  }
}*/

/* 명령 메시지 전송 시간 측정 : 드론을 안정적으로 띄우기 위해서는 적어도 100Hz정도로 loop문을 수행. 명령 메시지 전송 시간이 너무 길면 드론이 안정적으로
뜨는데 문제가 발생한다 */
/*void setup(){
  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop(){
  static uint32_t t_head1, t_crc;
  static uint32_t msp_time;
  static uint32_t cnt;

  if(Serial1.available()){
    uint8_t mspData = Serial1.read();

    if(mspData == '$'){
      t_head1 = micros();
      cnt = 0;
    } else{
      cnt++; //명령 데이터를 받을때마다 증가//
    }
    //cnt를 10까지 체크하는 이유는 하나의 command 패킷의 크기가 11이다. $ M < N T N-bytes(5) C ==> 11
    if(cnt == 10){
      t_crc = micros();
      msp_time = t_crc - t_head1; //가장 마지막인 CRC데이터를 받은 시간에서 처음 헤더의 값을 받았을 때의 시간하고 차이를 구한다.

      Serial.println(msp_time, DEC);
    }
  }
}*/

/* Roll, Pitch, Yaw, Throttle 값 추출 */
/* MSP command 데이터 크기가 11임을 고려하여 설계 */
void setup(){
  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop(){
  checkMspPacket();
}

//MSP command데이터 기준으로 열거형 정의//
enum {
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
  PACKETSIZE,
};
uint8_t mspPacket[PACKETSIZE]; //PACKETSIZE는 11//
///////////////////
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
        printMspPacket();
      }
    }
  }
}
////////////////////
void printMspPacket(){
  Serial.print((char)mspPacket[HEAD1]);
  Serial.print((char)mspPacket[HEAD2]);
  Serial.print((char)mspPacket[HEAD3]);
  Serial.print(mspPacket[DATASIZE]);
  Serial.print(' ');
  Serial.print(mspPacket[CMD]);
  Serial.print('\t');
  Serial.print("R:");
  Serial.print(mspPacket[ROLL]);
  Serial.print('\t');
  Serial.print("P:");
  Serial.print(mspPacket[PITCH]);
  Serial.print('\t');
  Serial.print("Y:");
  Serial.print(mspPacket[YAW]);
  Serial.print('\t');
  Serial.print("T:");
  Serial.print(mspPacket[THROTTLE]);
  Serial.print('\t');
  Serial.print(mspPacket[AUX]);
  Serial.print(' ');
  Serial.print(mspPacket[CRC]);
  Serial.print('\n');
}

