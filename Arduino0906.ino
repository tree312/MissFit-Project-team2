#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9150.h"
#include <RFduinoBLE.h>   //블루투스 통신을 위한 함수
#define pi 3.141592
MPU9150 accelGyroMag;

int num=0;
float arr[502];
int i,k,j=0;
double px, py, pz, qx, qy, qz;
long etime, stime;
int16_t ax, ay, az;   //가속도
int16_t gx, gy, gz;   //자이로
int16_t mx, my, mz;   //지자기
int16_t count1, count2;
float result,mid;
float mmax=-1;

double start;
double myaw, apit, arol;

float yaw, pitch, roll;   
//자세 의미하는 각도 -> yaw : z축 방향 회전, roll : 좌우회전. pitch : 중력방향 기준으로 얼마나 기울어져 있는지(얼마나 쏠렸는지)
double gyaw = 0, gpit = 0, grol = 0, pre_gyaw, pre_gpit, pre_grol;
int fyaw, fpitch, froll, ffyaw;
unsigned long used_millisec, pre_millisec = 0;
double Co_gain = 1;///change

#define addr 0x28 //I2C addr set
uint8_t uuid[16] = {0x02, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0F, 0x10};
//uint8_t uuid[16] = {0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0};
//사용자가 uuid바꿀수 있도록 함. sensor beacon pdf 10쪽
 
void setup() {
  Wire.begin(); //i2C통신 setup
  RFduinoBLE.iBeacon = true;  //iBeacon시작
  memcpy(RFduinoBLE.iBeaconUUID, uuid, sizeof(RFduinoBLE.iBeaconUUID));
  RFduinoBLE.iBeaconMajor = 1234;
  RFduinoBLE.iBeaconMinor =7891;
  
  RFduinoBLE.iBeaconMeasuredPower = 0xC6;   //1m 에서 측정된 iBeacon출력
  //고정 정보들 변경
 
  Serial.begin(9600);//must change   UART통신, usb포트 출력
  accelGyroMag.initialize();  //구축센서 초기화?
  Serial.print("initialize sucess ");
  Serial.print("               ");

  Serial.print("calibration....");
  Serial.print("               ");

  Serial.print("waiting....");

  for (i = 1 ; i <= 500 ; i++)
  {
    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);   //구축센서로부터 데이터값 받아옴
    px = 0 - mx;
    py = 0 - my;
    pz = 0 - mz;
    //x,y,z에 대한 지자기축을 px,py, pz로 바꿔줌

    qx = qx * (i - 1) / i + (px / i);
    qy = qy * (i - 1) / i + (py / i);
    qz = qz * (i - 1) / i + (pz / i);
  }

  yaw = atan2(my, mx) * 180 / pi;
  pitch = atan2(az, ax) * 180 / pi;
  roll = atan2(az, ay) * 180 / pi;    
  //atan2 = 아크탄젠트, 구축센서로 부터 받은 값을 이용하여 자세 계산함.

  Serial.println("ready...");
  Serial.println("GO");
}

void loop() {   //switch to lower power mode
  if(num == 0){
     dumbbell();   //num=0이면, dumbel함수 실행      
  }
}


void dumbbell(){
  used_millisec = millis() - pre_millisec;
  pre_millisec = millis();
  
  gyaw = pre_gyaw * used_millisec / 1000;
  gpit = pre_gpit * used_millisec / 1000;
  grol = pre_grol * used_millisec / 1000;

  yaw += gyaw * (Co_gain - 1) / Co_gain + (myaw - yaw) / Co_gain;
  pitch += gpit * (Co_gain - 1) / Co_gain + (apit - pitch) / Co_gain;
  roll += grol * (Co_gain - 1) / Co_gain + (arol - roll) / Co_gain;

  accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz); //구축센서로부터 값 받음
  
  myaw = atan2(my, mx) * 180 / pi;
  apit = atan2(az, ax) * 180 / pi;
  arol = atan2(az, ay) * 180 / pi;
  
  pre_gyaw = mz + qz;
  pre_gpit = my + qy;
  pre_grol = mx + qx;

  fyaw = (int)yaw;
  fpitch = (int)pitch;
  froll = (int)roll;

  mid = ax*ax + ay*ay + az*az; //가속도 값의 제곱수
  
  if(mid <0 )
  mid = (-1)*mid; //양수로 바꾸기
  
  result= sqrt(mid); //가속도 값의 크기
  ffyaw=fyaw;
  fyaw = (int)result; 
  
  //각 uuid 배열에 가속도 값; 넣기
  uuid[0]= (int)fyaw/10000;
  uuid[1] = (int)(fyaw%10000)/1000;
  uuid[2] = (int)(fyaw%1000)/100;
  uuid[3] = (int)(fyaw%100)%10;

 //uuid 배열에 각도 값 넣기   
 if(froll > 0) uuid[4] = 0;
  else {
    uuid[8] = 1;
    froll *= (-1);
  }
  uuid[5] = froll/100;
  uuid[6] = (froll%100)/10;
  uuid[7] = (froll%100)%10; 

/*
 *  각도 값을 5개의 범위로 나누어 상태에 따라 저장
 */
if(90<froll){ //수직보다 각도가 클 경우
  uuid[4]=0;
}else if(froll<=25&&froll>=0){ //각도가 0~25도 사이 
  uuid[4]=3;
}else if(froll<=60&&froll>25){ //각도가 25~60도 사이
  uuid[4]=2;
}else if(froll>60&&froll<90){ //각도가 60~90도 사이
  uuid[4]=1;
}else if(froll<0){ //각도가 수평 이하일 경우
  uuid[4]=4;
}
  uuid[5] = fpitch/100;
  uuid[6] = (fpitch%100)/10;
  uuid[7] = (fpitch%100)%10;
 
  memcpy(RFduinoBLE.iBeaconUUID, uuid, sizeof(RFduinoBLE.iBeaconUUID));   //사용자 iBeacon정보 갱신
  RFduinoBLE.begin();   
  RFduinoBLE_ULPDelay(20);  //설정시간동안 ultra low power delay실행
  RFduinoBLE.end();
}   
void RFduinoBLE_onReceive(char *data, int length){    //스마트폰으로 받은 데이터 리턴
  num=-1;
}
