#define F_CPU 16000000
#define BAUD_RATE 9600

#include <math.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>

#include <avr/interrupt.h>
//#include <avr/io.h>
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(9,8);
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(9,8);

#define _PORTA (*(volatile unsigned char*) 0x0400)
#define _PORTA_DIR (*(volatile unsigned char*) (0x0400 + 0x00))
#define _PORTA_OUT (*(volatile unsigned char*) (0x0400 + 0x04))
#define _PORTC (*(volatile unsigned char*) 0x0440)
#define _PORTC_DIR (*(volatile unsigned char*) (0x0440 + 0x00))
#define _PORTC_OUT (*(volatile unsigned char*) (0x0440 + 0x04))
#define _PORTC_PIN2CTRL (*(volatile unsigned char*) (0x0440 + 0x12))
#define _PORTC_PIN2CTRL_PULLUPEN_bm ((unsigned char) 0b00001000)
#define _PORTC_PIN3CTRL (*(volatile unsigned char*) (0x0440 + 0x13))
#define _PORTC_PIN3CTRL_PULLUPEN_bm ((unsigned char) 0b00001000)
#define _PORTF (*(volatile unsigned char*) 0x04A0)
#define _PORTF_DIR (*(volatile unsigned char*) (0x04A0 + 0x00))
#define _PORTF_OUT (*(volatile unsigned char*) (0x04A0 + 0x04))
#define _PORTMUX (*(volatile unsigned char*) 0x05E0)
#define _PORTMUX_TWISPIROUTEA (*(volatile unsigned char*) (0x05E0 + 0x03))
#define _PORTMUX_TWI0_ALT2_bm ((unsigned char) 0b00100000)
#define _PORTMUX_TCBROUTEA (*(volatile unsigned char*) (0x05E0 + 0x05))
#define _PORTMUX_TCB1_ALT1_gc ((unsigned char) 0b00000010)
#define _PORTMUX_TCB0_ALT1_gc ((unsigned char) 0b00000001)

#define _TWI0 (*(volatile unsigned char*) 0x08A0)
#define _TWI0_MCTRLA (*(volatile unsigned char*) (0x08A0 + 0x03))
#define _TWI_ENABLE_bm ((unsigned char) 0b00000001)
#define _TWI0_MCTRLB (*(volatile unsigned char*) (0x08A0 + 0x04))
#define _TWI_ACKACT_NACK_gc ((unsigned char) 0b00000100)
#define _TWI_MCMD_RECVTRANS_gc ((unsigned char) 0b00000010)
#define _TWI_MCMD_STOP_gc ((unsigned char) 0b00000011)
#define _TWI0_MSTATUS (*(volatile unsigned char*) (0x08A0 + 0x05))
#define _TWI_RIF_bm ((unsigned char) 0b10000000)
#define _TWI_WIF_bm ((unsigned char) 0b01000000)
#define _TWI_RXACK_bm ((unsigned char) 0b00010000)
#define _TWI_ARBLOST_bm ((unsigned char) 0b00001000)
#define _TWI_BUSERR_bm ((unsigned char) 0b00000100)
#define _TWI_BUSSTATE_IDLE_gc ((unsigned char) 0b00000001)
#define _TWI0_MBAUD (*(volatile unsigned char*) (0x08A0 + 0x06))
#define _TWI0_MADDR (*(volatile unsigned char*) (0x08A0 + 0x07))
#define _TWI0_MDATA (*(volatile unsigned char*) (0x08A0 + 0x08))

#define TCA0_CTRLA    (*(volatile uint8_t *)(0x0A00))  // 타이머/카운터 제어 레지스터 A
#define TCA0_CTRLB    (*(volatile uint8_t *)(0x0A01))  // 타이머/카운터 제어 레지스터 B
#define TCA0_SINGLE_CTRLB (*(volatile uint8_t *)(0x0A04)) // 단일 모드 타이머/카운터 제어 레지스터 B
#define TCA0_SINGLE_PER   (*(volatile uint8_t *)(0x0A06)) // 단일 모드 타이머/카운터 기간 레지스터
#define TCA0_SINGLE_CMP0  (*(volatile uint8_t *)(0x0A08)) // 단일 모드 타이머/카운터 비교 레지스터 0

#define TCA_SINGLE_ENABLE_bm 0x01
#define TCA_SINGLE_CMP0EN_bm 0x10
#define TCA_SINGLE_WGMODE_SINGLESLOPE_gc 0x03
#define TCA_SINGLE_CLKSEL_DIV8_gc 0x02

#define IN1_PIN 1
#define IN2_PIN 2
#define ENA_PIN 3

#define _TWI_READ true
#define _TWI_WRITE false

#define MPU6050 0x68
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_CLOCK_PLL_XGYRO_bm 0b00000001

#define PI 3.141592
#define ALPHA 0.6
#define DT 0.2534

//void linear_reduce();
void _twi_init();
bool _twi_start(unsigned char device, bool read);
void _twi_stop();
bool _twi_read(unsigned char* data, bool last);
bool _twi_write(unsigned char data);
void mpu6050_init();
void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz);

short mpu6050_offsets[6];
int distance_below_threshold_count=0;

float angle_gx = 0, angle_x = 0;
int i=0;
int a=0;

void setup() {
  Serial1.begin(9600);

  //DC MOTOR 초기화 시작
  _PORTA_DIR |= ((1 << 2) | (1 << 3)| (1 << 4) | (1 << 5)| (1 << 6));
  _PORTF_DIR |= (1<<0);
  _PORTA_OUT &= ~((1 << 4) | (1 << 5)| (1 << 6));
  _PORTF_OUT &= ~(1<<0); 
  _PORTA_OUT |= (1 << 2)|(1 << 3);
  //DC MOTOR 초기화 끝

  //초음파 초기화
  _PORTA_DIR &= ~0b00000001; // PA0: INPUT ->에코=INPUT
  _PORTA_DIR |= 0b00000010; // PA1: OUTPUT ->트리거=출력
  PORTA_PIN0CTRL |= 0b00000001; // PA0: PULLUP
  _PORTA_OUT &= ~0b00000010; // PA1: LOW
  //초음파 초기화 END

  //리니어 초기화
  _PORTF_DIR |= 0b00001110;
  _PORTF_OUT &= ~(1 << ENA_PIN);
  //리니어 초기화 END

  TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
  TCA0_SINGLE_PER = 0xFF; // 주기 설정 (8-bit)
  TCA0_SINGLE_CMP0 = 0x80; // 듀티 사이클 설정 (50%)
  TCA0_CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm; // 타이머 시작

  TCB3_CTRLB = TCB_CNTMODE_INT_gc;  // Interval mode
  TCB3_CTRLA = TCB_ENABLE_bm;  // Enable TCB
  
 // _delay_ms(10);
  mpu6050_init();
}

void loop(){
  Serial1.println("loop");

  if(a==0){
    //초음파 SENSOR START
    _PORTA_OUT |= 0b00000010; // PA1: HIGH
    _delay_ms(10);
    _PORTA_OUT &= ~0b00000010; // PA1: LOW
    //초음파 SENSOR END

    while(!(PORTA_IN & 0b00000001));

    // 에코 핀이 HIGH인 동안의 시간을 측정
    TCB3_CNT = 0;  // 타이머 카운터를 0으로 초기화

    while(PORTA_IN & 0b00000001);

    // 초음파가 왕복하는데 걸린 시간을 계산
    // (타이머 카운트 값 / 클럭 주파수) * 초음파 속도(340.29 m/s) * 100 (cm로 변환) / 2 (왕복 거리이므로 2로 나눔)
    //카운트수x클럭주기x초음파속도=거리
    float distance = ((float)TCB3_CNT / F_CPU) * 340.29 * 100.0 / 2.0;
    
    Serial1.print("distace:");
    Serial1.print(distance);
    Serial1.print("\n");

    // 거리가 8cm 이하인 경우 카운트 증가
    if (distance < 8) {
      distance_below_threshold_count++;
      Serial1.print("count:");
      Serial1.print(distance_below_threshold_count);
      Serial1.print("\n");
    } 
    _delay_ms(10);
  }
    
  if(distance_below_threshold_count<10) return; //아직 카운트를 다 세지 않았으므로 계속 초음파로 측정하면 됨

  else if(distance_below_threshold_count>=10){
    a=1;
    Serial1.println("distance_below_threshold_count up to 10");
    //8이하일 때가, 10번 이상이므로 알고리즘 시작
    if(i==0){
      //리니어 감소된 상태로 초기화
      Serial1.println("liner initialization");
      _PORTF_OUT |= (1 << ENA_PIN);
      _PORTF_OUT |= (1 << IN2_PIN);
      _PORTF_OUT &= ~(1 << IN1_PIN);
      i=i+1;
    }
    _delay_ms(10000);

    Serial1.println("motor on");
    _PORTA_OUT |= (1 << 4); //dc motor 주행 시작
    _delay_ms(10);
    _PORTA_OUT |=(1<<6);
    _delay_ms(10);  

    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    // 가속도,자이로 센서의 x,y,z 축 데이터
    mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    // calculate angle by accel data
    float ax = raw_ax - mpu6050_offsets[0];
    float ay = raw_ay - mpu6050_offsets[1];
    float az = raw_az - mpu6050_offsets[2];

    float angle_ax = atan(ay / sqrt(ax * ax + az * az)) * (180 / PI);
    //y축을 기준으로 한 x축의 각도 

    float gz = ((float) (raw_gx - mpu6050_offsets[3])) / 131; //각속도
    // angle_gx += gz * DT;

    // complementary filter
    angle_x = ALPHA * (angle_x + gz * DT) + (1 - ALPHA) * angle_ax; // 최종 각도

    Serial1.print("angle:");
    Serial1.println(angle_x);
    _delay_ms(3000); //angle에 따라서 리니어 센서를 확인하기 위한 딜레이 나중에는 바꾸기

    if(angle_x<=-4){
      Serial1.println("linear reduce");
      _PORTF_OUT |= (1 << ENA_PIN); 
      _PORTF_OUT |= (1 << IN2_PIN);
      _PORTF_OUT &= ~(1 << IN1_PIN);
      //_delay_ms(2000);
    }
    else{
      Serial1.println("linear up");
      _PORTF_OUT |= (1 << ENA_PIN); //리니어 다시 늘어남
      _PORTF_OUT &= ~(1 << IN2_PIN);
      _PORTF_OUT |= (1 << IN1_PIN);
      //_delay_ms(2000);
    }
    _delay_ms(10);    
  }
}

// void linear_reduce() {
//   Serial1.println("linear_reduce");
//   //줄임
//   _PORTF_OUT |= (1 << ENA_PIN);
//   _PORTF_OUT |= (1 << IN2_PIN);
//   _PORTF_OUT &= ~(1 << IN1_PIN);
//   // ENA HIGH (PWM 활성화)
//   //TCA0_SINGLE_CTRLB |= TCA_SINGLE_CMP0EN_bm; ->안되면 이 코드 수정하기
//   Serial1.println("linear_reduce end");
// }

void _twi_init() {
  _PORTMUX_TWISPIROUTEA |= _PORTMUX_TWI0_ALT2_bm; // PC2: SDA, PC3: SCL
  _PORTC_PIN2CTRL |= _PORTC_PIN2CTRL_PULLUPEN_bm; // PC2: PULLUP
  _PORTC_PIN3CTRL |= _PORTC_PIN3CTRL_PULLUPEN_bm; // PC3: PULLUP

  unsigned int frequency = 400000; // 400kHz
  unsigned short t_rise = 300; // 300ns
  unsigned int baud = (F_CPU / frequency - F_CPU / 1000 / 1000 * t_rise / 1000 - 10) / 2;
  _TWI0_MBAUD = (unsigned char) baud;

  _TWI0_MCTRLA = _TWI_ENABLE_bm;
  _TWI0_MSTATUS = _TWI_BUSSTATE_IDLE_gc;
}

bool _twi_start(unsigned char device, bool read) {
  _TWI0_MADDR = device << 1 | (read ? 1 : 0);

  while (!(_TWI0_MSTATUS & (_TWI_WIF_bm | _TWI_RIF_bm)));

  if (_TWI0_MSTATUS & _TWI_ARBLOST_bm) {
    while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc));
    return false;
  }
  if (_TWI0_MSTATUS & _TWI_RXACK_bm) {
    _TWI0_MCTRLB |= _TWI_MCMD_STOP_gc;
    while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc));

    return false;
  }

  return true;
}

void _twi_stop() {
  _TWI0_MCTRLB |= _TWI_MCMD_STOP_gc;

  while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc));
}

bool _twi_read(unsigned char* data, bool last) {
  while (!(_TWI0_MSTATUS & _TWI_RIF_bm));

  *data = _TWI0_MDATA;
  if (last) {
    _TWI0_MCTRLB = _TWI_ACKACT_NACK_gc; // send NACK
  } 
  
  else {
    _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc; // send ACK
  }
  return true;
}

bool _twi_write(unsigned char data) {
  _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc;
  _TWI0_MDATA = data;

  while (!(_TWI0_MSTATUS & _TWI_WIF_bm));

  if (_TWI0_MSTATUS & (_TWI_ARBLOST_bm | _TWI_BUSERR_bm)) {
    return false; // ERROR
  }

  return !(_TWI0_MSTATUS & _TWI_RXACK_bm); // check ACK
}

void mpu6050_init() { //처음 시작할 때, offset계산
  _twi_init();

  _twi_start(MPU6050, _TWI_WRITE);
  _twi_write(MPU6050_PWR_MGMT_1); // reg address
  _twi_write(0); // reg value
  _twi_stop();

  // init sensor offset
  short sum[6] = { 0 };

  for (int i = 0; i < 10; i += 1) {
    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    sum[0] += raw_ax;
    sum[1] += raw_ay;
    sum[2] += raw_az;
    sum[3] += raw_gx;
    sum[4] += raw_gy;
    sum[5] += raw_gz;
  }

  for (int i = 0; i < 6; i += 1) {
    mpu6050_offsets[i] = sum[i] / 10;
  }
}

void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz) {
  unsigned char buf[14];
  
  for (int i = 0; i < 14; i += 1) {
    _twi_start(MPU6050, _TWI_WRITE);
    _twi_write(MPU6050_ACCEL_XOUT_H + i); // reg address
    _twi_stop();
    _twi_start(MPU6050, _TWI_READ);
    _twi_read(&buf[i], true); // reg value
    _twi_stop();
  }

  *raw_ax = (buf[0] << 8) | buf[1];
  *raw_ay = (buf[2] << 8) | buf[3];
  *raw_az = (buf[4] << 8) | buf[5];
  *raw_gx = (buf[8] << 8) | buf[9];
  *raw_gy = (buf[10] << 8) | buf[11];
  *raw_gz = (buf[12] << 8) | buf[13];
// 8비트를 16비트로 만듦 (두개를 합치는것)
}