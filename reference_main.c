#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define TRIGGER_PIN  PIN4_bm   //초음파 trig
#define ECHO_PIN     PIN6_bm   //초음파 echo
#define PA3_PIN 3   //스위치 1
#define PA5_PIN 5   //스위치 2
#define PD2_PIN 2   //모터 정
#define PD4_PIN 4   //모터 역
#define PB6_PIN 6   //부저
#define PB7_PIN 7   //led
#define MIN_HEIGHT_THRESHOLD 0
#define MAX_HEIGHT_THRESHOLD 50
#define HEIGHT_CHECK_PERIOD_MS 10

#define F_CPU 3333333
#define BAUD_RATE 9600

#define USART_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64/ (16 * (float)BAUD_RATE)) + 0.5)
#define PIN(N)  (1 << N)

#define _ADC0_CTRLA            (unsigned char *)(0x0600)
#define _ADC0_CTRLA            (unsigned char *)(0x0600)
#define _ADC0_MUXPOS         (unsigned char *)(0x0606)
#define _ADC0_COMMAND         (unsigned char *)(0x0608)
#define _ADC0_RES            (unsigned char *)(0x0610)
#define _ADC_RESSEL_10BIT_gc   (0x01<<0)
#define _ADC_ENABLE_bm         (0x01)
#define _ADC_STCONV_bm         (0x01)

#define PORTC               (unsigned char *)(0x0440)
#define PORTC_DIR            (unsigned char *)(0x0440)
#define PORTC_DIRSET         (unsigned char *)(0x0441)
#define PORTC_DIRCLR         (unsigned char *)(0x0442)
#define PORTC_OUT            (unsigned char *)(0x0444)
#define PORTC_OUTSET         (unsigned char *)(0x0445)
#define PORTC_OUTCLR         (unsigned char *)(0x0446)
#define PORTC_IN            (unsigned char *)(0x0448)

//volatile uint16_t height = 0;
volatile uint8_t captureStart = 0;
volatile uint16_t captureTime = 0;
int status = 0;
int height = 10;

void initialize();
float measureDistance();
void USART3_init();
int USART3_printChar(char, FILE *);
void USART3_Transmit(unsigned char);
int checkHeightCondition();
void moveforward();
void movereverse();
void stop();
void setup();
void TCA0_init();
void ADC_init();
uint16_t ADC_read(int);
uint8_t motor_speed(int);

void initialize() {
	// 초음파 센서의 트리거 핀을 출력으로 설정
	PORTB_DIR |= TRIGGER_PIN;

	// 초음파 센서의 에코 핀을 입력으로 설정
	PORTB_DIR &= ~ECHO_PIN;

	TCB1_CTRLB = TCB_CNTMODE_INT_gc;  // Interval mode
	TCB1_CTRLA = TCB_ENABLE_bm;  // Enable TCB
}

float measureDistance() {
	// 트리거 핀을 LOW로 설정하여 초기화
	PORTA_OUT &= ~TRIGGER_PIN;
	_delay_us(2);
	// 10us 동안 트리거 핀을 HIGH로 설정
	PORTA_OUT |= TRIGGER_PIN;
	_delay_us(10);
	PORTA_OUT &= ~TRIGGER_PIN;

	// 에코 핀이 HIGH가 될 때까지 기다림
	while(!(PORTA_IN & ECHO_PIN));

	// 에코 핀이 HIGH인 동안의 시간을 측정
	TCB1_CNT = 0;  // 타이머 카운터를 0으로 초기화
	while(PORTA_IN & ECHO_PIN);

	// 초음파가 왕복하는데 걸린 시간을 계산
	// (타이머 카운트 값 / 클럭 주파수) * 초음파 속도(340.29 m/s) * 100 (cm로 변환) / 2 (왕복 거리이므로 2로 나눔)
	float distance = ((float)TCB1_CNT / F_CPU) * 340.29 * 100.0 / 2.0;
	//float distance = (float)TCB0.CNT * 340.0 * 0.000001 / 2.0 * 100.0;
	
	return distance;
}

void USART3_init(void){
	PORTB_DIR |= PIN(0);
	PORTB_DIR &= ~PIN(1);
	// UART 통신 초기화
	USART3_BAUD = (uint16_t)USART_BAUD_RATE(BAUD_RATE);
	USART3_CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
}

void USART3_Transmit(unsigned char data){
	while (!((USART3_STATUS) & USART_DREIF_bm));
	USART3_TXDATAL = data;
	//while (!(USART0.STATUS & USART_TXCIF_bm));
}

int USART3_printChar(char c, FILE *stream){
	USART3_Transmit(c);
	return 0;
}

void moveforward(){
	PORTD_OUTSET = (1 << PD2_PIN);
	PORTD_OUTCLR = (1 << PD4_PIN);
	status = 1;
} //status 1

void movereverse(){
	PORTD_OUTSET = (1 << PD4_PIN);
	PORTD_OUTCLR = (1 << PD2_PIN);
	status = 2;
} //status 2

void stop(){
	PORTD_OUTCLR = (1 << PD4_PIN);
	PORTD_OUTCLR = (1 << PD2_PIN);
	status = 0;
} //status 0

ISR(PORTA_PORT_vect) {
	if (PORTA_INTFLAGS & (1 << PA3_PIN)) {
		// PA3 스위치가 눌렸을 때 (PA3에 GND 연결)
		if (checkHeightCondition() && status == 0) {
			// 특정 높이 값 조건을 만족할 경우, PD2에 5V 인가 (모터 정방향 회전)
			moveforward();
			status = 1;
		}
		else if(checkHeightCondition() && status != 0){
			stop();
			status = 0;
		}
		PORTA_INTFLAGS = (1 << PA3_PIN); // 인터럽트 플래그 클리어
	}

	if (PORTA_INTFLAGS & (1 << PA5_PIN)) {
		// PA5 스위치가 눌렸을 때 (PA5에 GND 연결)
		if (checkHeightCondition() && status == 0) {
			// 특정 높이 값 조건을 만족할 경우, PD4에 5V 인가 (모터 역방향 회전)
			movereverse();
			status = 2;
		}
		else if(checkHeightCondition() && status != 0){
			stop();
			status = 0;
		}
		PORTA_INTFLAGS = (1 << PA5_PIN); // 인터럽트 플래그 클리어
	}
}

void setup() {
	// PD2와 PD4를 출력으로 설정
	PORTD_DIRSET = (1 << PD2_PIN) | (1 << PD4_PIN);
	
	// PA3와 PA5를 입력으로 설정
	PORTA_DIRCLR = (1 << PA3_PIN) | (1 << PA5_PIN);
	
	// PA3와 PA5의 풀업 저항 활성화
	PORTA_PIN3CTRL = PORT_PULLUPEN_bm;
	PORTA_PIN5CTRL = PORT_PULLUPEN_bm;

	// 외부 인터럽트 설정
	PORTA_PIN3CTRL |= PORT_ISC_FALLING_gc; // PA3 스위치를 하강 에지로 인터럽트 설정
	PORTA_PIN5CTRL |= PORT_ISC_FALLING_gc; // PA5 스위치를 하강 에지로 인터럽트 설정

	// 전역 인터럽트 활성화
	sei();
}

int checkHeightCondition(uint16_t height) {
	// 특정 높이 값 조건 확인 함수
	// 적절한 높이 값 조건을 구현하고, 조건을 만족하면 1을 반환하고 그렇지 않으면 0을 반환합니다.
	// 이 예제에서는 임의로 HEIGHT_THRESHOLD 값 이상의 높이가 조건을 만족하는 것으로 가정합니다.
	
	// 여기에 높이 조건을 확인하는 코드를 작성합니다.
	// 만약 HEIGHT_THRESHOLD 이상의 높이가 조건을 만족한다면 1을 반환합니다.
	// 조건을 만족하지 않는다면 0을 반환합니다.
	height = 30;
	return ((height > MIN_HEIGHT_THRESHOLD) && (height < MAX_HEIGHT_THRESHOLD));
}

void TCA0_init(void) {
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CLKSEL_DIV1024_gc    // 클럭 소스를 system_clk/1024로 설정
	| TCA_SINGLE_WGMODE_NORMAL_gc;  // 카운터를 Normal mode로 설정
	TCA0.SINGLE.PER = HEIGHT_CHECK_PERIOD_MS * (F_CPU / 1024 / 1000) - 1;  // 주기 설정
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;  // 오버플로우 인터럽트 활성화
	TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;  // TCA0 타이머 활성화
}

ISR(TCA0_SINGLE_OVF_vect) {
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;  // 인터럽트 플래그 클리어
	if ((height > MAX_HEIGHT_THRESHOLD) || (height < MIN_HEIGHT_THRESHOLD)) {
		// 'height'가 50을 초과하면, 원하는 동작을 수행합니다.
		stop();
	}
}

void ADC_init(void){
	PORTD_PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;

	ADC0_CTRLA = ADC_RESSEL_10BIT_gc;
	ADC0_CTRLC |= ADC_REFSEL_VDDREF_gc;
	ADC0_CTRLA |= ADC_ENABLE_bm | ADC_FREERUN_bm;
}

uint16_t ADC_read(int ch){
	ADC0_MUXPOS = ch;
	ADC0_COMMAND |= ADC_STCONV_bm;
	
	while(!(ADC0_COMMAND) & ADC_STCONV_bm);
	
	return (uint16_t)ADC0_RES;
}

uint8_t motor_speed(int Y){	// Y 값에 따라 모터의 속도를 6단계로 제어
	if(Y >= 492 && Y <= 532){
		TCB0_CCMPH = 0;
	}
	else if((Y >= 533 && Y <= 631) || (Y >= 394 && Y <= 492)){
		TCB0_CCMPH = 32;
	}
	else if((Y >= 632 && Y <= 730) || (Y >= 295 && Y <= 393)){
		TCB0_CCMPH = 64;
	}
	else if((Y >= 731 && Y <= 829) || (Y >= 196 && Y <= 294)){
		TCB0_CCMPH = 128;
	}
	else if((Y >= 830 && Y <= 928) || (Y >= 98 && Y <= 195)){
		TCB0_CCMPH = 192;
	}
	else if((Y >= 929 && Y <= 1023) || (Y >= 0 && Y <= 97)){
		TCB0_CCMPH = 255;
	}
	return TCB0_CCMPH;
}

FILE USART_stream = FDEV_SETUP_STREAM(USART3_printChar, NULL, _FDEV_SETUP_WRITE);


int main(void) {
	setup();
	initialize();
	USART3_init();
	ADC_init();

	stdout = &USART_stream;
	PORTA_DIR |= PIN2_bm;
	PORTA_OUT |= PIN2_bm;

	TCB0_CTRLA |= TCB_ENABLE_bm // TCB enable
	| TCB_CLKSEL_CLKDIV2_gc; // (Clock 속도 / 2)

	TCB0_CTRLB |= TCB_CCMPEN_bm   // Compare mode enable
	| TCB_CNTMODE_PWM8_gc; // 8bit PWM Mode

	TCB0_CCMPL = 0xFF; // Period 설정
	TCB0_CCMPH = 0xFF; // Duty 설정

	*PORTC_DIRSET = PIN(2) | PIN(3) | PIN(4) | PIN(5); // PC2,3,4,5 핀을 출력핀으로 설정

	int X_value = ADC_read(0);
	_delay_ms(10);		// 약간의 딜레이를 주어 X축과 Y축 값의 인식이 겹치지 않도록 함.
	int Y_value = ADC_read(1);


	while (1) {
		//float distance = measureDistance();
		//printf("Distance: %.2f cm\n\r", distance);
		//height = distance;
		//float debugheight = checkHeightCondition(distance);
		//printf("height: %d\n\r",height);
		int X_value = ADC_read(0);
		_delay_ms(10);
		int Y_value = ADC_read(1);
		printf("X value : %d \t Y value : %d \n\r", X_value, Y_value);
		
		// 좌측 모터 순방향 : PIN5 , 좌측 모터 역방향 : PIN4
		// 우측 모터 순방향 : PIN2 , 우측 모터 역방향 : PIN3
		if(Y_value >= 492 && Y_value <= 532){
			if(X_value >= 467 && X_value <= 506){
				motor_speed(Y_value);
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))); 
				printf("stop\n\r");
			}
			else if(X_value >= 507 ){	// 우회전
				TCB0_CCMPH = 230;
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(3) | PIN(4); 
				printf("Right\n\r");
			}
			else if(X_value <= 466 ){	// 좌회전
				TCB0_CCMPH = 230;
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(2) | PIN(5);
				printf("Left\n\r");
			}
			_delay_ms(10);
		}
		else if(Y_value >= 533){	// 전진
			if(X_value >= 467 && X_value <= 506){
				motor_speed(Y_value);
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(2) | PIN(4);
				printf("drive\n\r");
			}
			else if(X_value >= 507 ){
				TCB0_CCMPH = 230;
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(3) | PIN(4);
				printf("Right\n\r");
			}
			else if(X_value <= 466 ){
				TCB0_CCMPH = 230;
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(2) | PIN(5);
				printf("Left\n\r");
				
			}
			_delay_ms(10);
		}
		else if(Y_value <= 491){	// 후진
			if(X_value >= 467 && X_value <= 506){
				motor_speed(Y_value);
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(3) | PIN(5);
				printf("reverse\n\r");
			}
			else if(X_value >= 507 ){
				TCB0_CCMPH = 230;
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(3) | PIN(4);
				printf("Right\n\r");
			}
			else if(X_value <= 466 ){
				TCB0_CCMPH = 230;
				*PORTC_OUT = (*PORTC_OUT & ~(PIN(2) | PIN(3) | PIN(4) | PIN(5))) | PIN(2) | PIN(5);
				printf("Left\n\r");
			}
			_delay_ms(10);
		}
		
		_delay_ms(300);
		

		//_delay_ms(1000);
	}
}