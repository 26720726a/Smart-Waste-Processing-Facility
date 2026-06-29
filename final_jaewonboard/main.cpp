#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>

// === 공통 매크로 ===
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define PRESSURE_SENSOR_CHANNEL 5
#define IR_SENSOR_CHANNEL 7

#define LED_DDR  DDRA
#define LED_PORT PORTA

#define VIB_MOTOR_DDR  DDRE
#define VIB_MOTOR_PIN  PE5

#define TRIG_PORT PORTC
#define TRIG_DDR  DDRC
#define TRIG_PIN  PC0

#define ECHO_PORT PINC
#define ECHO_DDR  DDRC
#define ECHO_PIN  PC1

#define SERVO_DDR  DDRB
#define SERVO_PIN  PB5

#define BUZZER_DDR  DDRB
#define BUZZER_PORT PORTB
#define BUZZER_PIN  PB4

#define MAF_SIZE 5
#define FIR_ORDER 5

// === 전역 변수 ===
uint16_t maf_buffer[MAF_SIZE] = {0};
uint8_t maf_index = 0, maf_filled = 0;
int fir_buffer[FIR_ORDER] = {0};
uint8_t fir_index = 0;

// === UART ===
void uart_init(unsigned int ubrr) {
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void uart_transmit(char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}
void uart_print(const char* str) {
	while (*str) uart_transmit(*str++);
}

// === ADC ===
void adc_init() {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
uint16_t read_adc(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

// === Timer0 (부저) ===
void timer0_pwm_init() {
	BUZZER_DDR |= (1 << BUZZER_PIN);
	TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01);
	TCCR0 |= (1 << CS01) | (1 << CS00);
	OCR0 = 128;
}
void pwm_stop() {
	TCCR0 &= ~((1 << CS01) | (1 << CS00) | (1 << COM01) | (1 << COM00));
	BUZZER_PORT &= ~(1 << BUZZER_PIN);
}
void pwm_start() {
	TCCR0 |= (1 << COM01);
	TCCR0 |= (1 << CS01) | (1 << CS00);
}

// === Timer1 (서보) ===
void timer1_init() {
	TCCR1A |= (1 << COM1A1) | (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	ICR1 = 39999;
	SERVO_DDR |= (1 << SERVO_PIN);
}
void set_servo_angle(uint8_t angle) {
	if (angle > 180) angle = 180;
	uint16_t pulse = 100 + ((uint32_t)angle * 3000) / 180;
	OCR1A = pulse * 2;
}

// === Timer3 (진동모터) ===
void timer3_pwm_init() {
	VIB_MOTOR_DDR |= (1 << VIB_MOTOR_PIN);
	TCCR3A |= (1 << COM3C1) | (1 << WGM30);
	TCCR3B |= (1 << WGM32) | (1 << CS31);
	OCR3C = 0;
}

// === 압력 LED 표시 ===
void led_display(uint8_t level) {
	LED_PORT = 0xFF >> level;
}

// === 초음파 ===
void ultrasonic_trigger() {
	TRIG_PORT &= ~(1 << TRIG_PIN);
	_delay_us(2);
	TRIG_PORT |= (1 << TRIG_PIN);
	_delay_us(10);
	TRIG_PORT &= ~(1 << TRIG_PIN);
}
uint16_t read_distance() {
	uint32_t timeout = 30000;
	while (!(ECHO_PORT & (1 << ECHO_PIN)) && timeout--) _delay_us(1);
	uint16_t count = 0;
	while ((ECHO_PORT & (1 << ECHO_PIN)) && count < 30000) {
		_delay_us(1);
		count++;
	}
	return count / 58;
}
uint16_t MAF_filter() {
	ultrasonic_trigger();
	uint16_t new_sample = read_distance();
	if (new_sample > 50) new_sample = 50;  // 필터 범위 확장
	if (new_sample < 5) new_sample = 5;    // 필터 범위 확장
	maf_buffer[maf_index] = new_sample;
	maf_index = (maf_index + 1) % MAF_SIZE;
	if (maf_filled < MAF_SIZE) maf_filled++;
	uint32_t sum = 0;
	for (uint8_t i = 0; i < maf_filled; i++) sum += maf_buffer[i];
	return sum / maf_filled;
}
uint8_t map_distance_to_angle(uint16_t distance) {
	if (distance >= 50) return 0;
	if (distance <= 5) return 180;
	return (50 - distance) * 3;  // 범위 50cm로 확대
}

// === 적외선 FIR 필터 ===
int fir_filter(int new_sample) {
	fir_buffer[fir_index] = new_sample;
	fir_index = (fir_index + 1) % FIR_ORDER;
	int sum = 0;
	for (uint8_t i = 0; i < FIR_ORDER; i++) sum += fir_buffer[i];
	return sum / FIR_ORDER;
}

// === 거리 → ms 변환 ===
uint16_t distance_to_delay(int distance) {
	if (distance < 10) distance = 10;
	if (distance > 50) distance = 50;  // 거리 범위 확장
	return 100 + (distance - 10) * (900) / (50 - 10);  // 최대 900ms까지 딜레이
}

// === 적외선 센서 ADC → 거리 ===
int adc_to_cm_int(uint16_t adc_value) {
	if (adc_value == 0) return 30;
	float distance = 12343.85 * pow((float)adc_value, -1.15);
	if (distance < 10) distance = 10;
	if (distance > 30) distance = 30;
	return (int)(distance + 0.5);
}

// === 메인 ===
int main(void) {
	char buffer[64];
	LED_DDR = 0xFF;
	LED_PORT = 0x00;
	TRIG_DDR |= (1 << TRIG_PIN);
	ECHO_DDR &= ~(1 << ECHO_PIN);

	uart_init(MYUBRR);
	adc_init();
	timer0_pwm_init();
	timer1_init();
	timer3_pwm_init();

	_delay_ms(1000);

	while (1) {
		// 압력센서
		uint16_t adc_val = read_adc(PRESSURE_SENSOR_CHANNEL);
		if (adc_val > 750) adc_val = 750;
		uint8_t led_count = (adc_val * 8) / 1024;
		led_display(led_count);
		uint8_t vib_duty = 0;
		if (adc_val >= 50) {
			if (adc_val < 190) vib_duty = 20;
			else if (adc_val < 330) vib_duty = 40;
			else if (adc_val < 470) vib_duty = 60;
			else if (adc_val < 610) vib_duty = 80;
			else vib_duty = 100;
		}
		OCR3C = (vib_duty * 255) / 100;

		// 초음파 거리 → 서보
		uint16_t curr_dist = MAF_filter();
		uint8_t angle = map_distance_to_angle(curr_dist);
		set_servo_angle(angle);

		// 적외선 거리 측정
		uint16_t ir_adc = read_adc(IR_SENSOR_CHANNEL);
		int ir_dist = adc_to_cm_int(ir_adc);
		int filtered_dist = fir_filter(ir_dist);
		snprintf(buffer, sizeof(buffer), "Raw: %d cm, Filtered: %d cm\r\n", ir_dist, filtered_dist);
		uart_print(buffer);

		uint16_t delay_time = distance_to_delay(filtered_dist);
		pwm_start();
		for (uint16_t i = 0; i < delay_time; i++) _delay_ms(1);
		pwm_stop();
		for (uint16_t i = 0; i < delay_time; i++) _delay_ms(1);
	}
}