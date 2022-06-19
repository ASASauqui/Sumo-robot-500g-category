/*
 * Sumo robot.c
 *
 * Created: 10/04/2022 01:29:50 p. m.
 * Author : Alan Samuel Aguirre Salazar
 */ 


// Frecuencia CPU
	#define F_CPU 8000000
	
// Librerias
	#include <avr/io.h>
	#include <util/delay.h>								// Para poder hacer retardos
	#include <stdint.h>
	#include <stdlib.h>
	#include <avr/interrupt.h>
	#include <time.h>
	
// Definiciones
	#define  Trigger_pin	PD5							// Trigger pin
	#define startMotorTimer 50
	
// Esqueletos de funciones de los motores
	void adelante();
	void atras();
	void derecha();
	void izquierda();
	void lento();
	
// Esqueletos de funciones utiles
	uint8_t cero_en_bit(volatile uint8_t *LUGAR, uint8_t BIT);
	uint8_t uno_en_bit(volatile uint8_t *LUGAR, uint8_t BIT);
	void saca_uno(volatile uint8_t *LUGAR, uint8_t BIT);
	void saca_cero(volatile uint8_t *LUGAR, uint8_t BIT);
	
// Funciones a usar
	void printValues(uint8_t valor);
	void printValues16(int valor);
	void Start();
	void RutinaPreAtaque();
	double getDistance();
	void Atacar();
	void monitorearDistancia();
	void sensor1();
	void sensor2();
	
// Variables a usar
	volatile int TimerOverflow = 0;
	volatile double D = 0;
	



int main(void)
{
	// Configuración del puerto A para el botón de inicio
		DDRA = 0b00000000;
		PORTA = 0b11111111;
		
	// Configuración del puerto B para los motores
		DDRB = 0b11111111;
		PORTB = 0b00000000;
		
	// Puerto D para el sensor ultrasónico
		DDRD = 0b00100000;							// Hacer pin trigger como salida
		PORTD = 0b01001100;							// Activar Pull-ups
		
	Start();
	
}

void Start(){
	// Revisar si se apretó el botón de inicio
		while(uno_en_bit(&PINA,0));
	
	// Retardo
		_delay_ms(50);
		while(cero_en_bit(&PINA,0));
		_delay_ms(50);
		
	// Retardo de 5 segundos
		_delay_ms(1000);
		_delay_ms(1000);
		_delay_ms(1000);
		_delay_ms(1000);
		_delay_ms(1000);
		
		RutinaPreAtaque();
}

void RutinaPreAtaque(){
	sei();
	
	// Activar Timer1 con overflow
		TIMSK = (1 << TOIE1);						// Activar Timer1 overflow interrupciones
		TCCR1A = 0;
	
	// Activación de los motores con lentitud
		OCR0 = 50; 									// Velocidad
		TCCR0 = 0b01101001; 						// Config PWM
	
	do{
		derecha();
		monitorearDistancia();
		
		izquierda();
		monitorearDistancia();
	}
	while(1);
}

void monitorearDistancia(){
	for(int i=0; i<5;i++){
		D = getDistance();
		if(D < 65){
			Atacar();
		}

		if(cero_en_bit(&PIND,2)){
			sensor1();
		}
		if(cero_en_bit(&PIND,3)){
			sensor2();
		}
		
		_delay_ms(100);
	}
}

void sensor1(){
	OCR0 = 50;
	derecha();
	_delay_ms(700);
	
	PORTB = 0;
	RutinaPreAtaque();
}

void sensor2(){
	OCR0 = 50;
	izquierda();
	_delay_ms(700);
	
	PORTB = 0;
	RutinaPreAtaque();
}

void Atacar(){
	// Desactivar interrupción del Timer1
		TIMSK = 0;									// Activar Timer1 overflow interrupciones
		TCCR1A = 0;
	
	// Interrupciones externas para los sensores infrarrojos
		MCUCR = 0b00001010;
		GIFR = 0b11100000;
		GICR = 0b11000000;
	
	// Arranca al ataque
		OCR0 = 255;
		adelante();
	
	while(1);
}

ISR(TIMER1_OVF_vect)
{
	TimerOverflow++;								// Incrementar Timer Overflow count
}

ISR(INT0_vect){
	MCUCR = 0;
	GIFR = 0;
	GICR = 0;
	
	OCR0 = 50;
	derecha();
	_delay_ms(700);
	
	cli();
	
	RutinaPreAtaque();
}

ISR(INT1_vect){
	MCUCR = 0;
	GIFR = 0;
	GICR = 0;
	
	OCR0 = 50;
	izquierda();
	_delay_ms(700);
	
	cli();
	
	RutinaPreAtaque();
}


double getDistance(){
	volatile long count;
	volatile double distance;
	
	// Pulso en el trigger de 10us
		PORTD |= (1 << Trigger_pin);
		_delay_us(10);
		PORTD &= (~(1 << Trigger_pin));
		
		TCNT1 = 0;									// Limpiar el contador
		TCCR1B = 0x41;								// Captura en flanco ascendente. No prescaler
		TIFR = 1<<ICF1;								// Limpiar ICP flag
		TIFR = 1<<TOV1;								// Limpiar Timer Overflow flag

	// Calcular el ancho del echo por captura de entrada (ICP)
		while ((TIFR & (1 << ICF1)) == 0); 			// Esperar al flanco ascendente
		TCNT1 = 0;									// Limpiar Timer counter
		TCCR1B = 0x01;								// Captura en flanco descendente. No prescaler
		TIFR = 1<<ICF1;								// Limpiar ICP flag
		TIFR = 1<<TOV1;								// Limpiar Timer Overflow flag
		TimerOverflow = 0; 							// Limpiar Timer overflow count

	// Distancia
		while ((TIFR & (1 << ICF1)) == 0); 			// Esperar al flanco descendente
		count = ICR1 + (65535 * TimerOverflow);		// Tomar conteo
		// 8MHz Timer frecuencia, velocidad del sonido =343 m/s
		distance = (double)count / 466.47;
	
	return distance;
}




// Funciones de los motores
void adelante(){
	PORTB = 0b00100110;
}

void atras(){
	PORTB = 0b00100110;
}

void derecha(){
	PORTB = 0b00000110;
}

void izquierda(){
	PORTB = 0b00100100;
}

void lento(){
	OCR0 = 50;
	PORTB = 0b00100110;
}



// Funciones utiles
uint8_t cero_en_bit(volatile uint8_t *LUGAR, uint8_t BIT){
	return (!(*LUGAR&(1<<BIT)));
}

uint8_t uno_en_bit(volatile uint8_t *LUGAR, uint8_t BIT){
	return (*LUGAR&(1<<BIT));
}
void saca_uno(volatile uint8_t *LUGAR, uint8_t BIT){
	*LUGAR=*LUGAR|(1<<BIT);
}

void saca_cero(volatile uint8_t *LUGAR, uint8_t BIT){
	*LUGAR=*LUGAR&~(1<<BIT);
}
