#include <avr/io.h>         //Incluindo biblioteca
#include <util/delay.h>       //Incluindo biblioteca
#include <stdbool.h>

#define S1 PD2    // pino digital 2     1 - em cima da linha, 10 - fora da linha
#define S2 PD3    // pino digital 3
#define S3 PD4    // pino digital 4

#define motor1 PB1  // pino digital 9 
#define motor2 PB2  // pino digital 10

#define IN1 PB3  // pino digital 11, define sentido para motor girar
#define IN2 PB4  // pino digital 12, define sentido para motor girar

#define kp 150
#define kd 100
#define ki 0
#define velIni 600  // velocidade inicial para ligar motor

bool LS1,LS2,LS3; // leitura dos sensores
int sinalControle;  // valor do pid calculado
double erro,erroAnti; // variaveis para armazenar valor dos erros
int proporcional,derivativo,integrador;

void iniciaPWM(){ // configurão para utilizar o pwm dos pinos digitais 9 e 10
  TCCR1A = ((1 << COM0A1) | (1 << COM0B1) | (1 << WGM11) | (1 << WGM10));
  TCCR1B = ((1 << CS11) | (1 << CS10)); 
}
void sentidoMotor(){
  // aciona pinos digitais 11 e 12 para definir o sentido dos motores
  PORTB = PORTB | (1<<IN1) & (~(1<<IN2)); 
}
void leituraSensores(){
    if(PIND & (1<<S1)){ // verifica se sensor foi acionado
      LS1 = 1;  // se for acionado seta a variavel como 1
    } 
    else{     // se não seta como 0
      LS1 = 0;
    } 
    if(PIND & (1<<S2)){
      LS2 = 1;
    } 
    else{
      LS2 = 0;
    } 
    if(PIND & (1<<S3)){
      LS3 = 1;
    } 
    else{
      LS3 = 0;
    } 
}
void calculaErro(){
  if(!LS1 && !LS2 && LS3){
    erro = 2;
  }
  if(!LS1 && LS2 && LS3){
    erro = 1;
  }
  if(!LS1 && LS2 && !LS3){
    erro = 0;
  }
  if(LS1 && LS2 && !LS3){
    erro = -1;
  }
  if(LS1 && !LS2 && !LS3){
    erro = -2;
  }
}

void PID(){ //função controle PID
  proporcional = erro;
  derivativo = erro - erroAnti;
  integrador = integrador + erro;
  
  sinalControle = kp*proporcional + kd*derivativo + ki*integrador;

  OCR1A = velIni+sinalControle;     // PWM pino 9
  OCR1B = velIni-sinalControle;     // PWM pino 10

  erroAnti = erro;
}

int main(){
  iniciaPWM();// inicia as configurações do pwm nos pinos D9 e D10

  DDRD |= (0<<S1) | (0<<S2) | (0<<S3);  // sensores configurados como entrada
  DDRB |= (1<<motor1) | (1<<motor2) | (1<<IN1) | (1<<IN2);

  sentidoMotor(); // chama função para setar sentido nos motores

  erroAnti = 0;
  integrador = 0;

  while(1){
    leituraSensores();
    calculaErro();
    PID();
    _delay_ms(10);
  }
}
