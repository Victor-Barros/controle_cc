#include <MsTimer2.h>
#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS 1

const int pwm1=10; //Pino pwm sentido normal
const int pwm2=9; //Pino pwm sentido reverso
float setpoint=700;
volatile float x1[2]={0,0},x2[2]={0,0},ts=0.001,wc=150,u=0,y1[2]={0,0},e[2]={0,0},last_pwm; //Variaveis de discretizacao
float kp=2.38/100, ki=535.35/1000; //Ganhos do controlador
Encoder myEnc(2, 3); //Objeto leitura do encoder

void setup() {
  pinMode(pwm1, OUTPUT);
  pinMode(A1,INPUT); //Pot do setpoint
  Serial.begin(9600);
  MsTimer2::set(1000*ts, svf); //Interrupcao de controle
  MsTimer2::start();
  
}

void loop() {
  if(millis()%100==0) { //Prints
    Serial.print(String(x2[1])+",");
    Serial.print(String(y1[1]*50)+",");
    Serial.println(setpoint);
    setpoint=(((float)analogRead(A1)/1023)-0.5)*2400; //Leitura do setpoint
  }
}

void svf() {
  x1[0]=x1[1]; //Proximo passo
  x2[0]=x2[1];
  y1[0]=y1[1];
  e[0]=e[1];
  
  u=(PI/64)*myEnc.read(); //Atualizacao da velocidade (svf)
  x1[1]=x2[0]*ts+x1[0];
  x2[1]=x2[0]*(1-2*wc*ts)-x1[0]*wc*wc*ts+u*wc*wc*ts;
  
  e[1]=setpoint-x2[1];
  y1[1]=y1[0]+(kp)*(e[1]-e[0])+(ki)*e[0]*ts; //Atualizacao da acao de controle
  
  if (y1[1]>12) y1[1]=12; //Evita o Windup do fator integral
  if (y1[1]<-12) y1[1]=-12;
  if (y1[1] != last_pwm) writePWM(y1[1]);
}

void writePWM(float v) { //Acionamento do motor
  if (v >= 0) {
    analogWrite(pwm1,(v/12)*255);
    analogWrite(pwm2,0);  
  } else {
    analogWrite(pwm2,(-v/12)*255);
    analogWrite(pwm1,0);
  }
  last_pwm=v;
}

/*void svf() {
  noInterrupts();
  x1[0]=x1[1];
  x2[0]=x2[1];
  if (u >= 2*PI) {
    pulsos-=32;
    x1[0]-= 2*PI;
  }
  u=(PI/16)*pulsos;
  x1[1]=x2[0]*ts+x1[0];
  x2[1]=x2[0]*(1-2*wc*ts)-x1[0]*wc*wc*ts+u*wc*wc*ts;
  erro=setpoint-x2[1];
  setval=erro*k;
  if (setval>12) setval=12;
  if (setval<-12) setval=-12;
  writePWM(setval);
  interrupts();
}*/
