#include <MsTimer2.h>

const int canal_a=2, canal_b=3, pwm1=6;
int i=0;
volatile long pulsos;
volatile double x1[2]={0,0},x2[2]={0,0},ts=0.002,wc=50,u=0,soma=0;
volatile int cont_soma=0;
int serial_mode=0; //0-serial print; 1-serial data
String str;

void setup() {
  pinMode(canal_a, INPUT);
  pinMode(canal_b, INPUT);
  Serial.begin(115200);
  MsTimer2::set(2, svf); //2ms
  MsTimer2::start();
  attachInterrupt(digitalPinToInterrupt(canal_a), conta_pulsos_a, RISING);
}

void loop() {

  if(Serial.available() > 0) {
    str=Serial.readString();
    if (str == "c\n") serial_mode=1;
    else if (str == "p\n") serial_mode=2;
    else serial_mode=0;
  }
  
  if(millis()%100==0 && serial_mode == 2) {
    Serial.println((x2[1]*60)/(2*PI));
  }
}

void capturar() {
  
}

void svf() {
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
  if (serial_mode == 1) {
    //Serial.println(x2[1]);
    soma+=x2[1];
    cont_soma++;
    if(cont_soma >= 5) {
      //Serial.println(soma/5);
      soma=0;
      cont_soma=0;
    }
  }
  interrupts();
}

void conta_pulsos_a() {
  noInterrupts();
  if (digitalRead(canal_b) == 0) pulsos++;
  else pulsos--; 
  interrupts();
}
