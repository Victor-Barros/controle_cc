#include <MsTimer2.h>
const int canal_a=2, canal_b=3;
int i=0;
volatile long pulsos;
volatile double x1[2]={0,0},x2[2]={0,0},ts=0.002,wc=50,u=0;
void setup() {
  pinMode(canal_a, INPUT);
  pinMode(canal_b, INPUT);
  Serial.begin(115200);
  MsTimer2::set(2, svf); //2ms
  MsTimer2::start();
  attachInterrupt(digitalPinToInterrupt(canal_a), conta_pulsos_a, RISING);
}

void loop() {
  if(millis()%100==0) {
    Serial.println((x1[1]*60)/(2*3.14159));
  }
}

void svf() {
  noInterrupts();
  u=0.19635*pulsos;
  x1[0]=x1[1];
  x2[0]=x2[1];
  
  x1[1]=x2[0]*ts+x1[0];
  x2[1]=x2[0]*(1-2*wc*ts)-x1[0]*wc*wc*ts+u*wc*wc*ts;
  interrupts();
}

void conta_pulsos_a() {
  noInterrupts();
  if (digitalRead(canal_b) == 0) pulsos++;
  else pulsos--; 
  interrupts();
}
