#include <MsTimer2.h>

const int pwm1=10; //Pino pwm sentido normal
const int pwm2=9; //Pino pwm sentido reverso
const int a=2, b=3; //Canais a e b do encoder
static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
static uint8_t enc_val = 0;
double setpoint=0;
double x1[2]={0,0},x2[2]={0,0},ts=0.001,wc=150,u=0,y1[2]={0,0},e[2]={0,0},last_pwm; //Variaveis de discretizacao
double kp=0.01748, ki=0.8293;
volatile long enc_count = 0;
float soma_pot=0;
int n_pot=0;

void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000010; //for PWM 3921.16 Hz prescaler 8
  pinMode(pwm1, OUTPUT);
  pinMode(A1,INPUT); //Pot do setpoint
  pinMode(a,INPUT_PULLUP);
  pinMode(b,INPUT_PULLUP);
  Serial.begin(115200);
  MsTimer2::set(1000*ts, svf); //Interrupcao de controle
  MsTimer2::start();
  attachInterrupt(digitalPinToInterrupt(a),encoder_isr,CHANGE);
  attachInterrupt(digitalPinToInterrupt(b),encoder_isr,CHANGE);
  delayMicroseconds(2000);
}

void loop() {
  if(millis()%100==0) { //Prints
    Serial.print(String(x2[1])+",");
    Serial.print(String(y1[1]*50)+",");
    Serial.println(setpoint);
  }
  if(millis()%5==0) {
    soma_pot+=((float)analogRead(A1)/1023)*1200;
    n_pot++;
    if(n_pot >= 10) {
      setpoint=trunc(soma_pot/10);
      n_pot=0;
      soma_pot=0; 
    }
  }
}

void svf() {
  x1[0]=x1[1]; //Proximo passo
  x2[0]=x2[1];
  y1[0]=y1[1];
  e[0]=e[1];
  
  if (u>=2*PI) {
    noInterrupts();
    enc_count-=128;
    interrupts();
    x1[0]-=2*PI;
  }
  noInterrupts();
  u=(PI/64)*enc_count; //Atualizacao da velocidade (svf)
  interrupts();
  
  x1[1]=x2[0]*ts+x1[0];
  x2[1]=x2[0]*(1-2*wc*ts)-x1[0]*wc*wc*ts+u*wc*wc*ts;

  e[1]=setpoint-x2[1];
  y1[1]=y1[0]+(kp)*(e[1]-e[0])+(ki)*e[0]*ts; //Atualizacao da acao de controle
  
  if (y1[1]>12) y1[1]=12; //Evita o Windup do fator integral
  if (y1[1]<-12) y1[1]=-12;
  if (setpoint==0) y1[1]=0;
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

void encoder_isr() {
  enc_val = enc_val << 2;
  enc_val = enc_val | ((PIND & 0b1100) >> 2);
  enc_count += lookup_table[enc_val & 0b1111];
}
