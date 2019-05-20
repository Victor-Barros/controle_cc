const int canal_a=2, canal_b=3;
int i=0;
unsigned long tempo=0, ultimo_tempo=0;
volatile double angulo=0; 
double ultimo_angulo=0, velocidade[3]={0,0,0};

void setup() {
  pinMode(canal_a, INPUT);
  pinMode(canal_b, INPUT);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(canal_a), conta_pulsos_a, RISING);
}

void loop() {
  tempo=millis();
  
  if (tempo - ultimo_tempo >= 100) {
    velocidade[i++] = 9.5493*(angulo-ultimo_angulo)/((tempo-ultimo_tempo));
    Serial.println((velocidade[0]+velocidade[1]+velocidade[2])/3);
    ultimo_angulo=angulo;
    ultimo_tempo=tempo;
  }
  
  if (i >=3) i=0;
}

void conta_pulsos_a() {
  noInterrupts();
  if (digitalRead(canal_b) == 0) {
    angulo+=196.35; //krad
    //pulsos++;
  } else {
    angulo-=196.35;
    //pulsos--;
  }
  interrupts();
}
