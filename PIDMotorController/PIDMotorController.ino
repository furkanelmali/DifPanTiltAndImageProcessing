#include <util/atomic.h>

#define ENCA 2
#define ENCB 6
#define PWM 9
#define IN2 4
#define IN1 5
#define ENCA_1 3
#define ENCB_1 7
#define PWM_1 10
#define IN2_1 11
#define IN1_1 12
#define limitSwitchPinback 8
#define limitSwitchPinfront 13

volatile int posi1 = 0;
volatile int posi2 = 0;
float eprev1 = 0, eintegral1 = 0;
float eprev2 = 0, eintegral2 = 0;

unsigned long sonVeriZamani = 0;
const unsigned long veriTimeout = 200; 

volatile long pulseCount1 = 0;
volatile long pulseCount2 = 0;

unsigned long lastHizUpdate = 0;
const unsigned long hizGuncellemeAraligi = 100; // ms

long lastPulse1 = 0;
long lastPulse2 = 0;

float motorHiz1 = 0;
float motorHiz2 = 0;


int smoothedPWM1 = 0;

void readEncoder1();
void readEncoder2();
void PID(int target, int in1, int in2, int pwm, int pos, float &eprev, float &eintegral);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder1, RISING);
  
  pinMode(ENCA_1, INPUT);
  pinMode(ENCB_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_1), readEncoder2, RISING);
  
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  
  pinMode(limitSwitchPinback, INPUT_PULLUP);
  pinMode(limitSwitchPinfront, INPUT_PULLUP);

}


bool direction1 = true; // true = ileri, false = geri
bool direction2 = true;

unsigned long lastSwitchTime = 0;
const unsigned long interval = 500; // 2 saniyede bir yön değiştir



String gelenVeri = "";
float speedX = 0.0;
float speedY = 0.0;

void loop() {
   if (digitalRead(limitSwitchPinback) == LOW) {
      
    setMotor(0, 0, PWM, IN1, IN2);
    setMotor(-1,100, PWM_1, IN1_1, IN2_1);
    return;
    }
   else if (digitalRead(limitSwitchPinfront) == LOW) {
      
    setMotor(0, 0, PWM, IN1, IN2);
    setMotor(1,100, PWM_1, IN1_1, IN2_1);
    return;
    }
   
    while (Serial.available()) {

    
    
    char c = Serial.read();
    if (c == '\n') {
      parseVeri(gelenVeri);
      gelenVeri = "";
      sonVeriZamani = millis(); 
    } else {
      gelenVeri += c;
    }
  }
  if (millis() - sonVeriZamani > veriTimeout) {
  setMotor(0, 0, PWM, IN1, IN2);
  setMotor(0,0, PWM_1, IN1_1, IN2_1);
  }
  if (millis() - lastHizUpdate >= hizGuncellemeAraligi) {
  noInterrupts(); // atomik erişim
  long currentPulse1 = pulseCount1;
  long currentPulse2 = pulseCount2;
  interrupts();

  long fark1 = currentPulse1 - lastPulse1;
  long fark2 = currentPulse2 - lastPulse2;

  // pulse per second
  motorHiz1 = (float)fark1 / (hizGuncellemeAraligi / 1000.0); // pulse/s
  motorHiz2 = (float)fark2 / (hizGuncellemeAraligi / 1000.0);

  lastPulse1 = currentPulse1;
  lastPulse2 = currentPulse2;
  lastHizUpdate = millis();
  }
  if (millis() - sonVeriZamani <= veriTimeout) {

      if (abs(speedX) < 30 && abs(speedY) < 30) {
      setMotor(0, 0, PWM, IN1, IN2);
      setMotor(0, 0, PWM_1, IN1_1, IN2_1);
      return;
    }

    int cDegeri = yonBelirle(speedX, -speedY);
    int hedefPWM = hesaplaPWM(speedX, speedY, 100); 

    if(yonBelirle(speedX, -speedY) == 1 || yonBelirle(speedX, -speedY) == 2 || yonBelirle(speedX, -speedY) == 5 || yonBelirle(speedX, -speedY) == 8)
    {
      PID(hedefPWM, motorHiz1, eprev1, eintegral1, smoothedPWM1);
    }
   else if(yonBelirle(speedX, -speedY) == 3 || yonBelirle(speedX, -speedY) == 4 || yonBelirle(speedX, -speedY) == 6 || yonBelirle(speedX, -speedY) == 7)
   {
    PID(hedefPWM, motorHiz2, eprev1, eintegral1, smoothedPWM1);
   }

    setDirection(cDegeri, smoothedPWM1);

    
    
  } else {
    setMotor(0, 0, PWM, IN1, IN2);
    setMotor(0, 0, PWM_1, IN1_1, IN2_1);
  }

    
  
 
  
}


int hesaplaPWM(float speedX, float speedY, float maxHiz) {
  float hiz = min(abs(speedX), abs(speedY)); 
  hiz = constrain(hiz, 0, maxHiz);           

  float oran = 0.3;              
  int pwm = (int)70+(hiz * oran);            
  if(pwm >90) pwm=90;
  if (pwm < 10) pwm = 0; 

  return pwm;
}

void setDirection(int c, int sp)
{
  int speed1 = 0;
  int speed2 = 0;
  int dir1 = 0;
  int dir2 = 0;
  switch (c) {
    case 1: // sola dön
      speed1 = sp;
      speed2 = sp - 20 ;
      dir1 = 1; // saat yönü
      dir2 = 1;
      break;

    case 2: // sağa dön
      speed1 = sp;
      speed2 = sp - 20;
      dir1 = -1; // saat yönü tersi
      dir2 = -1;
      break;

    case 3: // stand aşağı
      speed1 = 0;
      speed2 = sp+10;
      dir2 = -1;
      break;

    case 4: // stand yukarı
      speed1 = 0;
      speed2 = sp;
      dir2 = 1;
      break;

    case 5: // sağa dön + stand aşağı
      speed1 = sp;
      speed2 = sp + 20;
      dir1 = -1;
      dir2 = -1;
      break;

    case 6: // sağa dön + stand yukarı
      speed1 = sp + 20;
      speed2 = sp;
      dir1 = -1;
      dir2 = -1;
      break;

    case 7: // sola dön + stand aşağı
      speed1 = sp + 20;
      speed2 = sp;
      dir1 = 1;
      dir2 = 1;
      break;

    case 8: // sola dön + stand yukarı
      speed1 = sp;
      speed2 = sp+20;
      dir1 = 1;
      dir2 = 1;
      break;
  }
  
  setMotor(dir1, speed1, PWM, IN1, IN2);
  setMotor(dir2, speed2, PWM_1, IN1_1, IN2_1);

}

void parseVeri(String veri) {
  int xIndex = veri.indexOf("X:");
  int yIndex = veri.indexOf("Y:");

  if (xIndex != -1 && yIndex != -1) {
    String xDeger = veri.substring(xIndex + 2, yIndex - 1);
    String yDeger = veri.substring(yIndex + 2);

    speedX = xDeger.toFloat();
    speedY = yDeger.toFloat();
  }
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
 

  pwmVal = constrain(pwmVal, 0, 255);

  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(pwm,pwmVal);
}


int yonBelirle(float x, float y) {
  float angle = atan2(y, x) * 180 / PI;

  
  if (angle >= -22.5 && angle < 22.5) return 2;        // sağ
  if (angle >= 22.5 && angle < 67.5) return 6;         // sağ + yukarı
  if (angle >= 67.5 && angle < 112.5) return 4;        // yukarı
  if (angle >= 112.5 && angle < 157.5) return 8;       // sol + yukarı
  if (angle >= 157.5 || angle < -157.5) return 1;      // sol
  if (angle >= -157.5 && angle < -112.5) return 7;     // sol + aşağı
  if (angle >= -112.5 && angle < -67.5) return 3;      // aşağı
  if (angle >= -67.5 && angle < -22.5) return 5;       // sağ + aşağı

  return 1;
}

void PID(int target, int current, float &eprev, float &eintegral, int &smoothedPWM) {
  int targetHiz = target * 7;
  float kp = 6, kd = 1.5, ki = 0.5;
  int e = targetHiz - current;
  float dedt = e - eprev;
  eintegral += e;
  eintegral = constrain(eintegral, -100, 100);
  float u = kp * e + kd * dedt + ki * eintegral;
  smoothedPWM = constrain((int)abs(u),50, 255) / 7;
  if(smoothedPWM < 70) smoothedPWM = 70;
  eprev = e;
}
void readEncoder1() {
  posi1 += (digitalRead(ENCB) > 0) ? 1 : -1;
}

void readEncoder2() {
  posi2 += (digitalRead(ENCB_1) > 0) ? 1: -1;
}
