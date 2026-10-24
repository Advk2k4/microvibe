// L293D / motor wiring diagnostic. Same pins as benchtestA. Serial 115200, send one character:
//   1  EN1 static HIGH (no PWM), IN1 HIGH, IN2 LOW   -> forward, full on   (default at boot)
//   2  EN1 static HIGH, IN1 LOW, IN2 HIGH            -> reverse, full on
//   0  everything LOW                                -> motor off
//   A  PWM duty 255 at 20 kHz on EN1, forward        (benchtestA settings)
//   B  PWM duty 255 at 5 kHz on EN1, forward
//   L  slow blink: EN1, IN1, IN2 all toggle together every 1 s (for LED-probe checks)
// Every second it prints the commanded levels and what the pads actually read back.
// A pad commanded HIGH that reads 0 is shorted to GND or loaded down.
const int ENA=5, IN1=6, IN2=7;
char mode='1';
bool pwmOn=false;
int cmd1=0, cmd2=0;               // commanded IN1/IN2 levels

void enStatic(int level){
  if(pwmOn){ ledcDetach(ENA); pwmOn=false; }
  pinMode(ENA,OUTPUT); digitalWrite(ENA,level);
}
void enPwm(uint32_t freq){
  if(pwmOn) ledcDetach(ENA);
  ledcAttach(ENA, freq, 8); ledcWrite(ENA,255); pwmOn=true;
}
void dir(int a,int b){ cmd1=a; cmd2=b; digitalWrite(IN1,a); digitalWrite(IN2,b); }

void apply(char m){
  mode=m;
  switch(m){
    case '1': enStatic(HIGH); dir(HIGH,LOW); Serial.println("# mode 1: EN static HIGH, forward"); break;
    case '2': enStatic(HIGH); dir(LOW,HIGH); Serial.println("# mode 2: EN static HIGH, reverse"); break;
    case '0': enStatic(LOW);  dir(LOW,LOW);  Serial.println("# mode 0: all LOW, motor off"); break;
    case 'A': enPwm(20000);   dir(HIGH,LOW); Serial.println("# mode A: PWM 255 @ 20 kHz, forward"); break;
    case 'B': enPwm(5000);    dir(HIGH,LOW); Serial.println("# mode B: PWM 255 @ 5 kHz, forward"); break;
    case 'L': enStatic(LOW);  dir(LOW,LOW);  Serial.println("# mode L: EN/IN1/IN2 blink together, 1 s"); break;
  }
}

void setup(){
  Serial.begin(115200);
  delay(1500);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  Serial.println("# diag_l293d: send 1 2 0 A B L");
  apply('1');
}

void loop(){
  while(Serial.available()){
    char c=toupper(Serial.read());
    if(strchr("120ABL",c)) apply(c);
  }
  static unsigned long t0=0; static bool blink=false;
  if(millis()-t0>=1000){
    t0=millis();
    if(mode=='L'){ blink=!blink; digitalWrite(ENA,blink); dir(blink,blink); }
    // digitalRead on an output pin returns the pad level on ESP32
    Serial.printf("mode %c | EN1(G5)=%s IN1(G6) cmd %d read %d | IN2(G7) cmd %d read %d\n", mode,
      pwmOn? "PWM" : (digitalRead(ENA)? "1":"0"),
      cmd1, digitalRead(IN1), cmd2, digitalRead(IN2));
  }
}
