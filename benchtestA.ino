const int ENA=5, IN1=6, IN2=7;
void setup(){
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  ledcAttach(ENA, 20000, 8);          // 20 kHz PWM, 8-bit
}
void loop(){
  for(int d=0; d<=255; d+=15){ ledcWrite(ENA,d); delay(400); }
  for(int d=255; d>=0; d-=15){ ledcWrite(ENA,d); delay(400); }
}