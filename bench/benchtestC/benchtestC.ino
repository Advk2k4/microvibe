const int TILT=4; volatile long edges=0; int last=HIGH;
void setup(){ Serial.begin(115200); pinMode(TILT, INPUT_PULLUP); }
void loop(){
  static unsigned long t0=millis(); static long count=0;
  int v=digitalRead(TILT);
  if(v!=last){ count++; last=v; }
  if(millis()-t0>=1000){ Serial.print("edges/s:"); Serial.println(count); count=0; t0=millis(); }
}