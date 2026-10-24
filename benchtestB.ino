const int PIEZO=1;
void setup(){ Serial.begin(115200); }
void loop(){
  long sum=0; int mn=4095, mx=0;
  unsigned long t0=millis();
  while(millis()-t0 < 100){                    // 100 ms window
    int v=analogRead(PIEZO);
    if(v<mn) mn=v; if(v>mx) mx=v; sum+=abs(v-0);
  }
  int p2p = mx-mn;                              // peak-to-peak counts
  Serial.print("p2p:"); Serial.println(p2p);    // view in Serial Plotter
  delay(20);
}