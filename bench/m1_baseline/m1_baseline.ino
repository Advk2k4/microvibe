// Milestone 1: benchtestA ramp + benchtestB piezo p2p + benchtestC tilt edges, in one sketch.
// Boots into one A-ramp (up/down), then holds duty 180 and prints one line per second.
// Serial 115200. Commands (send a line):
//   H / F   start labelling seconds as HEALTHY / FAULT (stats reset for that label)
//   P       print summary (mean/std of p2p and edges/s per label)
//   R       re-run the A ramp, then return to hold duty
//   S       stop motor (duty 0)      G   go (back to hold duty)
//   D<n>    set hold duty 0..255, e.g. D150
#include "esp_system.h"

const int ENA=5, IN1=6, IN2=7;     // from benchtestA
const int PIEZO=1;                 // from benchtestB
const int TILT=4;                  // from benchtestC

int holdDuty=180, duty=0;
char label='-';                    // '-' unlabelled, 'H', 'F'

struct Stat { long n=0; double s=0, ss=0; void add(double v){ n++; s+=v; ss+=v*v; }
  double mean() const { return n? s/n : 0; }
  double sd() const { return n>1? sqrt((ss-s*s/n)/(n-1)) : 0; } };
Stat p2pH, p2pF, edgH, edgF;

const char* resetName(esp_reset_reason_t r){
  switch(r){ case ESP_RST_POWERON: return "POWERON"; case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SW: return "SW"; case ESP_RST_PANIC: return "PANIC"; case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "WDT"; case ESP_RST_EXT: return "EXT(EN)";
    case ESP_RST_USB: return "USB"; default: return "OTHER"; } }

void setDuty(int d){ duty=constrain(d,0,255); ledcWrite(ENA,duty); }

void ramp(){                       // benchtestA, unchanged steps/timing
  Serial.println("# ramp start");
  for(int d=0; d<=255; d+=15){ setDuty(d); Serial.printf("# ramp duty %d\n", d); delay(400); }
  for(int d=255; d>=0; d-=15){ setDuty(d); Serial.printf("# ramp duty %d\n", d); delay(400); }
  Serial.println("# ramp done");
  setDuty(holdDuty);
}

void summary(){
  Serial.println("# ---- summary ----");
  Serial.printf("# HEALTHY n=%ld  p2p mean %.1f sd %.1f  edges/s mean %.2f sd %.2f\n",
    p2pH.n, p2pH.mean(), p2pH.sd(), edgH.mean(), edgH.sd());
  Serial.printf("# FAULT   n=%ld  p2p mean %.1f sd %.1f  edges/s mean %.2f sd %.2f\n",
    p2pF.n, p2pF.mean(), p2pF.sd(), edgF.mean(), edgF.sd());
}

void handleSerial(){
  static String buf;
  while(Serial.available()){
    char c=Serial.read();
    if(c!='\n' && c!='\r'){ buf+=c; continue; }
    buf.trim(); if(!buf.length()) continue;
    char k=toupper(buf[0]);
    if(k=='H'){ label='H'; p2pH=Stat(); edgH=Stat(); Serial.println("# label HEALTHY"); }
    else if(k=='F'){ label='F'; p2pF=Stat(); edgF=Stat(); Serial.println("# label FAULT"); }
    else if(k=='P') summary();
    else if(k=='R') ramp();
    else if(k=='S'){ setDuty(0); Serial.println("# STOP"); }
    else if(k=='G'){ setDuty(holdDuty); Serial.println("# GO"); }
    else if(k=='D'){ holdDuty=constrain(buf.substring(1).toInt(),0,255); setDuty(holdDuty);
                     Serial.printf("# hold duty %d\n", holdDuty); }
    buf="";
  }
}

void setup(){
  Serial.begin(115200);
  delay(1500);                     // give the USB serial monitor time to attach
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  ledcAttach(ENA, 20000, 8);       // 20 kHz PWM, 8-bit
  pinMode(TILT, INPUT_PULLUP);
  // A reset reason other than POWERON/USB/EXT after the motor starts = power/ground problem.
  Serial.printf("# boot, reset reason: %s\n", resetName(esp_reset_reason()));
  ramp();
  Serial.println("t_s,label,duty,p2p_max,p2p_mean,tilt_edges");
}

void loop(){
  static unsigned long sec0=millis();
  static int lastTilt=digitalRead(TILT);
  static long edges=0, p2pSum=0; static int p2pMax=0, windows=0;

  // benchtestB: 100 ms min/max window; benchtestC tilt polled inside the same window
  int mn=4095, mx=0; unsigned long t0=millis();
  while(millis()-t0 < 100){
    int v=analogRead(PIEZO);
    if(v<mn) mn=v; if(v>mx) mx=v;
    int t=digitalRead(TILT);
    if(t!=lastTilt){ edges++; lastTilt=t; }
  }
  int p2p=mx-mn; p2pSum+=p2p; windows++; if(p2p>p2pMax) p2pMax=p2p;
  handleSerial();

  if(millis()-sec0 >= 1000){
    float p2pMean=(float)p2pSum/windows;
    Serial.printf("%lu,%c,%d,%d,%.1f,%ld\n", millis()/1000, label, duty, p2pMax, p2pMean, edges);
    if(duty==holdDuty && duty>0){  // only count steady-state seconds at the hold duty
      if(label=='H'){ p2pH.add(p2pMean); edgH.add(edges); }
      if(label=='F'){ p2pF.add(p2pMean); edgF.add(edges); }
    }
    sec0=millis(); edges=0; p2pSum=0; p2pMax=0; windows=0;
  }
}
