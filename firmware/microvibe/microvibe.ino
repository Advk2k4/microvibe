// MicroVibe firmware. Evolved from bench/benchtestA (motor), benchtestB (piezo), benchtestC (tilt)
// and bench/m1_baseline. The ESP32 only samples and streams; RMS/FFT/baseline/alerts live in Python
// (dashboard/dashboard.py). Serial protocol (CSV lines, 115200) is documented in README.md:
//   R,<seq>,v1..v50          raw piezo ADC samples at 1000 Hz; seq = index of v1
//   S,<seq_end>,<t_ms>,<duty>,<killed>,<brake>,<tach_pulses>,<tach_min>,<tach_max>,
//     <tilt_edges>,<therm_adc>,<pot_adc>,<dropped_blocks>      once per 1000 samples (1 s)
//   E,<t_ms>,<text>          events (commands, buttons)
//   # ...                    comments / boot info
// Commands (one per line): D<0-255> duty | K kill (duty 0, latched) | G go (clear kill, hold duty)
//   B1 / B0 servo brake on/off | Vb<deg> / Vr<deg> brake / release angle | P1 / P0 pot control on/off
//   X0 / X1 / X2 dashboard state HEALTHY / WARNING / FAULT (drives RGB, buzzer, LCD)
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "esp_system.h"

// ---- pins (README has the full table) ----
const int ENA=5, IN1=6, IN2=7;                  // L293D, from benchtestA
const int PIEZO=1, TILT=4;                      // from benchtestB / benchtestC
const int THERM=2, TACH=10;                     // ADC1
const int POT=11;                               // ADC2, fine with WiFi off
const int SDA_PIN=8, SCL_PIN=9;                 // LCD1602 I2C
const int SERVO=13, ALARM=14;
const int LED_R=15, LED_G=16, LED_B=17;
const int SR_DATA=12, SR_CLK=18, SR_LATCH=21;   // 74HC595 -> LED bar graph
const int BTN_LEARN=38, BTN_INJECT=39, BTN_RESET=40;
const int SD_CS=41, SD_SCK=42, SD_MOSI=47, SD_MISO=48;

const int FS_HZ=1000;                              // piezo sample rate, Hz (1 FreeRTOS tick)
const int BLOCK=50;                             // samples per R line
const int BLADES=3;

// ---- shared state ----
struct Block { uint32_t seq; uint16_t v[BLOCK]; };
struct SecStats { uint32_t seqEnd; uint16_t tachPulses, tachMin, tachMax, tiltEdges, therm, pot; };
QueueHandle_t blockQ, secQ;
volatile uint32_t droppedBlocks=0;
volatile uint16_t potNow=0;

int holdDuty=180, duty=0;
bool killed=false, brake=false, potControl=false;
int brakeDeg=60, releaseDeg=0;
int dashState=0;                                // 0 healthy, 1 warning, 2 fault (set by dashboard)
int lastRpm=-1;
uint8_t lcdAddr=0;
bool sdOk=false; File logFile;
int barFullScale=100;                           // RMS counts that light all 8 bars (display only)
float lastBlockRms=0;

// ---- sampler: core 0, one sample per 1 ms tick. All ADC reads happen here. ----
void sampler(void*){
  TickType_t last=xTaskGetTickCount();
  Block b; int i=0; uint32_t seq=0;
  int lastTilt=digitalRead(TILT);
  uint16_t tMin=4095, tMax=0, pTachMin=0, pTachMax=0, pulses=0, edges=0;
  bool tachHigh=false; uint32_t thermSum=0; int thermN=0;
  for(;;){
    vTaskDelayUntil(&last, 1);
    uint16_t v=analogRead(PIEZO);
    if(i==0) b.seq=seq;
    b.v[i++]=v;
    if(i==BLOCK){ if(xQueueSend(blockQ,&b,0)!=pdTRUE) droppedBlocks++; i=0; }

    int t=digitalRead(TILT);
    if(t!=lastTilt){ edges++; lastTilt=t; }

    // tach: threshold at the middle of last second's swing, with hysteresis
    uint16_t a=analogRead(TACH);
    if(a<tMin) tMin=a; if(a>tMax) tMax=a;
    int swing=pTachMax-pTachMin, mid=(pTachMax+pTachMin)/2, hyst=swing/4;
    if(!tachHigh && a>mid+hyst){ tachHigh=true; pulses++; }
    else if(tachHigh && a<mid-hyst) tachHigh=false;

    uint32_t k=seq%FS_HZ;
    if(k%100==0) potNow=analogRead(POT);
    if(k>=FS_HZ-8){ thermSum+=analogRead(THERM); thermN++; }
    seq++;
    if(seq%FS_HZ==0){
      SecStats s{seq, pulses, tMin, tMax, edges, (uint16_t)(thermSum/max(thermN,1)), potNow};
      xQueueSend(secQ,&s,0);
      pTachMin=tMin; pTachMax=tMax; tMin=4095; tMax=0; pulses=0; edges=0; thermSum=0; thermN=0;
    }
  }
}

// ---- output helpers ----
char line[400];
void emit(){                                    // send `line` to serial and SD
  Serial.println(line);
  if(sdOk){ logFile.println(line); }
}
void event(const char* txt){ snprintf(line,sizeof line,"E,%lu,%s",millis(),txt); emit(); }

void setDuty(int d){ duty=constrain(d,0,255); ledcWrite(ENA,duty); }

void servoDeg(int deg){                         // 50 Hz, 14-bit: 500..2500 us for 0..180 deg
  uint32_t us=500+(uint32_t)constrain(deg,0,180)*2000/180;
  ledcWrite(SERVO, us*16384/20000);
}
void setBrake(bool on){ brake=on; servoDeg(on? brakeDeg : releaseDeg); event(on? "BRAKE_ON" : "BRAKE_OFF"); }

void bar(int level){                            // 0..8 LEDs on the 74HC595
  uint8_t bits= level<=0? 0 : (level>=8? 0xFF : (uint8_t)((1<<level)-1));
  digitalWrite(SR_LATCH,LOW); shiftOut(SR_DATA,SR_CLK,MSBFIRST,bits); digitalWrite(SR_LATCH,HIGH);
}

void rgb(int r,int g,int b){ digitalWrite(LED_R,r); digitalWrite(LED_G,g); digitalWrite(LED_B,b); }

// ---- minimal LCD1602 over PCF8574 backpack (P0 RS, P2 EN, P3 backlight, P4-7 D4-7) ----
void lcdNib(uint8_t nib, uint8_t rs){
  uint8_t d=(nib<<4)|0x08|rs;
  Wire.beginTransmission(lcdAddr); Wire.write(d|0x04); Wire.write(d); Wire.endTransmission();
}
void lcdByte(uint8_t b, uint8_t rs){ lcdNib(b>>4,rs); lcdNib(b&0x0F,rs); }
void lcdCmd(uint8_t c){ lcdByte(c,0); if(c<4) delay(2); }
void lcdInit(){
  delay(50); lcdNib(3,0); delay(5); lcdNib(3,0); delay(1); lcdNib(3,0); lcdNib(2,0);
  lcdCmd(0x28); lcdCmd(0x0C); lcdCmd(0x06); lcdCmd(0x01);
}
void lcdRow(int row, const char* s){
  lcdCmd(0x80|(row? 0x40:0));
  bool end=false; for(int i=0;i<16;i++){ if(!s[i]) end=true; lcdByte(end? ' ' : s[i],1); }
}

void i2cScan(){
  Wire.begin(SDA_PIN,SCL_PIN);
  String found="# I2C scan:";
  for(uint8_t a=1;a<127;a++){
    Wire.beginTransmission(a);
    if(Wire.endTransmission()==0){ found+=" 0x"+String(a,HEX); if(!lcdAddr && ((a>=0x20&&a<=0x27)||(a>=0x38&&a<=0x3F))) lcdAddr=a; }
  }
  Serial.println(found==String("# I2C scan:")? "# I2C scan: nothing found (LCD not wired?)" : found);
  if(lcdAddr){ lcdInit(); lcdRow(0,"MicroVibe"); lcdRow(1,"booting"); Serial.printf("# LCD at 0x%02X\n",lcdAddr); }
}

void sdInit(){
  SPI.begin(SD_SCK,SD_MISO,SD_MOSI,SD_CS);
  if(!SD.begin(SD_CS,SPI,4000000)){ Serial.println("# SD: not found, logging to serial only"); return; }
  char name[16];
  for(int n=0;n<1000;n++){ snprintf(name,sizeof name,"/mv%03d.csv",n); if(!SD.exists(name)) break; }
  logFile=SD.open(name,FILE_WRITE);
  sdOk=(bool)logFile;
  Serial.printf("# SD: %s %s\n", sdOk? "logging to":"cannot open", name);
}

// ---- commands ----
void command(String c){
  c.trim(); if(!c.length()) return;
  char k=toupper(c[0]); String arg=c.substring(1); char buf[48];
  if(k=='D'){ holdDuty=constrain(arg.toInt(),0,255); potControl=false; if(!killed) setDuty(holdDuty);
              snprintf(buf,sizeof buf,"DUTY_%d",holdDuty); event(buf); }
  else if(k=='K'){ killed=true; setDuty(0); event("KILL"); }
  else if(k=='G'){ killed=false; setDuty(holdDuty); event("GO"); }
  else if(k=='B'){ setBrake(arg.toInt()==1); }
  else if(k=='V' && arg.length()>1){ int d=arg.substring(1).toInt();
              if(toupper(arg[0])=='B') brakeDeg=d; else releaseDeg=d;
              servoDeg(brake? brakeDeg : releaseDeg); snprintf(buf,sizeof buf,"SERVO_%s_%d",toupper(arg[0])=='B'?"BRAKE":"RELEASE",d); event(buf); }
  else if(k=='P'){ potControl=(arg.toInt()==1); event(potControl? "POT_ON":"POT_OFF"); }
  else if(k=='X'){ dashState=constrain(arg.toInt(),0,2); }
}

void readSerial(){
  static String buf;
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\n'||ch=='\r'){ command(buf); buf=""; } else if(buf.length()<40) buf+=ch;
  }
}

// ---- buttons (INPUT_PULLUP, pressed = LOW) ----
struct Btn { int pin; bool last; unsigned long t; };
Btn btns[3]={{BTN_LEARN,true,0},{BTN_INJECT,true,0},{BTN_RESET,true,0}};
void readButtons(){
  for(int i=0;i<3;i++){
    bool v=digitalRead(btns[i].pin);
    if(v!=btns[i].last && millis()-btns[i].t>30){
      btns[i].t=millis(); btns[i].last=v;
      if(!v){
        if(i==0) event("BTN_LEARN");                          // dashboard starts baseline learning
        if(i==1){ event("BTN_INJECT"); setBrake(!brake); }    // toggle servo brake
        if(i==2){ event("BTN_RESET"); setBrake(false); killed=false; setDuty(holdDuty); }
      }
    }
  }
}

void setup(){
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  delay(1500);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  ledcAttach(ENA, 20000, 8);                    // 20 kHz PWM, 8-bit (benchtestA)
  ledcAttach(SERVO, 50, 14);
  pinMode(TILT, INPUT_PULLUP);
  for(int p: {BTN_LEARN,BTN_INJECT,BTN_RESET}) pinMode(p, INPUT_PULLUP);
  for(int p: {ALARM,LED_R,LED_G,LED_B,SR_DATA,SR_CLK,SR_LATCH}) pinMode(p, OUTPUT);
  digitalWrite(ALARM,LOW); rgb(0,0,1); bar(0);

  Serial.println("# MicroVibe firmware v1");
  Serial.printf("# boot, reset reason: %d (1=POWERON 3=SW 4=PANIC 9=BROWNOUT 21=USB)\n", (int)esp_reset_reason());
  Serial.printf("# fs_hz=%d block=%d blades=%d\n", FS_HZ, BLOCK, BLADES);
  Serial.println("# R,seq,v1..v50 | S,seq_end,t_ms,duty,killed,brake,tach_pulses,tach_min,tach_max,tilt_edges,therm_adc,pot_adc,dropped | E,t_ms,text");
  i2cScan();
  sdInit();
  servoDeg(releaseDeg);

  blockQ=xQueueCreate(64,sizeof(Block));        // 3.2 s of slack for slow SD/LCD writes
  secQ=xQueueCreate(8,sizeof(SecStats));
  xTaskCreatePinnedToCore(sampler,"sampler",4096,nullptr,5,nullptr,0);
  setDuty(holdDuty);
  event("BOOT");
}

void loop(){
  readSerial();
  readButtons();

  Block b;
  while(xQueueReceive(blockQ,&b,0)==pdTRUE){
    int n=snprintf(line,sizeof line,"R,%lu",b.seq);
    double s=0, ss=0;
    for(int i=0;i<BLOCK;i++){ n+=snprintf(line+n,sizeof line-n,",%u",b.v[i]); s+=b.v[i]; ss+=(double)b.v[i]*b.v[i]; }
    emit();
    double m=s/BLOCK; lastBlockRms=sqrt(max(0.0,ss/BLOCK-m*m));
  }

  SecStats st;
  while(xQueueReceive(secQ,&st,0)==pdTRUE){
    snprintf(line,sizeof line,"S,%lu,%lu,%d,%d,%d,%u,%u,%u,%u,%u,%u,%lu", st.seqEnd, millis(), duty,
      killed, brake, st.tachPulses, st.tachMin, st.tachMax, st.tiltEdges, st.therm, st.pot, droppedBlocks);
    emit();
    lastRpm = (st.tachMax-st.tachMin>=40)? st.tachPulses*60/BLADES : -1;   // -1: no usable tach swing
    if(sdOk) logFile.flush();
  }

  if(potControl && !killed){
    static int lastPotDuty=-1; int d=potNow*255/4095;
    if(abs(d-lastPotDuty)>4){ lastPotDuty=d; holdDuty=d; setDuty(d); }
  }

  static unsigned long tUi=0;
  if(millis()-tUi>=100){
    tUi=millis();
    bar((int)(lastBlockRms*8/barFullScale));
    if(dashState==0) rgb(0,1,0); else if(dashState==1) rgb(1,1,0); else rgb(1,0,0);
    digitalWrite(ALARM, dashState==2 && (millis()%1000)<200);
    static int lcdTick=0;
    if(lcdAddr && ++lcdTick>=5){
      lcdTick=0; char r0[17], r1[17];
      if(lastRpm>=0) snprintf(r0,sizeof r0,"RPM %-5d D%-3d",lastRpm,duty); else snprintf(r0,sizeof r0,"RPM --    D%-3d",duty);
      const char* stName= dashState==2? "FAULT" : dashState==1? "WARNING" : "HEALTHY";
      snprintf(r1,sizeof r1,"%-8s%s%s",stName,killed?"KILL ":"",brake?"BRK":"");
      lcdRow(0,r0); lcdRow(1,r1);
    }
  }
}
