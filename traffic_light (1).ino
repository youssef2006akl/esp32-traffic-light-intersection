#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>

// ---------- Pins (one traffic light module per direction) ----------
//                     RED  YELLOW  GREEN
const int NORTH[3] = { 16,   17,     2  };   // RX2, TX2, D2
const int SOUTH[3] = { 27,   32,     33 };   // D27, D32, D33
const int EAST[3]  = { 19,   21,     22 };   // D19, D21, D22
const int WEST[3]  = { 23,   26,     25 };   // D23, D26, D25
const int BUTTON   = 18;                     // D18, pedestrian button to GND

// ---------- TM1650 countdown displays ----------
// North + South displays share one bus, East + West share the other,
// so each pair always shows the same number.
const int NS_SDA = 5,  NS_SCL = 4;           // D5, D4
const int EW_SDA = 13, EW_SCL = 14;          // D13, D14
const bool USE_DISPLAYS = true;              // set to false if no displays are connected

// ---------- Control words (same as the 8086 program) ----------
const uint16_t ALL_RED   = 0x0249;
const uint16_t NS_GREEN  = 0x030C;
const uint16_t NS_YELLOW = 0x028A;
const uint16_t EW_GREEN  = 0x0861;
const uint16_t EW_YELLOW = 0x0451;
const uint16_t NIGHT_YELLOW = 0x0492;        // all four yellow lamps (bits 1, 4, 7, 10)
const uint16_t ALL_OFF      = 0x0000;

// ---------- States ----------
enum Phase { START, NSG, NSY, EWG, EWY, WALK };
enum Mode  { AUTO, EMERGENCY, MANUAL, NIGHT };
enum Street { NS, EW };

Phase phase = START;
Mode mode = AUTO;
Street manualStreet = NS;
Phase afterWalk = NSG;

// ---------- TIMING ----------
// A street is red while the other street is green + yellow, so:
//   N/S red = ewGreenMs + yellowMs = 7 + 3  = 10 s
//   E/W red = nsGreenMs + yellowMs = 15 + 3 = 18 s
unsigned long phaseEnd  = 0;
unsigned long nsGreenMs = 15000;   // North/South green: 15 s
unsigned long ewGreenMs = 7000;    // East/West green:   7 s  (gives N/S a 10 s red)
unsigned long yellowMs  = 3000;    // yellow:            3 s
const unsigned long WALK_MS  = 5000;
const unsigned long START_MS = 1000;

bool pedRequest = false;
unsigned long nightStart = 0;      // when night mode was switched on
unsigned long cycles     = 0;      // full cycles completed (statistics)
unsigned long pedPresses = 0;      // pedestrian button presses (statistics)
uint16_t TFR = 0;

WebServer server(80);
DNSServer dns;                     // makes the control page pop up after joining the Wi-Fi

// Function declarations
void outputWord(uint16_t w);
void setPhase(Phase p);
void nextPhase();
void checkButton();
bool holding();
// ---------- TM1650 displays ----------
// TM1650 talks over 2 wires like I2C: address 0x24 = control, 0x34-0x37 = the 4 digits
const uint8_t DIGITS[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
const uint8_t DASH = 0x40;

void tmInit(TwoWire &bus) {
  bus.beginTransmission(0x24);
  bus.write(0x01);                 // display on, full brightness
  bus.endTransmission();
}

// value >= 0: show the number on the right two digits; value < 0: show "--"
void tmShow(TwoWire &bus, int value) {
  uint8_t seg[4] = {0, 0, 0, 0};
  if (value < 0) {
    seg[2] = DASH; seg[3] = DASH;
  } else {
    if (value > 99) value = 99;
    if (value >= 10) seg[2] = DIGITS[value / 10];
    seg[3] = DIGITS[value % 10];
  }
  for (int i = 0; i < 4; i++) {
    bus.beginTransmission(0x34 + i);
    bus.write(seg[i]);
    bus.endTransmission();
  }
}

long secondsLeft() {
  long left = (long)(phaseEnd - millis());
  if (left < 1) left = 1;          // never show 0: the light changes at that moment
  return (left + 999) / 1000;
}

// seconds until the North/South lights change to the next color
long nsCountdown() {
  long left = secondsLeft();
  long ewG = ewGreenMs / 1000, y = yellowMs / 1000, walk = pedRequest ? WALK_MS / 1000 : 0;
  switch (phase) {
    case START: return left;
    case NSG:   return left;
    case NSY:   return left;
    case EWG:   return left + y + walk;
    case EWY:   return left + walk;
    case WALK:  return afterWalk == NSG ? left : left + ewG + y;
  }
  return left;
}

// seconds until the East/West lights change to the next color
long ewCountdown() {
  long left = secondsLeft();
  long nsG = nsGreenMs / 1000, y = yellowMs / 1000, walk = pedRequest ? WALK_MS / 1000 : 0;
  switch (phase) {
    case START: return left + nsG + y;
    case NSG:   return left + y + walk;
    case NSY:   return left + walk;
    case EWG:   return left;
    case EWY:   return left;
    case WALK:  return afterWalk == EWG ? left : left + nsG + y;
  }
  return left;
}

void updateDisplays() {
  if (!USE_DISPLAYS) return;
  static long lastNs = -100, lastEw = -100;
  long ns = -1, ew = -1;                      // "--" in emergency or manual hold
  if (mode == AUTO || (mode == MANUAL && !holding())) {
    ns = nsCountdown();
    ew = ewCountdown();
  }
  if (ns != lastNs) { tmShow(Wire, ns);  lastNs = ns; }
  if (ew != lastEw) { tmShow(Wire1, ew); lastEw = ew; }
}

void handleRoot();
void handleStatus();
void handleMode();
void handleSet();
void tmInit(TwoWire &bus);
void tmShow(TwoWire &bus, int value);
long secondsLeft();
long nsCountdown();
long ewCountdown();
void updateDisplays();

// ---------- Output a control word (like OUT 4, AX) ----------
// bits 0-2 = N/S red, yellow, green  -> North and South modules
// bits 3-5 = E/W red, yellow, green  -> East and West modules
void outputWord(uint16_t w) {
  for (int i = 0; i < 3; i++) {
    int ns = (w >> i) & 1;
    int ew = (w >> (i + 3)) & 1;
    digitalWrite(NORTH[i], ns);
    digitalWrite(SOUTH[i], ns);
    digitalWrite(EAST[i],  ew);
    digitalWrite(WEST[i],  ew);
  }
  TFR = w;
}

void setPhase(Phase p) {
  phase = p;
  unsigned long d = START_MS;
  uint16_t w = ALL_RED;
  switch (p) {
    case START: w = ALL_RED;   d = START_MS;  break;
    case NSG:   w = NS_GREEN;  d = nsGreenMs; break;
    case NSY:   w = NS_YELLOW; d = yellowMs;  break;
    case EWG:   w = EW_GREEN;  d = ewGreenMs; break;
    case EWY:   w = EW_YELLOW; d = yellowMs;  break;
    case WALK:  w = ALL_RED;   d = WALK_MS;   break;
  }
  outputWord(w);
  phaseEnd = millis() + d;
}

void nextPhase() {
  switch (phase) {
    case START:
      setPhase((mode == MANUAL && manualStreet == EW) ? EWG : NSG);
      break;
    case NSG: setPhase(NSY); break;
    case NSY:
      if (pedRequest) { pedRequest = false; afterWalk = EWG; setPhase(WALK); }
      else setPhase(EWG);
      break;
    case EWG: setPhase(EWY); break;
    case EWY:
      cycles++;
      if (pedRequest) { pedRequest = false; afterWalk = NSG; setPhase(WALK); }
      else setPhase(NSG);
      break;
    case WALK: setPhase(afterWalk); break;
  }
}

// ---------- Pedestrian button ----------
void checkButton() {
  static int lastReading = HIGH;
  static int stableState = HIGH;
  static unsigned long lastChange = 0;

  int reading = digitalRead(BUTTON);
  if (reading != lastReading) {
    lastChange = millis();
    lastReading = reading;
  }
  if (millis() - lastChange > 50 && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) pedPresses++;
    if (stableState == LOW && mode == AUTO) {
      pedRequest = true;
      if (phase == NSG || phase == EWG) {
        long left = (long)(phaseEnd - millis());
        if (left > 2000) phaseEnd = millis() + 2000;
      }
    }
  }
}

// ---------- Web page ----------
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Traffic Light Control</title>
<style>
body{margin:0;font-family:Arial,sans-serif;background:#1B1F27;color:#E8EAED;text-align:center;padding-bottom:20px}
h1{font-size:22px;margin:4px 0 4px}
.sub{color:#9AA3AF;font-size:14px;margin-bottom:12px}
#top{text-align:right;padding:8px 10px 0}
#lang{min-height:34px;font-size:13px;margin:0;padding:0 12px}
#cv{width:340px;height:340px;max-width:94vw;max-height:94vw;border-radius:14px;display:block;margin:0 auto 10px}
.card{background:#242A35;border-radius:12px;padding:14px;margin:12px auto;max-width:332px}
button{min-height:46px;margin:5px;padding:0 16px;border:none;border-radius:10px;font-size:15px;font-weight:bold;cursor:pointer}
.y{background:#F5B83D;color:#1B1F27}.r{background:#FF4D4D;color:#fff}.g{background:#3DDC6E;color:#1B1F27}.o{background:#343B48;color:#E8EAED}
.n{background:#5B6CFF;color:#fff}
input{width:60px;padding:8px;border-radius:8px;border:none;font-size:15px;text-align:center;margin:4px 0}
.on{outline:3px solid #fff}
#tl{position:relative}
.tr{display:flex;align-items:center;margin:4px 0}
.tn{width:38px;font-size:12px;color:#C9CED6;text-align:left}
.tb{flex:1;display:flex;height:22px;border-radius:6px;overflow:hidden}
.tb div{font-size:11px;line-height:22px;color:#111;font-weight:bold;overflow:hidden}
#mk{position:absolute;top:-2px;bottom:-2px;width:3px;margin-left:-1px;background:#fff;border-radius:2px;left:38px}
#ax{font-family:monospace;font-size:18px;font-weight:bold;margin-bottom:8px}
.bg{display:inline-block;margin:0 2px;vertical-align:top}
.bn{font-size:10px;color:#9AA3AF}
.bc{display:inline-block;width:15px;margin:0 1px}
.bv{display:block;height:20px;line-height:20px;border-radius:4px;background:#343B48;color:#9AA3AF;font:bold 12px/20px monospace}
.bl{display:block;font-size:10px;line-height:12px;color:#9AA3AF}
.st{display:flex;justify-content:space-between;font-size:14px;padding:5px 2px;border-bottom:1px solid #343B48}
.st:last-child{border-bottom:none}
.st b{font-size:15px}
</style></head><body>
<div id="top"><button id="lang" class="o"></button></div>
<h1 id="h"></h1>
<div class="sub">ESP32 &middot; <span id="cwl"></span> <b id="w" dir="ltr">----</b></div>
<canvas id="cv" dir="ltr"></canvas>
<div class="sub" id="info"></div>

<div class="card">
 <div class="sub" id="mdl"></div>
 <button class="y" id="bA" onclick="go('/mode?m=auto')"></button>
 <button class="r" id="bE" onclick="go('/mode?m=emergency')"></button>
 <button class="n" id="bNt" onclick="go('/mode?m=night')"></button><br>
 <button class="g" id="bN" onclick="go('/mode?m=ns')"></button>
 <button class="g" id="bW" onclick="go('/mode?m=ew')"></button>
</div>

<div class="card">
 <div class="sub" id="tTl"></div>
 <div id="tl" dir="ltr">
  <div class="tr"><span class="tn" id="tNS"></span><div class="tb" id="rowNS"></div></div>
  <div class="tr"><span class="tn" id="tEW"></span><div class="tb" id="rowEW"></div></div>
  <div id="mk"></div>
 </div>
</div>

<div class="card">
 <div class="sub" id="tCW"></div>
 <div id="ax" dir="ltr">AX = ----</div>
 <div id="bits" dir="ltr"></div>
</div>

<div class="card">
 <div class="sub" id="sTl"></div>
 <div class="st"><span id="s1l"></span><b id="s1">-</b></div>
 <div class="st"><span id="s2l"></span><b id="s2">-</b></div>
 <div class="st"><span id="s3l"></span><b id="s3">0</b></div>
 <div class="st"><span id="s4l"></span><b id="s4" dir="ltr">-</b></div>
</div>

<div class="card">
 <div class="sub" id="tmT"></div>
 <span id="lg1"></span> <input id="g" type="number" min="3" max="60"><br>
 <span id="lg2"></span> <input id="g2" type="number" min="3" max="60"><br>
 <span id="lgy"></span> <input id="yy" type="number" min="1" max="10"><br>
 <button class="o" id="bSv" onclick="go('/set?g='+g.value+'&g2='+g2.value+'&y='+yy.value)"></button>
</div>

<script>
function $(i){return document.getElementById(i);}

// ---------- languages ----------
var T={
en:{title:'Traffic Light Control',cw:'control word',mode:'Mode',auto:'Auto',emg:'Emergency',night:'Night',
 mns:'Manual: N/S green',mew:'Manual: E/W green',tl:'Cycle timeline (seconds)',ns:'N/S',ew:'E/W',
 x86:'8086 control word (OUT 4, AX)',stats:'Statistics',cyc:'Cycles completed',pp:'Pedestrian button presses',
 cars:'Cars passed (simulation)',up:'Uptime',timing:'Timing (seconds)',gns:'N/S green',gew:'E/W green',yel:'Yellow',
 save:'Save timing',conn:'connecting...',off:'connection lost',wait:'pedestrian waiting',
 modes:['AUTO','EMERGENCY','MANUAL','NIGHT'],
 ph:['Start-up all red','N/S green','N/S yellow','E/W green','E/W yellow','Pedestrians crossing'],
 allred:'All red',flash:'Flashing yellow',un:'unused',N:'North',E:'East',S:'South',W:'West',
 d:['N','S','E','W'],lamp:['R','Y','G']},
ar:{title:'التحكم في إشارة المرور',cw:'كلمة التحكم',mode:'الوضع',auto:'تلقائي',emg:'طوارئ',night:'ليلي',
 mns:'يدوي: شمال/جنوب أخضر',mew:'يدوي: شرق/غرب أخضر',tl:'الخط الزمني للدورة (بالثواني)',ns:'ش/ج',ew:'ق/غ',
 x86:'كلمة تحكم 8086 (OUT 4, AX)',stats:'الإحصائيات',cyc:'الدورات المكتملة',pp:'ضغطات زر المشاة',
 cars:'السيارات المارة (محاكاة)',up:'مدة التشغيل',timing:'التوقيت (بالثواني)',gns:'أخضر شمال/جنوب',gew:'أخضر شرق/غرب',yel:'أصفر',
 save:'حفظ التوقيت',conn:'جارٍ الاتصال...',off:'انقطع الاتصال',wait:'مشاة في الانتظار',
 modes:['تلقائي','طوارئ','يدوي','ليلي'],
 ph:['بدء التشغيل: الكل أحمر','شمال/جنوب أخضر','شمال/جنوب أصفر','شرق/غرب أخضر','شرق/غرب أصفر','عبور المشاة'],
 allred:'الكل أحمر',flash:'أصفر وامض',un:'غير مستخدم',N:'شمال',E:'شرق',S:'جنوب',W:'غرب',
 d:['ش','ج','ق','غ'],lamp:['ح','ص','خ']}};
var LG='en';try{LG=localStorage.getItem('lg')=='ar'?'ar':'en';}catch(e){}

var C={R:'#FF4D4D',Y:'#FFD23F',G:'#3DDC6E'},D={R:'#4A2222',Y:'#4A4218',G:'#1C4A2A'};
var first=true,cur=null,endAt=0,offNs=0,offEw=0,nightBase=0,shownW=-1,tlKey='',passed=0;
var st={ns:'R',ew:'R',nsl:-1,ewl:-1,walk:false,ped:false,night:false};
var lastG=null,lastT=0;

function hex(w){return ('000'+w.toString(16).toUpperCase()).slice(-4)+'H';}
function colr(w,sh){
 if((w>>sh)&1)return 'R';
 if((w>>(sh+1))&1)return 'Y';
 if((w>>(sh+2))&1)return 'G';
 return '-';
}

// ---------- 8086 control word, bit by bit ----------
function buildBits(){
 var L=T[LG],h='',grp=[[L.un,15,12],[L.W,11,9],[L.S,8,6],[L.E,5,3],[L.N,2,0]];
 grp.forEach(function(q,gi){
  h+='<span class="bg"><span class="bn">'+q[0]+'</span><br>';
  for(var i=q[1];i>=q[2];i--)
   h+='<span class="bc"><span class="bv" id="b'+i+'">0</span><span class="bl">'+(gi?L.lamp[i%3]:'&nbsp;')+'<br>'+i+'</span></span>';
  h+='</span>';
 });
 $('bits').innerHTML=h;shownW=-1;
}
function showWord(w){
 $('w').textContent=hex(w);$('ax').textContent='AX = '+hex(w);
 for(var i=0;i<16;i++){
  var e=$('b'+i),v=(w>>i)&1,lit=v&&i<12;
  e.textContent=v;
  e.style.background=lit?C['RYG'.charAt(i%3)]:'#343B48';
  e.style.color=lit?'#111':'#9AA3AF';
 }
}

// ---------- cycle timeline ----------
function seg(v,col){return '<div style="flex:'+v+';background:'+col+'">'+v+'</div>';}
function buildTL(a,b,y){
 $('rowNS').innerHTML=seg(a,C.G)+seg(y,C.Y)+seg(b+y,C.R);
 $('rowEW').innerHTML=seg(a+y,C.R)+seg(b,C.G)+seg(y,C.Y);
}
function mark(now){
 var a=cur.g,b=cur.g2,y=cur.y,tot=a+b+2*y,l=Math.max(0,(endAt-now)/1000),p=0;
 switch(cur.pi){
  case 1:p=a-l;break;
  case 2:p=a+y-l;break;
  case 3:p=a+y+b-l;break;
  case 4:p=tot-l;break;
  case 5:p=cur.aw?a+y:0;break;
 }
 if(p<0)p=0;if(p>tot)p=tot;
 var m=$('mk');
 m.style.left=(38+p/tot*($('tl').clientWidth-38))+'px';
 m.style.opacity=cur.ms>=0?1:0.25;
}

// ---------- intersection drawing ----------
var cv=$('cv'),x=cv.getContext('2d');
var S=340,c=170,RW=80,H=40,STOP=c-H-20;
var dpr=window.devicePixelRatio||1;cv.width=S*dpr;cv.height=S*dpr;x.scale(dpr,dpr);
var COL=['#4DA3FF','#FF8A4D','#E8EAED','#C77DFF','#3DDCC9','#FF6B9D'];
// lane 0: cars coming from South, 1: from North, 2: from West, 3: from East
var lanes=[{g:'ns',cars:[],next:0},{g:'ns',cars:[],next:0},{g:'ew',cars:[],next:0},{g:'ew',cars:[],next:0}];

function pos(d,p){
 if(d==0)return[c+H/2,S-p];
 if(d==1)return[c-H/2,p];
 if(d==2)return[p,c+H/2];
 return[S-p,c-H/2];
}
function rr(a,b,w,h,r){x.beginPath();x.moveTo(a+r,b);x.arcTo(a+w,b,a+w,b+h,r);x.arcTo(a+w,b+h,a,b+h,r);x.arcTo(a,b+h,a,b,r);x.arcTo(a,b,a+w,b,r);x.closePath();x.fill();}

function step(dt,t){
 // which street has a car inside the junction, and which has one waiting at its line
 var occ={ns:false,ew:false},wait={ns:false,ew:false},END=S-STOP+22;
 lanes.forEach(function(L){L.cars.forEach(function(k){
  if(k.p>STOP&&k.p<END)occ[L.g]=true;else if(k.p>STOP-1&&k.p<=STOP)wait[L.g]=true;});});
 for(var d=0;d<4;d++){
  var L=lanes[d],col=st[L.g],lim=1e9,o=L.g=='ns'?'ew':'ns',sp=st.night?45:70;
  for(var i=0;i<L.cars.length;i++){
   var k=L.cars[i],m=lim;
   if(k.p<=STOP){
    // normal: wait unless green.  night: slow down, give way, take turns
    var stop=st.night?(occ[o]||(wait[o]&&lastG==L.g&&t-lastT>400)):(col!='G');
    if(stop)m=Math.min(m,STOP);
   }
   var np=Math.max(k.p,Math.min(k.p+sp*dt,m));
   if(st.night&&k.p<=STOP&&np>STOP){occ[L.g]=true;lastG=L.g;lastT=t;}
   k.p=np;
   lim=k.p-30;                                      // keep a gap behind the car in front
  }
  if(L.cars.length&&L.cars[0].p>S+30){L.cars.shift();passed++;$('s3').textContent=passed;}
  var last=L.cars[L.cars.length-1];
  if(t>L.next&&(!last||last.p>35)){
   L.cars.push({p:-5,c:COL[Math.floor(Math.random()*COL.length)]});
   L.next=t+1200+Math.random()*2200;
  }
 }
}

function light(px,py,vert,col,label,cnt,tx,ty,al){
 x.fillStyle='#111317';
 if(vert)rr(px-8,py-20,16,40,5);else rr(px-20,py-8,40,16,5);
 var n=['R','Y','G'];
 for(var i=0;i<3;i++){
  var o=(i-1)*12;
  x.beginPath();x.arc(vert?px:px+o,vert?py+o:py,4.5,0,6.3);
  x.fillStyle=col==n[i]?C[n[i]]:D[n[i]];x.fill();
 }
 x.fillStyle='#E8EAED';x.font='bold 13px Arial';x.textAlign=al;
 x.fillText(label+' '+(cnt<0?'--':cnt),tx,ty);
}

function draw(t){
 x.fillStyle='#22332A';x.fillRect(0,0,S,S);
 x.fillStyle='#2E3440';x.fillRect(c-H,0,RW,S);x.fillRect(0,c-H,S,RW);
 // centre lines
 x.strokeStyle='#F5B83D';x.lineWidth=2;x.setLineDash([8,8]);x.beginPath();
 x.moveTo(c,0);x.lineTo(c,c-H-20);x.moveTo(c,c+H+20);x.lineTo(c,S);
 x.moveTo(0,c);x.lineTo(c-H-20,c);x.moveTo(c+H+20,c);x.lineTo(S,c);x.stroke();x.setLineDash([]);
 // zebra crossings
 x.fillStyle='#C9CED6';
 for(var i=0;i<8;i++){
  var o=c-H+2+i*10;
  x.fillRect(o,c-H-14,6,12);x.fillRect(o,c+H+2,6,12);
  x.fillRect(c-H-14,o,12,6);x.fillRect(c+H+2,o,12,6);
 }
 // stop lines
 x.fillStyle='#fff';
 x.fillRect(c,c+H+17,H,3);x.fillRect(c-H,c-H-20,H,3);
 x.fillRect(c-H-20,c,3,H);x.fillRect(c+H+17,c-H,3,H);
 // cars
 for(var d=0;d<4;d++){
  var L=lanes[d],v=d<2;
  for(var j=0;j<L.cars.length;j++){
   var k=L.cars[j],P=pos(d,k.p-11),F=pos(d,k.p-6);
   x.fillStyle=k.c;
   if(v)rr(P[0]-7,P[1]-11,14,22,4);else rr(P[0]-11,P[1]-7,22,14,4);
   x.fillStyle='rgba(0,0,0,.45)';
   if(v)x.fillRect(F[0]-5,F[1]-2,10,4);else x.fillRect(F[0]-2,F[1]-5,4,10);
  }
 }
 // pedestrians cross while every light is red for them
 if(st.walk){
  var f=(t/5000)%1,a=c-H-10+f*(RW+20),b=c+H+10-f*(RW+20);
  x.fillStyle='#FFD23F';
  [[a,c-H-8],[b,c+H+8],[c-H-8,b],[c+H+8,a]].forEach(function(q){x.beginPath();x.arc(q[0],q[1],4,0,6.3);x.fill();});
 }else if(st.ped){
  x.fillStyle='#FFD23F';
  [[c-H-10,c-H-10],[c+H+10,c+H+10]].forEach(function(q){x.beginPath();x.arc(q[0],q[1],4,0,6.3);x.fill();});
 }
 // traffic lights with their countdowns
 var dn=T[LG].d;
 light(c-H-16,c-H-42,true ,st.ns,dn[0],st.nsl,c-H-30,c-H-38,'right');
 light(c+H+16,c+H+42,true ,st.ns,dn[1],st.nsl,c+H+30,c+H+47,'left');
 light(c+H+42,c-H-16,false,st.ew,dn[2],st.ewl,c+H+42,c-H-30,'center');
 light(c-H-42,c+H+16,false,st.ew,dn[3],st.ewl,c-H-42,c+H+40,'center');
}

// Between messages from the ESP32 the page keeps its own clock, so the
// countdown, the night flashing and the timeline marker move smoothly.
function derive(now){
 if(!cur)return;
 var w=cur.w;
 if(cur.mi==3)w=((now-nightBase)%1000)<500?0x0492:0;
 st.ns=colr(w,0);st.ew=colr(w,3);
 if(cur.ms>=0){
  var l=Math.max(1,Math.ceil((endAt-now)/1000));
  st.nsl=l+offNs;st.ewl=l+offEw;
  if(now>endAt+10&&!asked){asked=true;upd();}     // phase just ended: ask at once
 }else{st.nsl=-1;st.ewl=-1;}
 if(w!=shownW){shownW=w;showWord(w);}
 mark(now);
}

var prev=0;
function frame(t){
 var dt=Math.min((t-prev)/1000,0.05);prev=t;
 derive(t);step(dt,t);draw(t);
 requestAnimationFrame(frame);
}

// ---------- text on the page ----------
function paint(){
 var L=T[LG],s=cur;
 $('info').textContent=L.modes[s.mi]+' · '+(s.mi==1?L.allred:s.mi==3?L.flash:L.ph[s.pi])+(s.ped?' · '+L.wait:'');
 ['bA','bE','bNt','bN','bW'].forEach(function(b){$(b).className=$(b).className.replace(' on','');});
 var act=s.mi==0?'bA':s.mi==1?'bE':s.mi==3?'bNt':(s.mn==0?'bN':'bW');
 $(act).className+=' on';
 $('s1').textContent=s.cyc;$('s2').textContent=s.pp;
 var u=s.up,hh=Math.floor(u/3600),mm=Math.floor(u/60)%60,ss=u%60;
 $('s4').textContent=hh+':'+(mm<10?'0':'')+mm+':'+(ss<10?'0':'')+ss;
}
function applyLang(){
 var L=T[LG];
 document.documentElement.dir=LG=='ar'?'rtl':'ltr';document.documentElement.lang=LG;
 var m={h:'title',cwl:'cw',mdl:'mode',bA:'auto',bE:'emg',bNt:'night',bN:'mns',bW:'mew',tTl:'tl',tNS:'ns',tEW:'ew',
  tCW:'x86',sTl:'stats',s1l:'cyc',s2l:'pp',s3l:'cars',s4l:'up',tmT:'timing',lg1:'gns',lg2:'gew',lgy:'yel',bSv:'save'};
 for(var k in m)$(k).textContent=L[m[k]];
 $('lang').textContent=LG=='en'?'العربية':'English';
 document.title=L.title;
 buildBits();
 if(cur)paint();else $('info').textContent=L.conn;
}
$('lang').onclick=function(){LG=LG=='en'?'ar':'en';try{localStorage.setItem('lg',LG);}catch(e){}applyLang();};

// ---------- talk to the ESP32 ----------
var busy=0,asked=false;
function recv(s){
 var now=performance.now();
 cur=s;endAt=now+s.ms;
 var l0=Math.max(1,Math.ceil(s.ms/1000));
 offNs=s.nsl-l0;offEw=s.ewl-l0;
 nightBase=now-s.nb;
 st.ped=!!s.ped;st.night=(s.mi==3);st.walk=(s.mi!=1&&s.mi!=3&&s.pi==5);
 var key=s.g+'/'+s.g2+'/'+s.y;
 if(key!=tlKey){tlKey=key;buildTL(s.g,s.g2,s.y);}
 if(first){g.value=s.g;g2.value=s.g2;yy.value=s.y;first=false;}
 paint();
}
function upd(){
 var n=Date.now();if(n-busy<1500)return;busy=n;
 fetch('/status').then(function(r){return r.json();}).then(function(s){busy=0;asked=false;recv(s);})
  .catch(function(){busy=0;$('info').textContent=T[LG].off;});
}
function go(u){fetch(u).then(function(){busy=0;upd();});}

applyLang();
requestAnimationFrame(frame);
setInterval(upd,250);upd();
</script></body></html>
)rawliteral";

bool holding() {
  return mode == MANUAL &&
         ((phase == NSG && manualStreet == NS) || (phase == EWG && manualStreet == EW));
}

void handleRoot() { server.send_P(200, "text/html", PAGE); }

// Any other address the phone asks for is sent to the control page.
// This is what makes the page open by itself after joining the Wi-Fi.
void handleNotFound() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

void handleStatus() {
  // ms = time left in this phase, -1 when nothing is counting down.
  // nsl / ewl = the numbers the TM1650 displays show.
  bool timed = (mode == AUTO || (mode == MANUAL && !holding()));
  long ms = -1, nsl = -1, ewl = -1;
  if (timed) {
    ms = (long)(phaseEnd - millis());
    if (ms < 0) ms = 0;
    nsl = nsCountdown();
    ewl = ewCountdown();
  }
  // nb = where we are inside the 1-second night flash, so the page blinks in step
  unsigned long nb = (mode == NIGHT) ? (millis() - nightStart) % 1000 : 0;

  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"w\":%u,\"mi\":%d,\"mn\":%d,\"pi\":%d,\"aw\":%d,\"ped\":%d,"
    "\"ms\":%ld,\"nsl\":%ld,\"ewl\":%ld,\"nb\":%lu,"
    "\"g\":%lu,\"g2\":%lu,\"y\":%lu,\"cyc\":%lu,\"pp\":%lu,\"up\":%lu}",
    (unsigned)TFR, (int)mode, (int)manualStreet, (int)phase,
    afterWalk == EWG ? 1 : 0, pedRequest ? 1 : 0,
    ms, nsl, ewl, nb,
    nsGreenMs / 1000, ewGreenMs / 1000, yellowMs / 1000,
    cycles, pedPresses, millis() / 1000);
  server.send(200, "application/json", buf);
}

void handleMode() {
  String m = server.arg("m");
  bool stopped = (mode == EMERGENCY || mode == NIGHT);   // the normal cycle is not running
  if (m == "emergency") {
    mode = EMERGENCY;
    pedRequest = false;
    outputWord(ALL_RED);
  } else if (m == "night") {
    if (mode != NIGHT) {
      mode = NIGHT;
      pedRequest = false;
      nightStart = millis();
      outputWord(NIGHT_YELLOW);
    }
  } else if (m == "auto") {
    if (stopped) setPhase(START);        // always restart through all red
    mode = AUTO;
  } else if (m == "ns" || m == "ew") {
    mode = MANUAL;
    manualStreet = (m == "ns") ? NS : EW;
    pedRequest = false;
    if (stopped) setPhase(START);
  }
  server.send(200, "text/plain", "OK");
}

void handleSet() {
  long g  = server.arg("g").toInt();
  long g2 = server.arg("g2").toInt();
  long y  = server.arg("y").toInt();
  if (g  >= 3 && g  <= 60) nsGreenMs = g  * 1000UL;
  if (g2 >= 3 && g2 <= 60) ewGreenMs = g2 * 1000UL;
  if (y  >= 1 && y  <= 10) yellowMs  = y  * 1000UL;
  server.send(200, "text/plain", "OK");
}

// ---------- Setup and main loop ----------
void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(NORTH[i], OUTPUT);
    pinMode(SOUTH[i], OUTPUT);
    pinMode(EAST[i],  OUTPUT);
    pinMode(WEST[i],  OUTPUT);
  }
  pinMode(BUTTON, INPUT_PULLUP);

  if (USE_DISPLAYS) {
    Wire.begin(NS_SDA, NS_SCL);
    Wire1.begin(EW_SDA, EW_SCL);
    tmInit(Wire);
    tmInit(Wire1);
  }

  setPhase(START);

  WiFi.mode(WIFI_AP);
  WiFi.softAP("TrafficLight", "12345678");
  dns.start(53, "*", WiFi.softAPIP());       // answer every name with our own address

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/mode", handleMode);
  server.on("/set", handleSet);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  dns.processNextRequest();
  server.handleClient();
  checkButton();
  updateDisplays();

  if (mode == EMERGENCY) return;

  if (mode == NIGHT) {                   // flash all yellow: 0.5 s on, 0.5 s off
    bool on = ((millis() - nightStart) % 1000) < 500;
    uint16_t w = on ? NIGHT_YELLOW : ALL_OFF;
    if (w != TFR) outputWord(w);
    return;
  }

  if (mode == MANUAL) {
    if (holding()) return;
    if (phase == NSG || phase == EWG) { nextPhase(); return; }
  }

  if ((long)(millis() - phaseEnd) >= 0) nextPhase();
}
