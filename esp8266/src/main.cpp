/*
 * Drone test firmware v0.5  –  30 may 2025
 *  • 4 coreless motors (low–side MOSFET)
 *  • Manual sliders: Throttle, Pitch (fwd/back), Roll (left/right), Yaw (cw/ccw)
 *  • Fallback: works without MPU6050
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/* ---------- Wi-Fi ---------- */
const char* ssid1 = "Spitalul 9", *pass1 = "faraParola5614!";
const char* ssid2 = "upb_guest",  *pass2 = "";

/* ---------- Web ---------- */
ESP8266WebServer server(80);

/* ---------- PWM / motors ---------- */
#define M1 D5   // front-right  CW
#define M2 D6   // front-left   CCW
#define M3 D7   // back-right   CCW
#define M4 D8   // back-left    CW
constexpr uint16_t PWM_RES  = 1023;
constexpr uint32_t PWM_FREQ = 16000;
constexpr uint16_t MOTOR_IDLE = 120;

/* ---------- optional MPU ---------- */
#define SDA_PIN D2
#define SCL_PIN D1
Adafruit_MPU6050 mpu;
bool mpuOK = false;

/* ---------- control vars ---------- */
volatile uint16_t throttlePWM = 0;
volatile int16_t  pitchCmd = 0;   // -512 … 512
volatile int16_t  rollCmd  = 0;
volatile int16_t  yawCmd   = 0;

float pitch = 0, roll = 0, offGX = 0, offGY = 0;
unsigned long tPrev = 0;

/* ---------- helpers ---------- */
uint16_t mixMotor(int32_t v){                    // idle clamp
  v = constrain(v, 0, PWM_RES);
  if (throttlePWM < MOTOR_IDLE) return 0;
  return (v < MOTOR_IDLE) ? MOTOR_IDLE : v;
}

/* ---------- prototypes ---------- */
void handleRoot();
void handleThrottle();
void handleCtrl();

/* ---------- setup ---------- */
void setup(){
  Serial.begin(115200); delay(200);

  WiFi.begin(ssid1, pass1);
  unsigned long t0 = millis(); bool tried2 = false;
  while (WiFi.status()!=WL_CONNECTED){
    delay(400);
    if(!tried2 && millis()-t0>8000){ WiFi.begin(ssid2, pass2); tried2=true; t0=millis(); }
  }
  Serial.printf("Wi-Fi OK  %s\n", WiFi.localIP().toString().c_str());

  pinMode(M1,OUTPUT); pinMode(M2,OUTPUT); pinMode(M3,OUTPUT); pinMode(M4,OUTPUT);
  analogWriteFreq(PWM_FREQ);

  Wire.begin(SDA_PIN, SCL_PIN);
  mpuOK = mpu.begin();
  if(mpuOK){
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange         (MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth   (MPU6050_BAND_21_HZ);
    sensors_event_t a,g,t; const int N=400;
    for(int i=0;i<N;i++){ mpu.getEvent(&a,&g,&t); offGX+=g.gyro.x; offGY+=g.gyro.y; delay(2);}
    offGX/=N; offGY/=N;
    Serial.println("MPU OK – stabilisation active");
  }else Serial.println("MPU absent – manual mode only");

  server.on("/",        handleRoot);
  server.on("/throttle",handleThrottle);
  server.on("/ctrl",    handleCtrl);
  server.begin();
  tPrev = micros();
}

/* ---------- loop ---------- */
void loop(){
  /* read MPU if present */
  int16_t dP=0, dR=0;      // sensor terms
  if(mpuOK){
    sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
    unsigned long now = micros();
    float dt = (now - tPrev)*1e-6f; tPrev = now;
    float accP = atan2(a.acceleration.x, a.acceleration.z);
    float accR = atan2(a.acceleration.y, a.acceleration.z);
    float gX = g.gyro.x - offGX;
    float gY = g.gyro.y - offGY;
    pitch = 0.98f*(pitch + gY*dt) + 0.02f*accP;
    roll  = 0.98f*(roll  + gX*dt) + 0.02f*accR;
    dP = pitch*57.3f*8;
    dR = roll *57.3f*8;
  }

  /* combine commands */
  int16_t pTerm = dP + pitchCmd;   // forward/back
  int16_t rTerm = dR + rollCmd;    // left/right
  int16_t yTerm = yawCmd;          // cw/ccw

  /* motor mix */
  uint16_t m1 = mixMotor(throttlePWM + pTerm + rTerm + yTerm);   // CW
  uint16_t m2 = mixMotor(throttlePWM + pTerm - rTerm - yTerm);   // CCW
  uint16_t m3 = mixMotor(throttlePWM - pTerm + rTerm - yTerm);   // CCW
  uint16_t m4 = mixMotor(throttlePWM - pTerm - rTerm + yTerm);   // CW

  analogWrite(M1,m1); analogWrite(M2,m2);
  analogWrite(M3,m3); analogWrite(M4,m4);

  server.handleClient();
}

/* ---------- HTTP handlers ---------- */
void handleThrottle(){
  if(server.hasArg("value"))
    throttlePWM = constrain(server.arg("value").toInt(),0,PWM_RES);
  server.send(200,"text/plain","OK");
}

void handleCtrl(){
  if(server.hasArg("p")) pitchCmd = constrain(server.arg("p").toInt(),-512,512);
  if(server.hasArg("r")) rollCmd  = constrain(server.arg("r").toInt(),-512,512);
  if(server.hasArg("y")) yawCmd   = constrain(server.arg("y").toInt(),-512,512);
  server.send(200,"text/plain","OK");
}

void handleRoot(){
  String html=R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone Ctrl</title><style>
body{font-family:sans-serif;padding:1em;background:#fafafa}
input[type=range]{-webkit-appearance:none;width:100%;height:3em;margin:.4em 0}
input[type=range]::-webkit-slider-thumb{
  -webkit-appearance:none;width:36px;height:36px;border-radius:50%;background:#2196f3;border:none}
label{font-weight:bold}
.warn{color:#d50000;font-weight:bold}
</style></head><body>

<label>Throttle</label>
<input type="range" id="thr" min="0" max="1023" value="0"
 oninput="sendThr(this.value)"><br>

<label>Pitch (fwd/back)</label>
<input type="range" id="pit" min="-512" max="512" value="0"
 oninput="sendCtrl()" onmouseup="reset(this)" ontouchend="reset(this)"><br>

<label>Roll (left/right)</label>
<input type="range" id="rol" min="-512" max="512" value="0"
 oninput="sendCtrl()" onmouseup="reset(this)" ontouchend="reset(this)"><br>

<label>Yaw (cw/ccw)</label>
<input type="range" id="yaw" min="-512" max="512" value="0"
 oninput="sendCtrl()" onmouseup="reset(this)" ontouchend="reset(this)"><br>

<p id="stat">%STATUS%</p>

<script>
function sendThr(v){ fetch('/throttle?value='+v,{keepalive:true}); }
function val(id){ return document.getElementById(id).value; }
function sendCtrl(){
  fetch('/ctrl?p='+val('pit')+'&r='+val('rol')+'&y='+val('yaw'),{keepalive:true});
}
function reset(el){ el.value=0; sendCtrl(); }
</script></body></html>
)rawliteral";

  html.replace("%STATUS%", mpuOK ? "MPU OK – stabilisation on" :
                                   "<span class=\"warn\">MPU OFFLINE – manual only</span>");
  server.send(200,"text/html",html);
}
