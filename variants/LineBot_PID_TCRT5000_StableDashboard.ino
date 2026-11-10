
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_arduino_version.h>
#include <math.h>

// ======================================================
// SENSORS — YOUR PINS, LEFT TO RIGHT
// ======================================================
const int sensors[7] = {36, 39, 14, 17, 13, 22, 21};
int values[7];

// ======================================================
// MOTOR PINS
// ======================================================
#define L_PWM 25
#define L_DIR1 32
#define L_DIR2 33

#define R_PWM 26
#define R_DIR1 27
#define R_DIR2 4

const int pwmFreq = 1000;
const int pwmResolution = 8;

// ======================================================
// PID DEFAULTS
// ======================================================
float Kp = 8.0;
float Ki = 0.0;
float Kd = 3.0;

float lastError = 0;
float integral = 0;
int baseSpeed = 50;       // Adjustable from dashboard (0-255)
int searchSpeed = 55;     // Adjustable pivot speed when ALL sensors see white (0-255)

// Your TCRT5000 digital outputs read HIGH on black and LOW on white.
const int LINE_VALUE = HIGH;

// Robot must start STOPPED
bool robotEnabled = false;

// ======================================================
// WIFI / SERVER / STORAGE
// ======================================================
const char* apName = "LineBot-PID";
const char* apPassword = "12345678";

WebServer server(80);
Preferences preferences;

// ======================================================
// DASHBOARD
// ======================================================
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LineBot PID</title>
<style>
body{background:#101827;color:#f1f5f9;font-family:Arial;
padding:16px;margin:0}
main{max-width:480px;margin:auto}
.card{background:#1e293b;padding:18px;margin:14px 0;
border-radius:14px}
h2{text-align:center}
label{display:block;font-weight:bold;margin-bottom:12px}
.controls{display:flex;gap:10px;align-items:center}
input[type=range]{flex:1;min-width:0}
input[type=number]{width:90px;padding:9px;background:#0f172a;
color:white;border:1px solid #64748b;border-radius:6px}
button{width:100%;padding:14px;border:0;border-radius:8px;
font-weight:bold;font-size:16px;margin-top:10px}
.start{background:#22c55e;color:#052e16}
.stop{background:#ef4444;color:white}
.apply{background:#38bdf8;color:#082f49}
.status{text-align:center;font-weight:bold;overflow-wrap:anywhere}
small{color:#cbd5e1}
</style>
</head>
<body>
<main>
<h2>LineBot PID Dashboard</h2>

<div class="card">
<h3>Robot control</h3>
<button class="start" onclick="robot('start')">START ROBOT</button>
<button class="stop" onclick="robot('stop')">STOP ROBOT</button>
<p id="robotStatus" class="status">Robot STOPPED</p>
</div>

<div class="card">
<h3>Motor speed (live)</h3>
<label>Following speed: <span id="baseSpeedLabel">50</span> / 255</label>
<div class="controls">
<input id="baseSpeedRange" type="range" min="0" max="255" step="1" value="50" oninput="syncSpeed('baseSpeed',this.value)">
<input id="baseSpeed" type="number" min="0" max="255" step="1" value="50" oninput="syncSpeed('baseSpeed',this.value)">
</div>
<label style="margin-top:18px">Lost-line search speed: <span id="searchSpeedLabel">55</span> / 255</label>
<div class="controls">
<input id="searchSpeedRange" type="range" min="0" max="255" step="1" value="55" oninput="syncSpeed('searchSpeed',this.value)">
<input id="searchSpeed" type="number" min="0" max="255" step="1" value="55" oninput="syncSpeed('searchSpeed',this.value)">
</div>
<p id="speedStatus" class="status">Speed settings ready</p>
<small>Changes are sent to the ESP32 while you move the sliders. Search speed is used only when all 7 sensors read white.</small>
</div>

<div class="card">
<label>Kp — Proportional</label>
<div class="controls">
<input id="kpRange" type="range" min="0" max="100"
step="0.1" value="8" oninput="sync('kp',this.value)">
<input id="kp" type="number" min="0" max="100"
step="0.1" value="8" oninput="sync('kp',this.value)">
</div>
</div>

<div class="card">
<label>Ki — Integral</label>
<div class="controls">
<input id="kiRange" type="range" min="0" max="10"
step="0.01" value="0" oninput="sync('ki',this.value)">
<input id="ki" type="number" min="0" max="10"
step="0.01" value="0" oninput="sync('ki',this.value)">
</div>
</div>

<div class="card">
<label>Kd — Derivative</label>
<div class="controls">
<input id="kdRange" type="range" min="0" max="100"
step="0.1" value="3" oninput="sync('kd',this.value)">
<input id="kd" type="number" min="0" max="100"
step="0.1" value="3" oninput="sync('kd',this.value)">
</div>
</div>

<div class="card">
<button class="apply" onclick="applyPID()">APPLY & SAVE PID</button>
<p id="status" class="status">Connecting...</p>
<p id="actual" class="status"></p>
<small>Settings are saved in ESP32 memory.</small>
</div>
</main>

<script>
function sync(name,value){
  const number=document.getElementById(name);
  const range=document.getElementById(name+'Range');
  // Do not rewrite the field currently being edited; update only its partner.
  if(document.activeElement!==number && document.activeElement!==range){
    number.value=value;
  }
  if(document.activeElement!==range){ range.value=value; }
  if(document.activeElement!==number){ number.value=value; }
}
let speedTimer;
function syncSpeed(name,value){
  // Keep the active number box editable; only clamp/send a valid integer.
  const number=document.getElementById(name);
  const range=document.getElementById(name+'Range');
  const raw=String(value).trim();
  if(raw==='') return;
  const n=Number(raw);
  if(!Number.isFinite(n)) return;
  const safe=Math.max(0,Math.min(255,Math.round(n)));
  if(document.activeElement!==number) number.value=safe;
  if(document.activeElement!==range) range.value=safe;
  document.getElementById(name+'Label').textContent=safe;
  document.getElementById('speedStatus').textContent='Sending speed...';
  clearTimeout(speedTimer);
  speedTimer=setTimeout(sendSpeeds,180);
}
async function sendSpeeds(){
  const base=Number(document.getElementById('baseSpeed').value);
  const search=Number(document.getElementById('searchSpeed').value);
  try{
    const d=await getJSON('/speed?base='+encodeURIComponent(base)+'&search='+encodeURIComponent(search));
    if(!d.ok) throw new Error('ESP32 rejected speed');
    document.getElementById('speedStatus').textContent='Live speed applied';
  }catch(e){document.getElementById('speedStatus').textContent='Speed update failed: '+e.message;}
}

async function getJSON(url){
  const response=await fetch(url,{cache:'no-store'});
  if(!response.ok) throw new Error('HTTP '+response.status);
  return await response.json();
}

async function robot(action){
  const label=document.getElementById('robotStatus');
  label.textContent='Sending command...';
  try{
    const d=await getJSON('/robot?action='+action);
    label.textContent=d.enabled?'Robot RUNNING':'Robot STOPPED';
  }catch(e){
    label.textContent='Command failed: '+e.message;
  }
}

async function loadStatus(){
  try{
    const d=await getJSON('/status');
    // Load ESP32 values without overwriting a control the user is editing.
    if(!['kp','kpRange'].includes(document.activeElement.id)) sync('kp',d.kp);
    if(!['ki','kiRange'].includes(document.activeElement.id)) sync('ki',d.ki);
    if(!['kd','kdRange'].includes(document.activeElement.id)) sync('kd',d.kd);
    if(!['baseSpeed','baseSpeedRange'].includes(document.activeElement.id)) syncSpeedDisplay('baseSpeed',d.baseSpeed);
    if(!['searchSpeed','searchSpeedRange'].includes(document.activeElement.id)) syncSpeedDisplay('searchSpeed',d.searchSpeed);
    document.getElementById('robotStatus').textContent=
      d.enabled?'Robot RUNNING':'Robot STOPPED';
    document.getElementById('status').textContent='Connected to ESP32';
    showActual(d);
  }catch(e){
    document.getElementById('status').textContent=
      'Connection failed: '+e.message;
  }
}

function syncSpeedDisplay(name,value){
  document.getElementById(name).value=value;
  document.getElementById(name+'Range').value=value;
  document.getElementById(name+'Label').textContent=value;
}

function showActual(d){
  document.getElementById('actual').textContent=
    'ESP32 values: Kp='+d.kp+' | Ki='+d.ki+' | Kd='+d.kd;
}

async function applyPID(){
  const kp=Number(document.getElementById('kp').value);
  const ki=Number(document.getElementById('ki').value);
  const kd=Number(document.getElementById('kd').value);

  if(!Number.isFinite(kp)||!Number.isFinite(ki)||
     !Number.isFinite(kd)||kp<0||kp>100||
     ki<0||ki>10||kd<0||kd>100){
    document.getElementById('status').textContent='Invalid values';
    return;
  }

  document.getElementById('status').textContent='Sending values...';

  try{
    const url='/set?kp='+encodeURIComponent(kp)+
      '&ki='+encodeURIComponent(ki)+'&kd='+encodeURIComponent(kd);
    const d=await getJSON(url);

    if(!d.ok) throw new Error('ESP32 rejected values');

    sync('kp',d.kp);
    sync('ki',d.ki);
    sync('kd',d.kd);

    document.getElementById('status').textContent=
      'Applied and saved successfully';
    showActual(d);
  }catch(e){
    document.getElementById('status').textContent=
      'Save failed: '+e.message;
  }
}

loadStatus();
</script>
</body>
</html>
)rawliteral";

// ======================================================
// PWM COMPATIBILITY: ESP32 CORE 2.x / 3.x
// ======================================================
void setupPWM() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttachChannel(L_PWM, pwmFreq, pwmResolution, 0))
    Serial.println("Left PWM setup failed");
  if (!ledcAttachChannel(R_PWM, pwmFreq, pwmResolution, 1))
    Serial.println("Right PWM setup failed");
#else
  ledcSetup(0, pwmFreq, pwmResolution);
  ledcSetup(1, pwmFreq, pwmResolution);
  ledcAttachPin(L_PWM, 0);
  ledcAttachPin(R_PWM, 1);
#endif
}

void writeLeftPWM(int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(L_PWM, duty);
#else
  ledcWrite(0, duty);
#endif
}

void writeRightPWM(int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(R_PWM, duty);
#else
  ledcWrite(1, duty);
#endif
}

// ======================================================
// MOTOR CONTROL
// ======================================================
void setMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(L_DIR1, left >= 0 ? HIGH : LOW);
  digitalWrite(L_DIR2, left >= 0 ? LOW : HIGH);

  digitalWrite(R_DIR1, right >= 0 ? HIGH : LOW);
  digitalWrite(R_DIR2, right >= 0 ? LOW : HIGH);

  writeLeftPWM(abs(left));
  writeRightPWM(abs(right));
}

// ======================================================
// PID STORAGE
// ======================================================
void loadPID() {
  Kp = preferences.getFloat("kp", 8.0);
  Ki = preferences.getFloat("ki", 0.0);
  Kd = preferences.getFloat("kd", 3.0);
  baseSpeed = preferences.getInt("baseSpeed", 50);
  searchSpeed = preferences.getInt("searchSpeed", 55);

  Serial.printf("Loaded PID: Kp=%.2f Ki=%.2f Kd=%.2f\n",
                Kp, Ki, Kd);
}

// ======================================================
// DASHBOARD HANDLERS
// ======================================================
void handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void handleStatus() {
  String json = "{\"ok\":true,\"kp\":" + String(Kp, 3) +
                ",\"ki\":" + String(Ki, 3) +
                ",\"kd\":" + String(Kd, 3) +
                ",\"enabled\":" +
                (robotEnabled ? "true" : "false") +
                ",\"baseSpeed\":" + String(baseSpeed) +
                ",\"searchSpeed\":" + String(searchSpeed) + "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleSetPID() {
  if (!server.hasArg("kp") ||
      !server.hasArg("ki") ||
      !server.hasArg("kd")) {
    server.send(400, "application/json",
                "{\"ok\":false,\"error\":\"Missing parameter\"}");
    return;
  }

  // Reject malformed or empty numbers instead of silently using zero.
  String skp = server.arg("kp");
  String ski = server.arg("ki");
  String skd = server.arg("kd");

  char* endPtr;
  float newKp = strtof(skp.c_str(), &endPtr);
  if (endPtr == skp.c_str() || *endPtr != '\0') {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  float newKi = strtof(ski.c_str(), &endPtr);
  if (endPtr == ski.c_str() || *endPtr != '\0') {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  float newKd = strtof(skd.c_str(), &endPtr);
  if (endPtr == skd.c_str() || *endPtr != '\0') {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  if (!isfinite(newKp) || !isfinite(newKi) || !isfinite(newKd) ||
      newKp < 0 || newKp > 100 ||
      newKi < 0 || newKi > 10 ||
      newKd < 0 || newKd > 100) {
    server.send(400, "application/json",
                "{\"ok\":false,\"error\":\"Out of range\"}");
    return;
  }

  // Apply immediately to the running PID loop.
  Kp = newKp;
  Ki = newKi;
  Kd = newKd;
  integral = 0;
  lastError = 0;

  // Save and verify persistent storage.
  size_t savedKp = preferences.putFloat("kp", Kp);
  size_t savedKi = preferences.putFloat("ki", Ki);
  size_t savedKd = preferences.putFloat("kd", Kd);

  float checkKp = preferences.getFloat("kp", -1);
  float checkKi = preferences.getFloat("ki", -1);
  float checkKd = preferences.getFloat("kd", -1);

  bool saved = savedKp > 0 && savedKi > 0 && savedKd > 0 &&
               fabsf(checkKp - Kp) < 0.001f &&
               fabsf(checkKi - Ki) < 0.001f &&
               fabsf(checkKd - Kd) < 0.001f;

  Serial.printf(
    "PID received: Kp=%.3f Ki=%.3f Kd=%.3f\n",
    Kp, Ki, Kd
  );

  Serial.printf("Saved bytes: Kp=%u Ki=%u Kd=%u\n",
                (unsigned)savedKp,
                (unsigned)savedKi,
                (unsigned)savedKd);

  Serial.printf("Storage verification: %s\n",
                saved ? "OK" : "FAILED");

  String json = String("{\"ok\":") + (saved ? "true" : "false") +
                ",\"kp\":" + String(Kp, 3) +
                ",\"ki\":" + String(Ki, 3) +
                ",\"kd\":" + String(Kd, 3) + "}";

  server.send(saved ? 200 : 500, "application/json", json);
}

void handleSetSpeed() {
  if (!server.hasArg("base") || !server.hasArg("search")) {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  int newBase = server.arg("base").toInt();
  int newSearch = server.arg("search").toInt();
  if (newBase < 0 || newBase > 255 || newSearch < 0 || newSearch > 255) {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }
  baseSpeed = newBase;
  searchSpeed = newSearch;
  String json = String("{\"ok\":true,\"baseSpeed\":") + baseSpeed +
                ",\"searchSpeed\":" + searchSpeed + "}";
  server.send(200, "application/json", json);
}

void handleRobotControl() {
  if (!server.hasArg("action")) {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  String action = server.arg("action");

  if (action == "start") {
    integral = 0;
    lastError = 0;
    robotEnabled = true;
  } else if (action == "stop") {
    robotEnabled = false;
    setMotor(0, 0);
  } else {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  Serial.println(robotEnabled ? "Robot STARTED" : "Robot STOPPED");

  String json = String("{\"ok\":true,\"enabled\":") +
                (robotEnabled ? "true}" : "false}");

  server.send(200, "application/json", json);
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 7; i++) {
    pinMode(sensors[i], INPUT);
  }

  pinMode(L_DIR1, OUTPUT);
  pinMode(L_DIR2, OUTPUT);
  pinMode(R_DIR1, OUTPUT);
  pinMode(R_DIR2, OUTPUT);

  setupPWM();

  // Force stopped at power-up.
  robotEnabled = false;
  setMotor(0, 0);

  preferences.begin("pid-settings", false);
  loadPID();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName, apPassword);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/set", HTTP_GET, handleSetPID);
  server.on("/speed", HTTP_GET, handleSetSpeed);
  server.on("/robot", HTTP_GET, handleRobotControl);
  server.begin();

  Serial.println("\n=== LINEBOT PID DASHBOARD ===");
  Serial.print("Wi-Fi: ");
  Serial.println(apName);
  Serial.print("Password: ");
  Serial.println(apPassword);
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("Robot starts STOPPED.");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  server.handleClient();

  // No motor movement until START is pressed.
  if (!robotEnabled) {
    setMotor(0, 0);
    delay(5);
    return;
  }

  // Read all seven digital TCRT5000 sensors.
  // Confirmed by your test: HIGH (1) = black line, LOW (0) = white floor.
  for (int i = 0; i < 7; i++) {
    values[i] = digitalRead(sensors[i]);
  }

  // Negative error means line is to the robot's left; positive means right.
  // This assumes sensors[] are physically ordered left-to-right.
  const float weights[7] = {-12, -8, -4, 0, 4, 8, 12};
  float sum = 0;
  int count = 0;

  for (int i = 0; i < 7; i++) {
    if (values[i] == LINE_VALUE) {
      sum += weights[i];
      count++;
    }
  }

  // Search ONLY when all 7 sensors read white (LOW).
  // Force the lost-line recovery to pivot LEFT, regardless of lastError.
  // Keep searching left until at least one sensor sees the black line again.
  if (count == 0) {
    integral = 0;  // prevent integral buildup while line is not visible
    setMotor(-searchSpeed, searchSpeed);
    Serial.printf("LINE LOST - forced search LEFT | sensors: %d %d %d %d %d %d %d\n",
                  values[0], values[1], values[2], values[3],
                  values[4], values[5], values[6]);
    delay(8);
    return;  // robot stays enabled; next loop reads sensors again
  }

  float error = sum / count;
  integral += error;
  integral = constrain(integral, -50.0f, 50.0f);

  float derivative = error - lastError;

  // Uses current dashboard values on every loop.
  float correction = Kp * error + Ki * integral + Kd * derivative;
  correction = constrain(correction, -((float)baseSpeed), (float)baseSpeed);

  int leftSpeed = baseSpeed + (int)correction;
  int rightSpeed = baseSpeed - (int)correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotor(leftSpeed, rightSpeed);

  Serial.printf(
    "S: %d %d %d %d %d %d %d | E: %.2f | Kp: %.2f Ki: %.2f Kd: %.2f | L:%d R:%d\n",
    values[0], values[1], values[2], values[3],
    values[4], values[5], values[6],
    error, Kp, Ki, Kd, leftSpeed, rightSpeed
  );

  lastError = error;
  delay(8);
}