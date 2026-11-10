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

float Kp = 25.0;

float Ki = 0.0;

float Kd = 5.0;

float lastError = 0;

float integral = 0;

int baseSpeed = 70;

int searchSpeed = 70;

// Change this if your sensor outputs are reversed.

const int LINE_VALUE = HIGH; // 1/HIGH means black

// Robot must start STOPPED

bool robotEnabled = false;

// On START, drive straight at exactly 120/120 for 2 seconds.

bool startupForward = false;

unsigned long startupForwardStart = 0;

const unsigned long STARTUP_FORWARD_MS = 700;

// White-line mode: detection becomes eligible 7 seconds after START.

// HIGH=black and LOW=white. A white line on a black surface is detected

// when black readings are the majority.

bool followWhite = false;

bool whiteDetectionEnabled = false;

unsigned long robotStartTime = 0;

int whiteModeConfidence = 0;

const unsigned long WHITE_DETECTION_DELAY_MS = 10000;

const int WHITE_MODE_CONFIDENCE_LIMIT = 5;

const int WHITE_MODE_SWITCH_THRESHOLD = 2;

// First-circle maneuver: trigger once at the first middle-three junction.

int junctionCount = 0;

bool junctionArmed = true;

bool firstCircleTurnDone = false;

bool firstCircleTurnActive = false;

bool firstCircleJunctionReady = false; // arm only after leaving the starting sensor pattern

unsigned long firstCircleTurnStart = 0;

const unsigned long FIRST_CIRCLE_LEFT_TURN_MS = 1000;

// Upper-loop route lock: sequence center-only -> middle-three -> sensors 1,3,5.

bool upperRouteLocked = false;

int upperEntryStage = 0;

unsigned long junctionClearStart = 0;

const unsigned long JUNCTION_CLEAR_MS = 35;

// If the line disappears, search in the direction of the last known line position.

bool lineWasLost = false;

unsigned long lineLossStart = 0;

const unsigned long LINE_LOSS_TIMEOUT_MS = 1200;

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

<label>Following speed: <span id="speedValue">70</span></label>

<div class="controls">

<input id="speedRange" type="range" min="0" max="255" step="1" value="70"

oninput="syncSpeed(this.value)">

<input id="speed" type="number" min="0" max="255" step="1" value="70"

oninput="syncSpeed(this.value)">

</div>

<button class="apply" onclick="applySpeed()">APPLY SPEED</button>

<p id="speedStatus" class="status">Speed: 70</p>

</div>

<div class="card">

<label>Kp — Proportional</label>

<div class="controls">

<input id="kpRange" type="range" min="0" max="100"

step="0.1" value="25" oninput="sync('kp',this.value)">

<input id="kp" type="number" min="0" max="100"

step="0.1" value="25" oninput="sync('kp',this.value)">

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

step="0.1" value="5" oninput="sync('kd',this.value)">

<input id="kd" type="number" min="0" max="100"

step="0.1" value="5" oninput="sync('kd',this.value)">

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

  document.getElementById(name).value=value;

  document.getElementById(name+'Range').value=value;

}

function syncSpeed(value){

  document.getElementById('speed').value=value;

  document.getElementById('speedRange').value=value;

  document.getElementById('speedValue').textContent=value;

}

async function applySpeed(){

  const speed=Number(document.getElementById('speed').value);

  if(!Number.isInteger(speed)||speed<0||speed>255){

    document.getElementById('speedStatus').textContent='Speed must be 0–255';

    return;

  }

  try{

    const d=await getJSON('/speed?value='+encodeURIComponent(speed));

    if(!d.ok) throw new Error('ESP32 rejected speed');

    syncSpeed(d.speed);

    document.getElementById('speedStatus').textContent='Speed applied: '+d.speed;

  }catch(e){

    document.getElementById('speedStatus').textContent='Speed update failed: '+e.message;

  }

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

    sync('kp',d.kp);

    sync('ki',d.ki);

    sync('kd',d.kd);

    syncSpeed(d.speed);

    document.getElementById('robotStatus').textContent=

      d.enabled?'Robot RUNNING':'Robot STOPPED';

    document.getElementById('status').textContent='Connected to ESP32';

    showActual(d);

  }catch(e){

    document.getElementById('status').textContent=

      'Connection failed: '+e.message;

  }

}

function showActual(d){

  document.getElementById('actual').textContent=

    'ESP32 values: Speed='+d.speed+' | Kp='+d.kp+' | Ki='+d.ki+' | Kd='+d.kd;

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

  Kp = preferences.getFloat("kp", 25.0);

  Ki = preferences.getFloat("ki", 0.0);

  Kd = preferences.getFloat("kd", 5.0);

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

  String json = "{\"ok\":true,\"speed\":" + String(baseSpeed) + ",\"kp\":" + String(Kp, 3) +

                ",\"ki\":" + String(Ki, 3) +

                ",\"kd\":" + String(Kd, 3) +

                ",\"enabled\":" +

                (robotEnabled ? "true" : "false") + "}";

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

  if (!server.hasArg("value")) {

    server.send(400, "application/json", "{\"ok\":false}");

    return;

  }

  int newSpeed = server.arg("value").toInt();

  if (newSpeed < 0 || newSpeed > 255) {

    server.send(400, "application/json", "{\"ok\":false}");

    return;

  }

  baseSpeed = newSpeed;

  searchSpeed = newSpeed;

  Serial.printf("Speed set to %d\n", baseSpeed);

  server.send(200, "application/json",

              String("{\"ok\":true,\"speed\":") + String(baseSpeed) + "}");

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

    followWhite = false;

    whiteDetectionEnabled = false;

    whiteModeConfidence = 0;

    robotStartTime = millis();

    junctionCount = 0;

    junctionArmed = true;

    firstCircleTurnDone = false;

    firstCircleTurnActive = false;

    firstCircleJunctionReady = false;

    junctionClearStart = 0;

    lineWasLost = false;

    lineLossStart = 0;

    startupForward = true;

    startupForwardStart = millis();

    robotEnabled = true;

  } else if (action == "stop") {

    robotEnabled = false;

    startupForward = false;

    setMotor(0, 0);

    followWhite = false;

    whiteDetectionEnabled = false;

    whiteModeConfidence = 0;

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

  if (!robotEnabled) {

    setMotor(0, 0);

    delay(5);

    return;

  }

  // Drive straight at exactly 120/120 for 2 seconds after START.

  // This is non-blocking so STOP and dashboard requests stay responsive.

  if (startupForward) {

    if (millis() - startupForwardStart < STARTUP_FORWARD_MS) {

      setMotor(120, 120);

      return;

    }

    startupForward = false;

    integral = 0;

    lastError = 0;

    lineWasLost = false;

    lineLossStart = 0;

    Serial.println("Startup forward complete - line following enabled");

  }

  for (int i = 0; i < 7; i++) values[i] = digitalRead(sensors[i]);

    // Enable white-line detection only after 7 seconds of black-line following.

    if (!followWhite && !whiteDetectionEnabled &&

        millis() - robotStartTime >= WHITE_DETECTION_DELAY_MS) {

      whiteDetectionEnabled = true;

      whiteModeConfidence = 0;

      Serial.println("White-line detection enabled; still following black line");

    }

    // White line on a black surface: background sensors read HIGH,

    // while the white line sensors read LOW.

    if (whiteDetectionEnabled && !followWhite) {

      int blackCount = 0;

      int whiteCount = 0;

      for (int i = 0; i < 7; i++) {

        if (values[i] == HIGH) blackCount++;

        else whiteCount++;

      }

      if (blackCount > whiteCount && whiteCount > 0) whiteModeConfidence++;

      else whiteModeConfidence--;

      whiteModeConfidence = constrain(whiteModeConfidence,

                                      -WHITE_MODE_CONFIDENCE_LIMIT,

                                      WHITE_MODE_CONFIDENCE_LIMIT);

      if (whiteModeConfidence > WHITE_MODE_SWITCH_THRESHOLD) {

        followWhite = true;

        integral = 0;

        lastError = 0;

        lineWasLost = false;

        lineLossStart = 0;

        upperRouteLocked = false;

        upperEntryStage = 0;

        firstCircleTurnActive = false;

        junctionCount = 0;

        junctionArmed = true;

        junctionClearStart = 0;

        Serial.println("WHITE LINE DETECTED - switched to white-line following");

      }

    }

    const int targetLineValue = followWhite ? LOW : LINE_VALUE;

    // Track the distinctive sequence leading into the upper/lower loop.

  // Sensor indexes are zero-based: [3] center, [2..4] middle three,

  // and [1], [3], [5] the alternating pattern described by the user.

  bool centerOnly = values[3] == targetLineValue;

  for (int i = 0; i < 7; i++) {

    if (i != 3 && values[i] == targetLineValue) centerOnly = false;

  }

  bool middleThree = values[2] == targetLineValue && values[3] == targetLineValue && values[4] == targetLineValue;

  bool alternating135 = values[1] == targetLineValue && values[3] == targetLineValue && values[5] == targetLineValue;

  bool sensor2Only = values[2] == targetLineValue;

  for (int i = 0; i < 7; i++) {

    if (i != 2 && values[i] == targetLineValue) sensor2Only = false;

  }

  if (!upperRouteLocked) {

    if (centerOnly) {

      upperEntryStage = 1;

    } else if (upperEntryStage == 1 && middleThree) {

      upperEntryStage = 2;

    } else if (upperEntryStage == 2 && alternating135) {

      upperRouteLocked = true;

      upperEntryStage = 0;

      integral = 0;

      lastError = 0;

      Serial.println("Upper path locked: following upper branch until sensor[2] only at exit");

    } else if (upperEntryStage > 0 && !middleThree && !alternating135 && !centerOnly) {

      // Keep the sequence state only across the expected patterns.

      // If a different pattern appears before the next stage, restart safely.

      if (upperEntryStage == 2) upperEntryStage = 0;

    }

  } else if (sensor2Only) {

    upperRouteLocked = false;

    integral = 0;

    lastError = 0;

    Serial.println("Upper-loop exit detected (sensor[2] only); normal PID resumed");

  }

  // First circle: detect the junction (middle three sensors on the active line).

  // Only the first junction can trigger this fixed LEFT maneuver.

  bool junctionPattern = middleThree;

  if (!junctionPattern) {

    // Do not allow a middle-three pattern present at START to trigger a turn.

    // First move off the starting pattern, then arm the first-circle detector.

    if (junctionClearStart == 0) junctionClearStart = millis();

    if (millis() - junctionClearStart >= 100) {

      junctionArmed = true;

      firstCircleJunctionReady = true;

    }

  } else {

    junctionClearStart = 0;

  }

  if (junctionPattern && junctionArmed) {

    junctionArmed = false;

    junctionCount++;

    Serial.printf("Junction occurrence %d\n", junctionCount);

    if (firstCircleJunctionReady && !firstCircleTurnDone) {

      firstCircleTurnActive = true;

      firstCircleTurnStart = millis();

      firstCircleTurnDone = true;

      integral = 0;

      lastError = 0;

      Serial.println("FIRST CIRCLE: locked LEFT turn started");

    }

  }

  // Keep the first-circle turn direction locked; PID cannot reverse it.

  if (firstCircleTurnActive &&

      millis() - firstCircleTurnStart >= FIRST_CIRCLE_LEFT_TURN_MS) {

    firstCircleTurnActive = false;

    integral = 0;

    lastError = 0;

    Serial.println("FIRST CIRCLE: left-turn phase complete; PID resumed");

  }

  // During the locked turn, bypass line-loss recovery and PID entirely so

  // a temporary all-white/all-black reading cannot reverse the maneuver.

  if (firstCircleTurnActive) {

    int leftSpeed = constrain(baseSpeed + 50, 0, 255);  // correction = -50

    int rightSpeed = constrain(baseSpeed - 50, 0, 255);

    setMotor(leftSpeed, rightSpeed);

    delay(8);

    return;

  }

  int count = 0;

  float sum = 0;

  const float weights[7] = {-12, -8, -4, 0, 4, 8, 12};

  for (int i = 0; i < 7; i++) {

    if (values[i] == targetLineValue) {

      sum += weights[i];

      count++;

    }

  }

  float error;

  if (count > 0) {

    // User-confirmed motor convention: positive correction speeds up the

    // left motor and slows the right motor, steering the robot right.

    // A positive weighted sensor position means the line is on the right.

    error = sum / count;

    lineWasLost = false;

    lineLossStart = 0;

  } else {

    if (!lineWasLost) {

      lineWasLost = true;

      lineLossStart = millis();

      Serial.println("LINE LOST - recovery search");

    }

    // With the user's motor convention, positive error means the line was

    // to the right. Search toward the last known line position.

    if (lastError > 0) {

      setMotor(-searchSpeed, searchSpeed); // search right

    } else {

      setMotor(searchSpeed, -searchSpeed); // search left

    }

    if (millis() - lineLossStart > LINE_LOSS_TIMEOUT_MS) {

      setMotor(0, 0);

      Serial.println("Line not found before timeout; motors stopped");

      delay(10);

      return;

    }

    delay(5);

    return;

  }

  integral += error;

  integral = constrain(integral, -50.0f, 50.0f);

  float derivative = error - lastError;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  // A fixed negative correction guarantees LEFT during the first-circle phase.

  // Outside that phase, preserve the existing upper-route steering bias.

  if (firstCircleTurnActive) {

    correction = -50.0f;

  } else if (upperRouteLocked) {

    correction -= 12.0f;

  }

  correction = constrain(correction, -50.0f, 50.0f);

  // User-confirmed motor equation.

  int leftSpeed = baseSpeed - (int)correction;

  int rightSpeed = baseSpeed + (int)correction;

  leftSpeed = constrain(leftSpeed, 0, 255);

  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotor(leftSpeed, rightSpeed);

  Serial.printf(

    "S: %d %d %d %d %d %d %d | E: %.2f | Kp: %.2f Ki: %.2f Kd: %.2f | L:%d R:%d | J:%d | Mode:%s\n",

    values[0], values[1], values[2], values[3], values[4], values[5], values[6],

    error, Kp, Ki, Kd, leftSpeed, rightSpeed, junctionCount,

    followWhite ? "WHITE" : (whiteDetectionEnabled ? "BLACK (WHITE DETECT ARMED)" : "BLACK")

  );

  lastError = error;

  delay(8);

}
