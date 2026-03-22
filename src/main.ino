*/
==========================================
    UVRobot – Final Version
    ESP32 Web Control + Auto Navigation
    + Path Recording / Replay
    + Session Log Tab
    + Return to Start
  ==========================================

  --- WIRING ---
  DRV8825 #1 (Left Motor)   STEP -> GPIO 19,  DIR -> GPIO 18
  DRV8825 #2 (Right Motor)  STEP -> GPIO 25,  DIR -> GPIO 26
  UV Light                  GPIO 2
  Ultrasonic                Trig -> GPIO 14,  Echo -> GPIO 27
  VCNL4040 Left             SDA -> GPIO 21,   SCL -> GPIO 22
  VCNL4040 Right            SDA -> GPIO 16,   SCL -> GPIO 17

  --- MODES ---
  MANUAL  : 4 arrow buttons control robot
  AUTO    : Robot navigates itself using sensors
  RECORD  : Record your manual moves, replay them
  LOG     : Session stats table
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VCNL4040.h>

// ==========================================
// WiFi
// ==========================================
const char* ssid     = "UVRobot-AP";
const char* password = "uvrobot123";
WebServer server(80);

// ==========================================
// Motor Pins
// ==========================================
#define STEP_L 19
#define DIR_L  18
#define STEP_R 25
#define DIR_R  26

// ==========================================
// UV Light
// ==========================================
#define UV_PIN 2

// ==========================================
// Proximity Sensors (VCNL4040)
// ==========================================
Adafruit_VCNL4040 vcnlLeft;
Adafruit_VCNL4040 vcnlRight;
TwoWire I2C_LEFT  = TwoWire(0);
TwoWire I2C_RIGHT = TwoWire(1);

// ==========================================
// Ultrasonic (Front only)
// ==========================================
const int TRIG_PIN = 14;
const int ECHO_PIN = 27;

#define VCNL_THRESHOLD 5

// Motor speed -- can be changed live from website
int STEP_INTERVAL = 2000;

// ==========================================
// Robot State
// ==========================================
bool isAutoMode = false;

enum MoveState { STOP, FORWARD, BACKWARD, LEFT, RIGHT };
MoveState currentMove = STOP;

// ==========================================
// Motor Timer
// ==========================================
unsigned long lastStep = 0;
bool stepState = false;

// ==========================================
// Path Recording & Replay (RECORD mode)
// ==========================================
struct MoveRecord {
  MoveState     move;
  unsigned long duration;
};

#define MAX_RECORDS 400
MoveRecord    path[MAX_RECORDS];
int           recordCount  = 0;
bool          isRecording  = false;
bool          isReplaying  = false;
unsigned long moveStartTime = 0;
int           replayIndex  = 0;
unsigned long replayStart  = 0;

// ==========================================
// Return to Start (AUTO mode path)
// ==========================================
#define MAX_RETURN 400
MoveRecord    returnPath[MAX_RETURN];
int           returnCount     = 0;
bool          isReturning     = false;
int           returnIndex     = 0;
unsigned long returnStepStart = 0;

// ==========================================
// Session Log (AUTO mode stats)
// ==========================================
unsigned long sessionStart  = 0;
unsigned long uvOnTotal     = 0;
unsigned long uvOnStart     = 0;
bool          uvWasOn       = false;
int           obstacleCount = 0;

// ==========================================
// Record Move Helper (RECORD mode)
// ==========================================
void recordMove(MoveState newMove) {
  if (!isRecording) {
    currentMove = newMove;
    return;
  }
  unsigned long now = millis();
  if (currentMove != STOP && recordCount < MAX_RECORDS) {
    path[recordCount].move     = currentMove;
    path[recordCount].duration = now - moveStartTime;
    recordCount++;
  }
  moveStartTime = now;
  currentMove   = newMove;
}

// ==========================================
// Step Motors (original working code)
// ==========================================
void stepMotors(bool leftFwd, bool rightFwd) {
  digitalWrite(DIR_L, leftFwd);
  digitalWrite(DIR_R, rightFwd);
  unsigned long now = micros();
  if (now - lastStep >= (unsigned long)STEP_INTERVAL) {
    lastStep  = now;
    stepState = !stepState;
    digitalWrite(STEP_L, stepState);
    digitalWrite(STEP_R, stepState);
  }
}

// ==========================================
// Run Motors
// ==========================================
void runMotors() {
  switch (currentMove) {
    case FORWARD:  digitalWrite(UV_PIN, HIGH); stepMotors(true,  true);  break;
    case BACKWARD: digitalWrite(UV_PIN, HIGH); stepMotors(false, false); break;
    case LEFT:     digitalWrite(UV_PIN, HIGH); stepMotors(false, true);  break;
    case RIGHT:    digitalWrite(UV_PIN, HIGH); stepMotors(true,  false); break;
    case STOP:     digitalWrite(UV_PIN, LOW);  break;
  }
}

// ==========================================
// Ultrasonic Distance
// ==========================================
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int dist = duration * 0.034 / 2;
  if (dist == 0) dist = 400;
  return dist;
}

// ==========================================
// Auto Mode
// Far zone  (>15cm) = normal steering
// Close zone (<15cm) = back up then random turn
// Records every 100ms snapshot for return-to-start
// ==========================================
void runAutoMode() {
  static unsigned long lastDecision   = 0;
  static unsigned long backingUpUntil = 0;
  static unsigned long turningUntil   = 0;
  static MoveState     turnDirection  = LEFT;
  static unsigned long lastRecord     = 0;

  if (isReturning) return;

  unsigned long now = millis();

  // Record current move every 100ms — always, regardless of direction
  if (now - lastRecord >= 100 && currentMove != STOP) {
    if (returnCount < MAX_RETURN) {
      returnPath[returnCount].move     = currentMove;
      returnPath[returnCount].duration = 100;
      returnCount++;
    }
    lastRecord = now;
  }

  // Only run sensor decision every 150ms
  if (now - lastDecision < 150) return;
  lastDecision = now;

  int  frontDist    = getDistance();
  int  leftVal      = vcnlLeft.getProximity();
  int  rightVal     = vcnlRight.getProximity();
  bool leftBlocked  = leftVal  > VCNL_THRESHOLD;
  bool rightBlocked = rightVal > VCNL_THRESHOLD;

  if (now < backingUpUntil) { currentMove = BACKWARD; return; }
  if (now < turningUntil)   { currentMove = turnDirection; return; }

  MoveState newMove;

  if (frontDist < 15) {
    obstacleCount++;
    turnDirection  = (random(2) == 0) ? LEFT : RIGHT;
    backingUpUntil = now + 600;
    turningUntil   = backingUpUntil + 400;
    newMove        = BACKWARD;
  }
  else if (leftBlocked)  newMove = RIGHT;
  else if (rightBlocked) newMove = LEFT;
  else                   newMove = FORWARD;

  // Track UV on time
  if (newMove != STOP) {
    if (!uvWasOn) { uvOnStart = now; uvWasOn = true; }
  } else {
    if (uvWasOn) { uvOnTotal += now - uvOnStart; uvWasOn = false; }
  }

  currentMove = newMove;
}

// ==========================================
// Web Page
// ==========================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>UVRobot</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Share+Tech+Mono&display=swap');

:root {
  --uv: #b44fff;
  --uv2: #d580ff;
  --uv3: #7a00cc;
  --bg: #07000f;
  --panel: #0e001c;
  --border: #2d0057;
  --text: #e0c8ff;
  --red: #ff3355;
  --green: #00ffaa;
  --yellow: #ffd600;
}

* { box-sizing: border-box; margin: 0; padding: 0; }

body {
  background: var(--bg);
  color: var(--text);
  font-family: 'Share Tech Mono', monospace;
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: flex-start;
  padding: 16px;
  overflow-x: hidden;
}

body::after {
  content: '';
  position: fixed; inset: 0;
  background: repeating-linear-gradient(0deg, transparent, transparent 3px, rgba(0,0,0,0.08) 3px, rgba(0,0,0,0.08) 4px);
  pointer-events: none;
  z-index: 100;
}

body::before {
  content: '';
  position: fixed; inset: 0;
  background: radial-gradient(ellipse 70% 50% at 50% 0%, #1a003344 0%, transparent 70%);
  pointer-events: none;
}

.wrap {
  width: 100%;
  max-width: 400px;
  display: flex;
  flex-direction: column;
  gap: 14px;
  position: relative;
  z-index: 1;
}

/* HEADER */
.header {
  text-align: center;
  padding: 14px 20px 10px;
  border: 1px solid var(--border);
  background: var(--panel);
  clip-path: polygon(0 0, calc(100% - 14px) 0, 100% 14px, 100% 100%, 14px 100%, 0 calc(100% - 14px));
}
.header h1 {
  font-family: 'Orbitron', sans-serif;
  font-size: 1.7rem; font-weight: 900; letter-spacing: 5px;
  color: var(--uv2);
  text-shadow: 0 0 18px var(--uv), 0 0 40px var(--uv3);
}
.header .tagline { font-size: 0.65rem; letter-spacing: 3px; color: var(--uv3); margin-top: 3px; }

/* STATUS BAR */
.statusbar { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; }
.stat {
  background: var(--panel); border: 1px solid var(--border);
  padding: 8px 10px; display: flex; flex-direction: column; gap: 3px;
}
.stat-label { font-size: 0.6rem; letter-spacing: 2px; color: var(--uv3); }
.stat-val { font-family: 'Orbitron', sans-serif; font-size: 0.75rem; font-weight: 700; color: var(--uv2); }
.green  { color: #00ffaa !important; text-shadow: 0 0 8px #00ffaa; }
.red    { color: #ff3355 !important; text-shadow: 0 0 8px #ff3355; }
.yellow { color: #ffd600 !important; text-shadow: 0 0 8px #ffd600; }
@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }
.pulse { animation: pulse 1s ease-in-out infinite; }

/* TABS */
.mode-tabs {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr 1fr;
  border: 1px solid var(--border);
  background: var(--panel);
  overflow: hidden;
}
.tab {
  padding: 12px 4px;
  text-align: center;
  font-family: 'Orbitron', sans-serif;
  font-size: 0.55rem; font-weight: 700; letter-spacing: 1px;
  cursor: pointer; border: none; background: transparent; color: var(--uv3);
  border-right: 1px solid var(--border);
  display: flex; flex-direction: column; align-items: center; gap: 4px;
  touch-action: manipulation;
  -webkit-tap-highlight-color: rgba(180,79,255,0.2);
  transition: all 0.2s;
}
.tab:last-child { border-right: none; }
.tab .icon { font-size: 1rem; }
.tab.active { background: var(--uv3); color: white; }
.tab.rec-active { background: #550000 !important; animation: pulse 1s infinite; }

/* TAB PANELS */
.tab-panel { display: none; flex-direction: column; gap: 14px; }
.tab-panel.show { display: flex; }

/* D-PAD */
.dpad-section {
  background: var(--panel); border: 1px solid var(--border);
  padding: 20px; display: flex; flex-direction: column; align-items: center; gap: 10px;
}
.dpad-label { font-size: 0.65rem; letter-spacing: 3px; color: var(--uv3); align-self: flex-start; }
.dpad-row { display: flex; gap: 10px; align-items: center; }
.btn {
  width: 78px; height: 78px;
  background: #150025; border: 2px solid var(--border);
  color: var(--text); font-size: 1.8rem; cursor: pointer;
  display: flex; align-items: center; justify-content: center;
  clip-path: polygon(8px 0%, calc(100% - 8px) 0%, 100% 8px, 100% calc(100% - 8px), calc(100% - 8px) 100%, 8px 100%, 0% calc(100% - 8px), 0% 8px);
  transition: all 0.1s; touch-action: manipulation;
  -webkit-tap-highlight-color: transparent; user-select: none;
}
.btn:active, .btn.pressed {
  background: var(--uv3); border-color: var(--uv2);
  color: white; transform: scale(0.92); box-shadow: 0 0 20px var(--uv);
}
.btn-mid {
  width: 52px; height: 52px; background: #0d001a; border: 1px solid var(--border);
  display: flex; align-items: center; justify-content: center;
  font-size: 0.55rem; letter-spacing: 1px; color: var(--uv3);
  clip-path: polygon(4px 0%, calc(100% - 4px) 0%, 100% 4px, 100% calc(100% - 4px), calc(100% - 4px) 100%, 4px 100%, 0% calc(100% - 4px), 0% 4px);
}

/* AUTO PANEL */
.auto-panel {
  background: var(--panel); border: 1px solid var(--border);
  padding: 16px; display: flex; flex-direction: column; gap: 10px;
}
.auto-panel-label { font-size: 0.65rem; letter-spacing: 3px; color: var(--uv3); }
.return-btn {
  height: 58px; width: 100%;
  border: 2px solid var(--green); background: #0d001a; color: var(--green);
  font-family: 'Orbitron', sans-serif; font-size: 0.7rem; font-weight: 700; letter-spacing: 2px;
  cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 8px;
  clip-path: polygon(6px 0%, calc(100% - 6px) 0%, 100% 6px, 100% calc(100% - 6px), calc(100% - 6px) 100%, 6px 100%, 0% calc(100% - 6px), 0% 6px);
  transition: all 0.15s; touch-action: manipulation;
}
.return-btn:disabled { opacity: 0.3; pointer-events: none; }
.return-btn:active { background: var(--green); color: #000; }
.auto-btns-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.auto-start-btn, .auto-stop-btn {
  height: 58px;
  font-family: 'Orbitron', sans-serif; font-size: 0.7rem; font-weight: 700; letter-spacing: 2px;
  cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 8px;
  clip-path: polygon(6px 0%, calc(100% - 6px) 0%, 100% 6px, 100% calc(100% - 6px), calc(100% - 6px) 100%, 6px 100%, 0% calc(100% - 6px), 0% 6px);
  transition: all 0.15s; touch-action: manipulation;
}
.auto-start-btn { border: 2px solid var(--green); background: #0d001a; color: var(--green); }
.auto-start-btn:active { background: var(--green); color: #000; }
.auto-stop-btn  { border: 2px solid var(--red);   background: #0d001a; color: var(--red); }
.auto-stop-btn:active  { background: var(--red);   color: #000; }
.auto-start-btn:disabled, .auto-stop-btn:disabled { opacity: 0.3; pointer-events: none; }

/* RECORD CONTROLS */
.rec-controls {
  display: grid; grid-template-columns: 1fr 1fr; gap: 10px;
  background: var(--panel); border: 1px solid var(--border); padding: 14px;
}
.rec-btn {
  height: 52px; border: 2px solid var(--border); background: #0d001a; color: var(--text);
  font-family: 'Orbitron', sans-serif; font-size: 0.65rem; font-weight: 700; letter-spacing: 1px;
  cursor: pointer; display: flex; align-items: center; justify-content: center; gap: 6px;
  clip-path: polygon(6px 0%, calc(100% - 6px) 0%, 100% 6px, 100% calc(100% - 6px), calc(100% - 6px) 100%, 6px 100%, 0% calc(100% - 6px), 0% 6px);
  transition: all 0.15s; touch-action: manipulation;
}
.rec-btn.start  { border-color: var(--red);   color: var(--red); }
.rec-btn.replay { border-color: var(--green);  color: var(--green); grid-column: span 2; }
.rec-btn:disabled { opacity: 0.3; pointer-events: none; }

/* SESSION LOG */
.session-panel {
  background: var(--panel); border: 1px solid var(--border);
  padding: 16px; display: flex; flex-direction: column; gap: 14px;
}
.session-title {
  font-family: 'Orbitron', sans-serif; font-size: 0.75rem; font-weight: 900;
  letter-spacing: 4px; color: var(--uv2); text-align: center;
  text-shadow: 0 0 10px var(--uv);
}
.session-table { width: 100%; border-collapse: collapse; }
.session-table tr { border-bottom: 1px solid var(--border); }
.session-table tr:last-child { border-bottom: none; }
.session-table td { padding: 12px 6px; }
.td-label { font-size: 0.65rem; letter-spacing: 2px; color: var(--uv3); width: 60%; }
.td-val   { font-family: 'Orbitron', sans-serif; font-size: 0.85rem; font-weight: 700; color: var(--uv2); text-align: right; }
.session-note { font-size: 0.6rem; color: var(--uv3); text-align: center; letter-spacing: 1px; }
.refresh-btn {
  height: 44px; width: 100%;
  border: 1px solid var(--border); background: #0d001a; color: var(--uv3);
  font-family: 'Orbitron', sans-serif; font-size: 0.65rem; font-weight: 700; letter-spacing: 2px;
  cursor: pointer; touch-action: manipulation; transition: all 0.2s;
}
.refresh-btn:active { border-color: var(--uv); color: var(--uv2); }

/* SPEED SLIDER */
.speed-panel {
  background: var(--panel); border: 1px solid var(--border);
  padding: 14px 16px; display: flex; flex-direction: column; gap: 10px;
}
.speed-top { display: flex; justify-content: space-between; align-items: center; }
.speed-label { font-size: 0.65rem; letter-spacing: 3px; color: var(--uv3); }
.speed-val { font-family: 'Orbitron', sans-serif; font-size: 0.85rem; font-weight: 700; color: var(--uv2); }
.speed-ticks { display: flex; justify-content: space-between; font-size: 0.55rem; color: var(--uv3); }
input[type=range] {
  -webkit-appearance: none; width: 100%; height: 4px;
  background: linear-gradient(to right, var(--uv), var(--uv2));
  border-radius: 2px; outline: none;
}
input[type=range]::-webkit-slider-thumb {
  -webkit-appearance: none; width: 22px; height: 22px; border-radius: 50%;
  background: var(--uv2); border: 2px solid var(--uv);
  box-shadow: 0 0 8px var(--uv); cursor: pointer;
}

/* LOG BAR */
.log {
  background: var(--panel); border: 1px solid var(--border);
  padding: 10px 14px; height: 52px; overflow: hidden;
  display: flex; flex-direction: column; justify-content: flex-end; gap: 2px;
}
.log-line { font-size: 0.65rem; color: var(--uv3); }
.log-line.new { color: var(--uv2); }

.footer { text-align: center; font-size: 0.6rem; color: #2d0057; letter-spacing: 1px; padding-bottom: 8px; }
</style>
</head>
<body>
<div class="wrap">

  <!-- HEADER -->
  <div class="header">
    <h1>UVROBOT</h1>
    <div class="tagline">UV STERILIZATION SYSTEM</div>
  </div>

  <!-- STATUS BAR -->
  <div class="statusbar">
    <div class="stat">
      <div class="stat-label">MODE</div>
      <div class="stat-val green" id="statMode">MANUAL</div>
    </div>
    <div class="stat">
      <div class="stat-label">UV LIGHT</div>
      <div class="stat-val" id="statUV">STANDBY</div>
    </div>
    <div class="stat">
      <div class="stat-label">DIRECTION</div>
      <div class="stat-val" id="statDir">STOP</div>
    </div>
  </div>

  <!-- TABS -->
  <div class="mode-tabs">
    <button class="tab active" id="tabManual" onclick="setTab('manual')">
      <span class="icon">🕹</span>MANUAL
    </button>
    <button class="tab" id="tabAuto" onclick="setTab('auto')">
      <span class="icon">🤖</span>AUTO
    </button>
    <button class="tab" id="tabRecord" onclick="setTab('record')">
      <span class="icon">⏺</span>RECORD
    </button>
    <button class="tab" id="tabLog" onclick="setTab('log')">
      <span class="icon">📊</span>LOG
    </button>
  </div>

  <!-- MANUAL PANEL -->
  <div class="tab-panel show" id="panelManual">
    <div class="dpad-section">
      <div class="dpad-label">// DIRECTIONAL CONTROL</div>
      <div class="dpad-row">
        <div style="width:78px"></div>
        <button class="btn" id="btnF"
          onmousedown="press('F')" onmouseup="release('F')"
          ontouchstart="press('F')" ontouchend="release('F')">&#9650;</button>
        <div style="width:78px"></div>
      </div>
      <div class="dpad-row">
        <button class="btn" id="btnL"
          onmousedown="press('L')" onmouseup="release('L')"
          ontouchstart="press('L')" ontouchend="release('L')">&#9664;</button>
        <div class="btn-mid">MOVE</div>
        <button class="btn" id="btnR"
          onmousedown="press('R')" onmouseup="release('R')"
          ontouchstart="press('R')" ontouchend="release('R')">&#9654;</button>
      </div>
      <div class="dpad-row">
        <div style="width:78px"></div>
        <button class="btn" id="btnB"
          onmousedown="press('B')" onmouseup="release('B')"
          ontouchstart="press('B')" ontouchend="release('B')">&#9660;</button>
        <div style="width:78px"></div>
      </div>
    </div>
  </div>

  <!-- AUTO PANEL -->
  <div class="tab-panel" id="panelAuto">
    <div class="auto-panel">
      <div class="auto-panel-label">// AUTO NAVIGATION</div>
      <div class="auto-btns-row">
        <button class="auto-start-btn" id="btnStartAuto" onclick="startAuto()">
          &#9654; START
        </button>
        <button class="auto-stop-btn" id="btnStopAuto" onclick="stopAuto()" disabled>
          &#9209; STOP
        </button>
      </div>
      <button class="return-btn" id="btnReturn" onclick="returnToStart()" disabled>
        &#127968; RETURN TO START
      </button>
    </div>
  </div>

  <!-- RECORD PANEL -->
  <div class="tab-panel" id="panelRecord">
    <div class="dpad-section">
      <div class="dpad-label">// DRIVE TO RECORD</div>
      <div class="dpad-row">
        <div style="width:78px"></div>
        <button class="btn" id="btnRF"
          onmousedown="press('F')" onmouseup="release('F')"
          ontouchstart="press('F')" ontouchend="release('F')">&#9650;</button>
        <div style="width:78px"></div>
      </div>
      <div class="dpad-row">
        <button class="btn" id="btnRL"
          onmousedown="press('L')" onmouseup="release('L')"
          ontouchstart="press('L')" ontouchend="release('L')">&#9664;</button>
        <div class="btn-mid">REC</div>
        <button class="btn" id="btnRR"
          onmousedown="press('R')" onmouseup="release('R')"
          ontouchstart="press('R')" ontouchend="release('R')">&#9654;</button>
      </div>
      <div class="dpad-row">
        <div style="width:78px"></div>
        <button class="btn" id="btnRB"
          onmousedown="press('B')" onmouseup="release('B')"
          ontouchstart="press('B')" ontouchend="release('B')">&#9660;</button>
        <div style="width:78px"></div>
      </div>
    </div>
    <div class="rec-controls">
      <button class="rec-btn start" id="recStart" onclick="recStart()">&#9210; START REC</button>
      <button class="rec-btn" id="recStop" onclick="recStop()" disabled>&#9209; STOP REC</button>
      <button class="rec-btn replay" id="recReplay" onclick="recReplay()" disabled>&#9654; REPLAY PATH</button>
    </div>
  </div>

  <!-- LOG PANEL -->
  <div class="tab-panel" id="panelLog">
    <div class="session-panel">
      <div class="session-title">SESSION LOG</div>
      <table class="session-table">
        <tr>
          <td class="td-label">DURATION</td>
          <td class="td-val" id="logDuration">--</td>
        </tr>
        <tr>
          <td class="td-label">UV ON TIME</td>
          <td class="td-val" id="logUV">--</td>
        </tr>
        <tr>
          <td class="td-label">OBSTACLES AVOIDED</td>
          <td class="td-val" id="logObs">--</td>
        </tr>
        <tr>
          <td class="td-label">EST. DISTANCE</td>
          <td class="td-val" id="logDist">--</td>
        </tr>
      </table>
      <div class="session-note">Run AUTO mode then press REFRESH</div>
      <button class="refresh-btn" onclick="refreshLog()">REFRESH</button>
    </div>
  </div>

  <!-- SPEED SLIDER (always visible) -->
  <div class="speed-panel">
    <div class="speed-top">
      <div class="speed-label">// MOTOR SPEED</div>
      <div class="speed-val" id="speedVal">MED</div>
    </div>
    <input type="range" id="speedSlider" min="1" max="5" value="3" step="1" oninput="setSpeed(this.value)">
    <div class="speed-ticks">
      <span>SLOW</span><span>.</span><span>MED</span><span>.</span><span>FAST</span>
    </div>
  </div>

  <!-- LOG BAR -->
  <div class="log" id="log">
    <div class="log-line">[ SYS ] UVRobot online</div>
    <div class="log-line">[ SYS ] Awaiting command...</div>
  </div>

  <div class="footer">ESP32 @ 192.168.4.1 | UVRobot-AP / uvrobot123</div>
</div>

<script>
  var mode        = 'manual';
  var isRecording = false;
  var hasRecording = false;

  var SPEED_MAP    = { 1:3000, 2:2500, 3:2000, 4:1500, 5:1000 };
  var SPEED_LABELS = { 1:'SLOW', 2:'MED-', 3:'MED', 4:'FAST', 5:'MAX' };

  function setTab(t) {
    var ids = ['manual','auto','record','log'];
    ids.forEach(function(id) {
      var tab   = document.getElementById('tab'   + id.charAt(0).toUpperCase() + id.slice(1));
      var panel = document.getElementById('panel' + id.charAt(0).toUpperCase() + id.slice(1));
      tab.className   = 'tab'       + (id === t ? ' active' : '') + (id === 'record' && isRecording ? ' rec-active' : '');
      panel.className = 'tab-panel' + (id === t ? ' show'   : '');
    });

    if (t === 'auto') {
      // Just show the panel — user must press START
      if (mode === 'auto') {
        // already running, do nothing
      } else {
        mode = 'idle_auto';
        document.getElementById('btnStartAuto').disabled = false;
        document.getElementById('btnStopAuto').disabled  = true;
        updateStatMode();
        addLog('[ AUTO ] Press START to begin');
      }
    } else if (t !== 'auto' && mode === 'auto') {
      // Leaving auto tab while running - stop the robot
      mode = 'manual';
      send('/cmd/mode/manual');
      document.getElementById('btnStartAuto').disabled = false;
      document.getElementById('btnStopAuto').disabled  = true;
      document.getElementById('btnReturn').disabled    = false;
      updateStatMode();
      addLog('[ AUTO ] Stopped -- return available');
    } else if (t === 'manual') {
      mode = 'manual';
      send('/cmd/mode/manual');
      updateStatMode();
    } else if (t === 'record') {
      mode = 'record';
      send('/cmd/mode/manual');
      updateStatMode();
    }

    if (t === 'log') refreshLog();
  }

  function updateStatMode() {
    var el = document.getElementById('statMode');
    el.className = 'stat-val';
    if (mode === 'manual')    { el.textContent = 'MANUAL'; el.classList.add('green'); }
    if (mode === 'auto')      { el.textContent = 'AUTO';   el.classList.add('yellow'); }
    if (mode === 'record')    { el.textContent = 'RECORD'; el.classList.add('red'); }
    if (mode === 'idle_auto') { el.textContent = 'AUTO';   el.classList.add('yellow'); }
  }

  var DIR_MAP    = { F:'move/forward', B:'move/backward', L:'move/left', R:'move/right' };
  var DIR_LABELS = { F:'FWD', B:'REV', L:'LEFT', R:'RIGHT' };

  function press(dir) {
    ['F','B','L','R'].forEach(function(d) {
      var b1 = document.getElementById('btn'  + d);
      var b2 = document.getElementById('btnR' + d);
      if (b1) b1.classList.toggle('pressed', d === dir);
      if (b2) b2.classList.toggle('pressed', d === dir);
    });
    document.getElementById('statUV').textContent  = 'ACTIVE';
    document.getElementById('statUV').className    = 'stat-val green pulse';
    document.getElementById('statDir').textContent = DIR_LABELS[dir];
    document.getElementById('statDir').className   = 'stat-val yellow';
    send('/cmd/' + DIR_MAP[dir]);
    addLog('[ MOVE ] ' + DIR_LABELS[dir]);
  }

  function release(dir) {
    var b1 = document.getElementById('btn'  + dir);
    var b2 = document.getElementById('btnR' + dir);
    if (b1) b1.classList.remove('pressed');
    if (b2) b2.classList.remove('pressed');
    document.getElementById('statUV').textContent  = 'STANDBY';
    document.getElementById('statUV').className    = 'stat-val';
    document.getElementById('statDir').textContent = 'STOP';
    document.getElementById('statDir').className   = 'stat-val';
    send('/cmd/stop');
  }

  function recStart() {
    isRecording  = true;
    hasRecording = false;
    document.getElementById('recStart').disabled  = true;
    document.getElementById('recStop').disabled   = false;
    document.getElementById('recReplay').disabled = true;
    document.getElementById('tabRecord').classList.add('rec-active');
    document.getElementById('statMode').textContent = 'REC';
    send('/cmd/record/start');
    addLog('[ REC ] Recording started...');
  }

  function recStop() {
    isRecording  = false;
    hasRecording = true;
    document.getElementById('recStart').disabled  = false;
    document.getElementById('recStop').disabled   = true;
    document.getElementById('recReplay').disabled = false;
    document.getElementById('tabRecord').classList.remove('rec-active');
    document.getElementById('statMode').textContent = 'RECORD';
    send('/cmd/record/stop');
    send('/cmd/stop');
    addLog('[ REC ] Saved! Press REPLAY to run.');
  }

  function recReplay() {
    send('/cmd/replay');
    addLog('[ REC ] Replaying path...');
  }

  function startAuto() {
    mode = 'auto';
    send('/cmd/mode/auto');
    document.getElementById('btnStartAuto').disabled = true;
    document.getElementById('btnStopAuto').disabled  = false;
    document.getElementById('btnReturn').disabled    = true;
    document.getElementById('statMode').className    = 'stat-val yellow';
    document.getElementById('statMode').textContent  = 'AUTO';
    document.getElementById('statUV').textContent    = 'ACTIVE';
    document.getElementById('statUV').className      = 'stat-val green pulse';
    addLog('[ AUTO ] Navigating...');
  }

  function stopAuto() {
    mode = 'idle_auto';
    send('/cmd/mode/manual');
    document.getElementById('btnStartAuto').disabled = false;
    document.getElementById('btnStopAuto').disabled  = true;
    document.getElementById('btnReturn').disabled    = false;
    document.getElementById('statMode').className    = 'stat-val yellow';
    document.getElementById('statMode').textContent  = 'AUTO';
    document.getElementById('statDir').textContent   = 'STOP';
    document.getElementById('statDir').className     = 'stat-val';
    document.getElementById('statUV').textContent    = 'STANDBY';
    document.getElementById('statUV').className      = 'stat-val';
    addLog('[ AUTO ] Stopped -- press RETURN or START again');
  }

  function returnToStart() {
    send('/cmd/return');
    document.getElementById('btnReturn').disabled   = true;
    document.getElementById('btnStopAuto').disabled = true;
    document.getElementById('statDir').textContent  = 'RETURN';
    document.getElementById('statDir').className    = 'stat-val green pulse';
    addLog('[ NAV ] Returning to start...');
  }

  function fmtTime(secs) {
    var m = Math.floor(secs / 60);
    var s = secs % 60;
    return (m > 0 ? m + 'm ' : '') + s + 's';
  }

  function refreshLog() {
    fetch('/cmd/session/status')
      .then(function(r) { return r.json(); })
      .then(function(data) {
        document.getElementById('logDuration').textContent = fmtTime(data.duration);
        document.getElementById('logUV').textContent       = fmtTime(data.uv);
        document.getElementById('logObs').textContent      = data.obstacles + ' avoided';
        document.getElementById('logDist').textContent     = '~' + data.dist + ' cm';
        document.getElementById('btnReturn').disabled      = (data.canReturn === 0);
      })
      .catch(function() { addLog('[ ERR ] Could not fetch log data'); });
  }

  function setSpeed(val) {
    var interval = SPEED_MAP[val];
    document.getElementById('speedVal').textContent = SPEED_LABELS[val];
    send('/cmd/speed/' + interval);
    addLog('[ SPD ] ' + SPEED_LABELS[val] + ' (' + interval + 'us)');
  }

  function send(url) { fetch(url).catch(function(){}); }

  function addLog(msg) {
    var log   = document.getElementById('log');
    var lines = log.querySelectorAll('.log-line');
    lines.forEach(function(l) { l.classList.remove('new'); });
    if (lines.length >= 2) lines[0].remove();
    var el = document.createElement('div');
    el.className   = 'log-line new';
    el.textContent = msg;
    log.appendChild(el);
  }
</script>
</body>
</html>
)rawliteral";

// ==========================================
// Routes
// ==========================================
void setupRoutes() {
  server.on("/", []() {
    server.send(200, "text/html", INDEX_HTML);
  });

  server.on("/cmd/move/forward",  []() { recordMove(FORWARD);  server.send(200, "text/plain", "OK"); });
  server.on("/cmd/move/backward", []() { recordMove(BACKWARD); server.send(200, "text/plain", "OK"); });
  server.on("/cmd/move/left",     []() { recordMove(LEFT);     server.send(200, "text/plain", "OK"); });
  server.on("/cmd/move/right",    []() { recordMove(RIGHT);    server.send(200, "text/plain", "OK"); });
  server.on("/cmd/stop",          []() { recordMove(STOP);     server.send(200, "text/plain", "OK"); });

  server.on("/cmd/mode/auto", []() {
    isAutoMode    = true;
    currentMove   = STOP;
    sessionStart  = millis();
    uvOnTotal     = 0;
    uvWasOn       = false;
    obstacleCount = 0;
    returnCount   = 0;
    server.send(200, "text/plain", "AUTO");
  });

  server.on("/cmd/mode/manual", []() {
    isAutoMode  = false;
    currentMove = STOP;
    if (uvWasOn) { uvOnTotal += millis() - uvOnStart; uvWasOn = false; }
    server.send(200, "text/plain", "MANUAL");
  });

  // Session stats -- returns JSON
  server.on("/cmd/session/status", []() {
    unsigned long now      = millis();
    unsigned long duration = (now - sessionStart) / 1000;
    unsigned long uvSecs   = (uvWasOn ? uvOnTotal + (now - uvOnStart) : uvOnTotal) / 1000;
    unsigned long fwdMs    = 0;
    for (int i = 0; i < returnCount; i++) {
      if (returnPath[i].move == FORWARD) fwdMs += returnPath[i].duration;
    }
    int distCm = fwdMs / 100;
    String json = "{";
    json += "\"duration\":"  + String(duration)     + ",";
    json += "\"uv\":"        + String(uvSecs)        + ",";
    json += "\"obstacles\":" + String(obstacleCount) + ",";
    json += "\"dist\":"      + String(distCm)        + ",";
    json += "\"canReturn\":" + String(returnCount > 0 ? 1 : 0);
    json += "}";
    server.send(200, "application/json", json);
  });

  // Return to start -- replay AUTO path in reverse
  server.on("/cmd/return", []() {
    if (returnCount > 0) {
      isAutoMode      = false;
      isReturning     = true;
      returnIndex     = returnCount - 1;
      returnStepStart = millis();
      currentMove     = STOP;
    }
    server.send(200, "text/plain", "RETURNING");
  });

  server.on("/cmd/record/start", []() {
    recordCount   = 0;
    isRecording   = true;
    moveStartTime = millis();
    server.send(200, "text/plain", "RECORDING");
  });

  server.on("/cmd/record/stop", []() {
    isRecording = false;
    server.send(200, "text/plain", "STOPPED");
  });

  server.on("/cmd/replay", []() {
    if (recordCount > 0) {
      isReplaying = true;
      replayIndex = 0;
      replayStart = millis();
    }
    server.send(200, "text/plain", "REPLAY");
  });

  // Speed control e.g. /cmd/speed/2000
  server.onNotFound([]() {
    String url = server.uri();
    if (url.startsWith("/cmd/speed/")) {
      int val = url.substring(11).toInt();
      if (val >= 500 && val <= 5000) {
        STEP_INTERVAL = val;
        lastStep      = 0;
        Serial.print("Speed: "); Serial.println(STEP_INTERVAL);
        server.send(200, "text/plain", "SPEED:" + String(val));
      } else {
        server.send(400, "text/plain", "BAD VALUE");
      }
    } else {
      server.send(404, "text/plain", "NOT FOUND");
    }
  });
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);

  // Proximity sensors
  I2C_LEFT.begin(21, 22);
  I2C_RIGHT.begin(16, 17);
  if (!vcnlLeft.begin(0x60,  &I2C_LEFT))  Serial.println("Left VCNL not found!");
  if (!vcnlRight.begin(0x60, &I2C_RIGHT)) Serial.println("Right VCNL not found!");

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Ultrasonic sanity test
  delay(500);
  Serial.println("Testing ultrasonic...");
  for (int i = 0; i < 5; i++) {
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 30000);
    Serial.print("  dur: "); Serial.print(dur);
    Serial.print("us  dist: "); Serial.print(dur * 0.034 / 2);
    Serial.println("cm");
    delay(200);
  }

  // Motors
  pinMode(STEP_L, OUTPUT); pinMode(DIR_L, OUTPUT);
  pinMode(STEP_R, OUTPUT); pinMode(DIR_R, OUTPUT);

  // UV
  pinMode(UV_PIN, OUTPUT);
  digitalWrite(UV_PIN, LOW);

  // WiFi
  WiFi.softAP(ssid, password);
  Serial.println("UVRobot Ready!");
  Serial.println(WiFi.softAPIP());

  setupRoutes();
  server.begin();
}

// ==========================================
// Loop
// ==========================================
void loop() {
  server.handleClient();

  // Debug print every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Front: "); Serial.print(getDistance());
    Serial.print("cm  Left: "); Serial.print(vcnlLeft.getProximity());
    Serial.print("  Right: "); Serial.println(vcnlRight.getProximity());
  }

  if (isAutoMode) {
    runAutoMode();
  }

  // Return to start -- replay AUTO path in reverse
  if (isReturning) {
    if (returnIndex >= 0) {
      MoveState rev = returnPath[returnIndex].move;
      if      (rev == FORWARD)  rev = BACKWARD;
      else if (rev == BACKWARD) rev = FORWARD;
      else if (rev == LEFT)     rev = RIGHT;
      else if (rev == RIGHT)    rev = LEFT;
      currentMove = rev;
      if (millis() - returnStepStart >= returnPath[returnIndex].duration) {
        returnStepStart = millis();
        returnIndex--;
      }
    } else {
      isReturning = false;
      currentMove = STOP;
      Serial.println("Returned to start!");
    }
  }

  // Replay recorded path (RECORD mode)
  if (isReplaying) {
    if (replayIndex < recordCount) {
      currentMove = path[replayIndex].move;
      if (millis() - replayStart >= path[replayIndex].duration) {
        replayStart = millis();
        replayIndex++;
      }
    } else {
      isReplaying = false;
      currentMove = STOP;
    }
  }

  runMotors();
}
