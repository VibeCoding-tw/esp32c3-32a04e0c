#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <esp_ota_ops.h>      // For OTA partition functions
#include <esp_partition.h>    // For finding partitions

// === WebSocket & HTTP Server ===
WebSocketsServer webSocket(81);
AsyncWebServer server(80);

// Replace with your network credentials
// const char* ssid = "YOUR_WIFI_SSID";
// const char* password = "YOUR_WIFI_PASSWORD";

// === Motor A (Forward/Backward) ===
const int motorA_pwm_fwd = 6;
const int motorA_pwm_rev = 5;

// === Motor B (Left/Right) ===
const int motorB_pwm_left  = 20;
const int motorB_pwm_right = 21;

// === Motor Standby Pin ===
const int motor_stby = 7; // Set HIGH to enable motors

// LEDC channels
const int CH_A_FWD = 0;
const int CH_A_REV = 1;
const int CH_B_LEFT = 2;
const int CH_B_RIGHT = 3;
const int PWM_FREQ = 20000; // 20 kHz (Inaudible range)
const int PWM_RES = 8; // 8-bit -> duty 0-255

// --- Helper function: Send log message to serial and all connected WebSocket clients ---
void sendLogMessage(const String& message) {
  Serial.println(message);
  webSocket.broadcastTXT(message.c_str(), message.length());
}

// --- Core function: Jump back to Factory partition (for fallback on connection failure) ---
void jumpToFactory() {
  sendLogMessage("--- WiFi connection failed. JUMPING TO FACTORY PARTITION ---");

  // Find Factory partition
  const esp_partition_t* factory = esp_partition_find_first(
      ESP_PARTITION_TYPE_APP,
      ESP_PARTITION_SUBTYPE_APP_FACTORY,
      NULL);
  
  if (factory != NULL) {
      // Set Factory partition as the next boot target
      esp_err_t err = esp_ota_set_boot_partition(factory);
      if (err == ESP_OK) {
          sendLogMessage("Successfully set Factory partition as next boot target.");
          // Ensure log is sent before reboot
          delay(500); 
          ESP.restart(); // Reboot, Factory App will be loaded
      } else {
          sendLogMessage("Error setting boot partition! (" + String(esp_err_to_name(err)) + ") Rebooting anyway...");
          delay(2000);
          ESP.restart();
      }
  } else {
      sendLogMessage("FATAL: Factory partition not found! Rebooting...");
      delay(2000);
      ESP.restart();
  }
}

void setupPWM() {
  ledcSetup(CH_A_FWD, PWM_FREQ, PWM_RES);
  ledcSetup(CH_A_REV, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B_LEFT, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B_RIGHT, PWM_FREQ, PWM_RES);

  ledcAttachPin(motorA_pwm_fwd, CH_A_FWD);
  ledcAttachPin(motorA_pwm_rev, CH_A_REV);
  ledcAttachPin(motorB_pwm_left, CH_B_LEFT);
  ledcAttachPin(motorB_pwm_right, CH_B_RIGHT);
}

// === Movement Modes ===
enum DriveMode { AUTO, MANUAL };
DriveMode currentMode = MANUAL;

// HTML Content (The log display area has been removed and JS updated to use console.log)
const char index_html[] = R"rawliteral(
<!doctype html>
<html lang="zh-TW">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 遙控車：遠端控制</title>
  <style>
    :root{--bg:#0b0d11;--card:#0f1720;--accent:#3b82f6;--muted:#98a2b3}
    html,body{height:100%;margin:0;background:linear-gradient(180deg,var(--bg),#071022);color:#e6eef6;font-family:Inter,system-ui,Segoe UI,Roboto,"Noto Sans TC",sans-serif}
    /* Updated grid to fill the full height with only the viewer */
    .app{display:grid;grid-template-columns:1fr;grid-template-rows:1fr;height:100vh;padding:12px;box-sizing:border-box;position:relative}
    .viewer{background:rgba(255,255,255,0.02);border-radius:12px;padding:0;position:relative;overflow:hidden;}
    .videoFrame{width:100%;height:100%;object-fit:cover;background:#000}
    .overlay{position:absolute;left:12px;top:12px;background:rgba(0,0,0,0.45);padding:6px 8px;border-radius:8px;font-size:13px;color:var(--muted);z-index:5}
    .controls{position:absolute;top:0;left:0;width:100%;height:100%;display:flex;justify-content:space-between;align-items:flex-end;pointer-events:none}
    .stick{width:120px;height:120px;border-radius:50%;background:rgba(255,255,255,0.15);display:grid;place-items:center;position:relative;pointer-events:auto; touch-action: none;}
    .base{width:70px;height:70px;border-radius:50%;background:rgba(255,255,255,0.05);border:2px dashed rgba(255,255,255,0.03);display:grid;place-items:center}
    .knob{width:40px;height:40px;border-radius:50%;background:linear-gradient(180deg,#fff,#cbd5e1);transform:translate(-50%,-50%);position:absolute;left:50%;top:50%;box-shadow:0 6px 18px rgba(2,6,23,0.6)}
    .value{font-size:12px;color:var(--muted);text-align:center;margin-top:4px}
  </style>
</head>
<body>
  <div class="app">
    <div class="viewer">
      <img id="video" class="videoFrame" alt="遠端影像" src="" />
      <div class="overlay">影像來源: <span id="imgSource">(未設定)</span> | WS: <span id="wsStatus">未連線</span></div>
      <div class="controls">
        <div style="margin:12px; display:flex; flex-direction:column; gap:8px;">
          <div class="stick" id="stickLeft" data-role="steer"><div class="base"></div><div class="knob" id="knobLeft"></div></div>
          <div class="value">方向: <span id="valSteer">0</span></div>
        </div>
        <div style="margin:12px; display:flex; flex-direction:column; gap:8px;">
          <div class="stick" id="stickRight" data-role="throttle"><div class="base"></div><div class="knob" id="knobRight"></div></div>
          <div class="value">油門: <span id="valThrottle">0</span></div>
        </div>
      </div>
    </div>
    <!-- Remote log area removed -->
  </div>

  <script>
    class VirtualStick {
      constructor(stickEl, knobEl, onChange){
        this.el = stickEl; this.knob = knobEl; this.cb = onChange; this.max = Math.min(stickEl.clientWidth, stickEl.clientHeight)/2 - 8;
        this.center = {x: this.el.clientWidth/2, y: this.el.clientHeight/2};
        this.pointerId = null; this.pos = {x:0,y:0}; this.deadzone = 6;
        this._bind();
      }
      _bind(){
        this.el.style.touchAction = 'none';
        this.el.addEventListener('pointerdown', e=>this._start(e));
        window.addEventListener('pointermove', e=>this._move(e));
        window.addEventListener('pointerup', e=>this._end(e));
        window.addEventListener('pointercancel', e=>this._end(e));
        window.addEventListener('resize', ()=>{this.center = {x:this.el.clientWidth/2,y:this.el.clientHeight/2};this.max = Math.min(this.el.clientWidth,this.el.clientHeight)/2 - 8});
      }
      _start(e){ if(this.pointerId!==null) return; this.pointerId = e.pointerId; this.el.setPointerCapture?.(e.pointerId); this._move(e); }
      _move(e){ if(this.pointerId===null || e.pointerId!==this.pointerId) return; const rect = this.el.getBoundingClientRect(); let x = e.clientX - rect.left - rect.width/2; let y = e.clientY - rect.top - rect.height/2; const d = Math.hypot(x,y); if(d>this.max){ const r = this.max/d; x*=r; y*=r; } this.pos = {x,y}; this.knob.style.left = (50 + (x/rect.width*100))+'%'; this.knob.style.top = (50 + (y/rect.height*100))+'%'; this._fire(); }
      _end(e){ if(this.pointerId===null || e.pointerId!==this.pointerId) return; this.pointerId=null; this.pos={x:0,y:0}; this.knob.style.left='50%'; this.knob.style.top='50%'; this._fire(); }
      _fire(){ const norm = {x: Math.abs(this.pos.x) < this.deadzone ? 0 : this.pos.x/this.max, y: Math.abs(this.pos.y) < this.deadzone ? 0 : this.pos.y/this.max}; if(this.cb) this.cb(norm); }
    }

    // Removed logContainer element reference
    const wsStatusEl = document.getElementById('wsStatus');
    const valSteer = document.getElementById('valSteer');
    const valThrottle = document.getElementById('valThrottle');
    const knobL = document.getElementById('knobLeft');
    const knobR = document.getElementById('knobRight');
    const stickL = document.getElementById('stickLeft');
    const stickR = document.getElementById('stickRight');

    const state = {steer:0, throttle:0, ws:null, sendInterval:null, videoInterval:null, config:{videoUrl:'',videoFps:10,wsUrl:'',sendRate:50}};

    const left = new VirtualStick(stickL, knobL, n=>{ state.steer = Math.round(n.x*100); valSteer.textContent=state.steer; });
    const right = new VirtualStick(stickR, knobR, n=>{ state.throttle = Math.round(-n.y*100); valThrottle.textContent=state.throttle; });
    
    // --- 日誌輔助函式 (現在輸出到 Console) ---
    function appendLog(message) {
        const timestamp = new Date().toLocaleTimeString('en-US', {hour12: false});
        console.log(`[WS LOG] [${timestamp}] ${message}`); // CRITICAL CHANGE: Use console.log
    }
    // ----------------------

    function connectWs(url){ 
        if(state.ws){ try{state.ws.close()}catch(e){} state.ws=null; } 
        const wsUrl = `ws://${window.location.hostname}:81`;
        
        appendLog(`嘗試連線到 WebSocket: ${wsUrl}`);
        setWsStatus('Connecting...');

        try{ 
            state.ws = new WebSocket(wsUrl); 
            state.ws.binaryType='arraybuffer'; 
            
            state.ws.onopen=()=>{
                setWsStatus('OPEN');
                appendLog('WebSocket 連線成功。');
            }; 
            
            state.ws.onclose=()=>{
                setWsStatus('CLOSED');
                appendLog('WebSocket 已斷線，3秒後重試連線...');
                setTimeout(connectWs, 3000); // 重試連線
            }; 
            
            state.ws.onerror=()=>{
                setWsStatus('ERROR');
                appendLog('WebSocket 連線錯誤。');
            }; 
            
            state.ws.onmessage = (event) => {
                const data = event.data;
                
                // 嘗試解析 JSON (控制狀態)
                try {
                    const json = JSON.parse(data);
                    if (json.debug) {
                        // 這是來自 ESP32 的遠端日誌 (JSON 格式)
                        appendLog(`[DBG] ${json.debug}`);
                    } else if (json.motorA !== undefined) {
                        // 這是馬達狀態更新 (可選)
                        // console.log("Motor Status:", json);
                    }
                } catch(e) {
                    // 如果不是 JSON，則視為遠端日誌文本
                    appendLog(data);
                }
            };
        }catch(e){ 
            setWsStatus('ERROR'); 
            appendLog(`WebSocket 建立失敗: ${e.message}`);
        } 
    }

    function setWsStatus(s){ wsStatusEl.textContent=s; }
    
    // Joystick sending logic remains the same (sends JSON)
    function startSending(rate){ 
      if(state.sendInterval) clearInterval(state.sendInterval); 
      state.sendInterval=setInterval(()=>{ 
        if(state.ws && state.ws.readyState===WebSocket.OPEN){ 
          state.ws.send(JSON.stringify({t:Date.now(),steer:state.steer,throttle:state.throttle})); 
        } 
      }, rate); 
    }
    function stopSending(){ if(state.sendInterval) clearInterval(state.sendInterval); state.sendInterval=null; }

    // Video polling logic remains the same
    async function fetchFrame(){ const url=state.config.videoUrl; if(!url) return; try{ const res=await fetch(url+(url.includes('?')?'&':'?')+'t='+Date.now(),{cache:'no-store'}); if(!res.ok) throw new Error('bad'); const blob=await res.blob(); const img=document.getElementById('video'); const old=img.src; img.src=URL.createObjectURL(blob); if(old&&old.startsWith('blob:')) URL.revokeObjectURL(old); }catch(e){ console.warn(e); } }
    function startVideoPoll(){ stopVideoPoll(); const fps=Math.max(1,parseInt(state.config.videoFps||10)); state.videoInterval=setInterval(fetchFrame, Math.round(1000/fps)); document.getElementById('imgSource').textContent=state.config.videoUrl||'(未設定)'; }
    function stopVideoPoll(){ if(state.videoInterval) clearInterval(state.videoInterval); state.videoInterval=null; }

    window.addEventListener('beforeunload', ()=>{ if(state.ws) state.ws.close(); stopSending(); stopVideoPoll(); });
    
    window.onload = () => {
        connectWs();
        startSending(50); 
    };
  </script>
</body>
</html>
)rawliteral";

// === Motor Control Functions ===
const int MAX_DUTY = 200;           
const unsigned long RAMP_INTERVAL_MS = 30; 
const int RAMP_STEP = MAX_DUTY / 34; 

volatile int targetA = 0; 
volatile int targetB = 0; 
int currentA = 0;          
int currentB = 0;

unsigned long lastRampMillis = 0;
unsigned long lastActivityMillis = 0;
const unsigned long STBY_IDLE_TIMEOUT_MS = 1500; 
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 300; 

void motorEnable(bool enable) {
  digitalWrite(motor_stby, enable ? HIGH : LOW);
}

void applyMotorA(int speed) {
  speed = constrain(speed, -MAX_DUTY, MAX_DUTY);
  if (speed > 0) {
    ledcWrite(CH_A_FWD, speed);
    ledcWrite(CH_A_REV, 0);
  } else if (speed < 0) {
    ledcWrite(CH_A_FWD, 0);
    ledcWrite(CH_A_REV, abs(speed));
  } else {
    ledcWrite(CH_A_FWD, 0);
    ledcWrite(CH_A_REV, 0);
  }
}

void applyMotorB(int speed) {
  speed = constrain(speed, -MAX_DUTY, MAX_DUTY);
  if (speed > 0) {
    ledcWrite(CH_B_RIGHT, speed);
    ledcWrite(CH_B_LEFT, 0);
  } else if (speed < 0) {
    ledcWrite(CH_B_RIGHT, 0);
    ledcWrite(CH_B_LEFT, abs(speed));
  } else {
    ledcWrite(CH_B_RIGHT, 0);
    ledcWrite(CH_B_LEFT, 0);
  }
}

void stopAllMotors() {
  applyMotorA(0);
  applyMotorB(0);
  motorEnable(false);
  sendLogMessage("Motors stopped due to command timeout.");
}

void emergencyStopNow() {
  targetA = targetB = 0;
  currentA = currentB = 0;
  applyMotorA(0);
  applyMotorB(0);
  motorEnable(false);
  sendLogMessage("!!! EMERGENCY STOP Triggered !!!");
}


// === Command Handling ===
void handleCarCommand(char cmd) {
  switch (cmd) {
    case 'A': 
      currentMode = AUTO; 
      sendLogMessage("Mode Switched: AUTO"); 
      break;
    case 'M': 
      currentMode = MANUAL; 
      sendLogMessage("Mode Switched: MANUAL"); 
      break;
    case 'S':
      emergencyStopNow();
      break;
  }
}

void controlByJoystick(int steer, int throttle) {
  targetA = throttle;
  targetB = steer;

  // Real-time control output
  if (steer == 0 && throttle == 0) {
    applyMotorA(0);
    applyMotorB(0);
    motorEnable(false);
  } else {
    motorEnable(true);
    // applyMotorA/B handles constrain
    applyMotorA(throttle); 
    applyMotorB(steer);
  }

  // Send real-time motor status to the browser (JSON format)
  JsonDocument status;
  status["motorA"] = throttle;
  status["motorB"] = steer;
  status["debug"] = String("JSTK:") + throttle + "/" + steer;
  
  size_t json_len = measureJson(status);
  char buffer[json_len + 1];
  size_t len = serializeJson(status, buffer, json_len + 1);
  webSocket.broadcastTXT(buffer, len);
}


// === Wi-Fi Connection and Fallback Logic ===
void connectToWiFi() {
  // Set connection timeout to 15 seconds
  const unsigned long CONNECT_TIMEOUT_MS = 15000; 
  unsigned long connectStart = millis();

  sendLogMessage("Setting WiFi mode to Station...");
  WiFi.mode(WIFI_STA);
  // Attempt to connect using credentials stored in NVS
  WiFi.begin();

  // Check connection status
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    
    if (millis() - connectStart > CONNECT_TIMEOUT_MS) {
      sendLogMessage("WiFi connection timed out after " + String(CONNECT_TIMEOUT_MS / 1000) + " seconds.");
      jumpToFactory(); // Timeout, jump back to Factory
      return; 
    }
    
    // Output wait log 
    if (WiFi.status() == WL_DISCONNECTED) {
        Serial.println("...Waiting for WiFi connection (Status: Disconnected)");
    } else {
        Serial.println("...Waiting for WiFi connection (Status: " + String(WiFi.status()) + ")");
    }
  }
  
  // Connection successful
  sendLogMessage("WiFi Connected! IP Address: " + WiFi.localIP().toString());
}

// === WebSocket Event ===
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: { 
        IPAddress ip = webSocket.remoteIP(num);
        sendLogMessage("--- WS Client Connected from " + ip.toString() + " ---");
      }
      break;
    case WStype_DISCONNECTED:
      sendLogMessage("--- WS Client Disconnected ---");
      // Stop motor immediately but let timeout handle the disable if connection is lost
      targetA = targetB = 0;
      break;
    case WStype_TEXT:
      {
        String msg = String((char*)payload);
        if (msg.length() == 1) {
          handleCarCommand(msg.charAt(0));
          lastCommandTime = millis(); 
        } else {
          JsonDocument doc;
          DeserializationError err = deserializeJson(doc, (const char*)payload); 
          if (!err) {
            int steer = doc["steer"] | 0;
            int throttle = doc["throttle"] | 0;
            controlByJoystick(steer, throttle);
            lastCommandTime = millis(); // Reset timeout on every joystick command
          } else {
            sendLogMessage("WS Error: JSON parse failed: " + String(err.c_str()));
          }
        }
      }
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Set up I/O
  pinMode(motorA_pwm_fwd, OUTPUT);
  pinMode(motorA_pwm_rev, OUTPUT);
  pinMode(motorB_pwm_left, OUTPUT);
  pinMode(motorB_pwm_right, OUTPUT);
  pinMode(motor_stby, OUTPUT);
  motorEnable(false); 
  setupPWM();

  // 1. Connect Wi-Fi (if failed, jump to factory)
  connectToWiFi();
  
  // 2. Setup OTA
  String hostname = "esp32car-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.setPassword("mysecurepassword"); 
  ArduinoOTA.onStart([]() { sendLogMessage("OTA: Start updating " + String(ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem")); });
  ArduinoOTA.onEnd([]() { sendLogMessage("OTA: Update Finished."); });
  ArduinoOTA.onError([](ota_error_t error) { sendLogMessage("OTA Error: " + String(error)); });

  ArduinoOTA.begin();
  sendLogMessage("OTA Ready. Hostname: " + hostname + ".local");


  // 3. Setup Web Interface
  if (MDNS.begin(hostname.c_str())) {
    Serial.printf("mDNS responder started: %s.local\n", hostname.c_str());
  } else {
    sendLogMessage("Error setting up mDNS!");
  }

  // Handle favicon.ico request (prevents 500 error)
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(204); 
  });

  // Serve the HTML content (without processor since no variables are being replaced)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });

  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  sendLogMessage("Web UI Ready on port 80. Remote Control Active.");
}

void loop() {
  ArduinoOTA.handle();
  webSocket.loop();
  
  // Motor timeout logic: if no command received for COMMAND_TIMEOUT (300ms)
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    // Check if motors are currently running (target speed is non-zero)
    if (targetA != 0 || targetB != 0) { 
      stopAllMotors();
      targetA = targetB = 0; // Reset target speeds
    }
  }

  // Example: Periodically send heartbeat log
  static unsigned long lastLogMillis = 0;
  if (millis() - lastLogMillis > 5000) { 
    sendLogMessage("Heartbeat: Car system active, Mode=" + String(currentMode == AUTO ? "AUTO" : "MANUAL"));
    lastLogMillis = millis();
  }
}
