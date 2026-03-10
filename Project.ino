  #include <Wire.h>
  #include <EEPROM.h>
  #include <WiFi.h>
  #include <WebServer.h>
  #include <WebSocketsServer.h>
  #include <PubSubClient.h>

  #define VBAT          25
  #define BUZZER        12
  #define BRAKE          4
  #define DIR1          16
  #define PWM1          17
  #define ENC_FG        19

  #define MPU6050_ADDR  0x68
  #define ACCEL_CONFIG  0x1C
  #define GYRO_CONFIG   0x1B
  #define PWR_MGMT_1    0x6B
  #define EEPROM_SIZE   64

  // --- WiFi & MQTT Config ---
  const char* ssid        = "K";
  const char* password    = "karn2006";
  const char* mqtt_server = "test.mosquitto.org";

  WiFiClient   espClient;
  PubSubClient client(espClient);
  WebServer    server(80);
  WebSocketsServer webSocket = WebSocketsServer(81);

  float vDividerRatio = 289.0;
  float Gyro_amount   = 0.85;
  float alpha         = 0.7;

  float eK1 = 10.0;
  float eK2 = 30.0;
  float eK3 = 2.5;
  float eK4 = 0.01;

  const float INTEGRAL_LIMIT = 50.0;
  int loop_time = 10;

  unsigned long currentT, previousT_1, previousT_2, previousT_IoT;

  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  float gyroXfilt      = 0;
  float robot_angleX   = 0;
  float Acc_angleX     = 0;
  float angle_offset   = 0;
  float gyroX_offset   = 0;
  float integral_angle = 0;

  volatile int enc_count1 = 0;
  int16_t motor1_speed    = 0;

  bool vertical_edge = false;
  bool calibrating   = false;
  bool calibrated    = false;

  // --- Web Dashboard HTML ---
  const char html_template[] PROGMEM = R"=====(
  <html><head><meta charset="utf-8"><title>Cube Monitor</title>
  <style>
    body { text-align:center; font-family:sans-serif; background:#222; color:#fff; margin:20px; }
    .container { display:grid; grid-template-columns: 1fr 1fr; gap:10px; max-width:400px; margin:auto; }
    .box { background:#333; padding:15px; border-radius:10px; border:1px solid #444; }
    .val { font-size:24px; color:#00ff00; font-weight:bold; }
    .label { font-size:12px; color:#aaa; display:block; }
    h1 { color:#00d1b2; }
  </style>
  <script>
    var socket = new WebSocket('ws://'+location.host+':81');
    socket.onmessage = function(e){
      var d = e.data.split('|');
      document.getElementById('ang').innerHTML = d[0];
      document.getElementById('bat').innerHTML = d[1];
      document.getElementById('k1').innerHTML  = d[2];
      document.getElementById('k2').innerHTML  = d[3];
      document.getElementById('k3').innerHTML  = d[4];
      document.getElementById('k4').innerHTML  = d[5];
    };
  </script>
  </head><body>
    <h1>Cube MQTT Monitor</h1>
    <div class="container">
      <div class="box" style="grid-column: span 2;"><span class="label">Angle</span><span id="ang" class="val">0</span>°</div>
      <div class="box" style="grid-column: span 2;"><span class="label">Battery</span><span id="bat" class="val">0</span>V</div>
      <div class="box"><span class="label">eK1</span><span id="k1" class="val">0</span></div>
      <div class="box"><span class="label">eK2</span><span id="k2" class="val">0</span></div>
      <div class="box"><span class="label">eK3</span><span id="k3" class="val">0</span></div>
      <div class="box"><span class="label">eK4</span><span id="k4" class="val">0</span></div>
    </div>
  </body></html>
  )=====";

  void IRAM_ATTR ENC1_READ() { enc_count1++; }

  // --- MQTT Callback: ปรับ PID ผ่าน /Cube/cmd ---
  // ส่ง "1+" เพิ่ม eK1, "1-" ลด eK1 ฯลฯ
  // ส่ง "c+" เพื่อเริ่ม calibrate, "c-" เพื่อยืนยัน
  void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    if (length < 2) return;
    char param = (char)payload[0];
    char cmd   = (char)payload[1];

    if (param == '1') { if (cmd=='+') eK1 += 2;     else if (cmd=='-') eK1 -= 2; }
    if (param == '2') { if (cmd=='+') eK2 += 0.5;   else if (cmd=='-') eK2 -= 0.5; }
    if (param == '3') { if (cmd=='+') eK3 += 0.05;  else if (cmd=='-') eK3 -= 0.05; }
    if (param == '4') { if (cmd=='+') eK4 += 0.001; else if (cmd=='-') eK4 -= 0.001; }

    if (param == 'c') {
      if (cmd == '+') {
        calibrating = true; beep();
      }
      if (cmd == '-' && calibrating) {
        float accY = (float)AcY / 16384.0;
        float accZ = (float)AcZ / 16384.0;
        angle_offset   = -atan2(accY, accZ) * 57.2958;
        robot_angleX   = 0;
        integral_angle = 0;
        gyroXfilt      = 0;
        calibrated     = true;
        calibrating    = false;
        beep(); delay(200); beep();
      }
    }
    beep();
  }

  void setup() {
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);

    pinMode(BUZZER, OUTPUT);
    pinMode(BRAKE,  OUTPUT);
    pinMode(VBAT,   INPUT);
    pinMode(DIR1,   OUTPUT);
    pinMode(ENC_FG, INPUT_PULLUP);

    digitalWrite(BRAKE, HIGH);
    digitalWrite(DIR1,  LOW);

    ledcAttach(PWM1, 20000, 8);
    ledcWrite(PWM1, 255);

    attachInterrupt(digitalPinToInterrupt(ENC_FG), ENC1_READ, RISING);

    // --- WiFi ---
    WiFi.begin(ssid, password);
    Serial.print("Connecting WiFi");
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
      delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi OK: " + WiFi.localIP().toString());
    } else {
      Serial.println("\nWiFi FAILED - continuing offline");
    }

    // --- MQTT ---
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqtt_callback);

    // --- Web & WebSocket ---
    server.on("/", []() { server.send_P(200, "text/html", html_template); });
    server.begin();
    webSocket.begin();

    angle_setup();
    calibrateGyroBias();
    beep();
  }

  void loop() {
    currentT = millis();

    if (currentT - previousT_1 >= (unsigned long)loop_time) {
      float dt = (currentT - previousT_1) / 1000.0;
      previousT_1 = currentT;

      angle_calc(dt);

      noInterrupts();
      motor1_speed = enc_count1;
      enc_count1   = 0;
      interrupts();

      if (vertical_edge && calibrated && !calibrating) {
        digitalWrite(BRAKE, HIGH);

        float gyroRate = (GyX - gyroX_offset) / 131.0;
        gyroXfilt = alpha * gyroRate + (1.0 - alpha) * gyroXfilt;

        float pid_out = eK1 * robot_angleX
                      + eK2 * gyroXfilt
                      - eK3 * (float)motor1_speed
                      + eK4 * integral_angle;

        int pwm_X = constrain((int)pid_out, -255, 255);
        Motor1_control(pwm_X);

        integral_angle += robot_angleX * dt;
        integral_angle  = constrain(integral_angle, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        Serial.print("A:"); Serial.print(robot_angleX, 2);
        Serial.print(" G:"); Serial.print(gyroXfilt, 2);
        Serial.print(" S:"); Serial.print(motor1_speed);
        Serial.print(" P:"); Serial.println(pwm_X);

      } else {
        Motor1_control(0);
        digitalWrite(BRAKE, LOW);
        integral_angle = 0;
        gyroXfilt      = 0;
      }
    }

    // --- IoT: WebSocket + MQTT publish ทุก 100ms ---
    if (currentT - previousT_IoT >= 100) {
      previousT_IoT = currentT;

      if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) {
          if (client.connect("Cubli_ESP32")) {
            client.subscribe("/Cube/cmd");
          }
        }
        client.loop();
        webSocket.loop();
        server.handleClient();

        float voltage = (float)analogRead(VBAT) / vDividerRatio;

        String data = String(robot_angleX, 1) + "|" +
                      String(voltage, 2)       + "|" +
                      String(eK1, 0)           + "|" +
                      String(eK2, 1)           + "|" +
                      String(eK3, 2)           + "|" +
                      String(eK4, 3);

        webSocket.broadcastTXT(data);
        client.publish("/Cube/status", data.c_str());
      }
    }

    // --- Battery Check ทุก 2 วินาที ---
    if (currentT - previousT_2 >= 2000) {
      checkBattery();
      previousT_2 = currentT;
    }
  }

  void Motor1_control(int sp) {
    if (sp == 0) {
      ledcWrite(PWM1, 255);
      return;
    }

    digitalWrite(BRAKE, HIGH);

    if (sp > 0) {
      digitalWrite(DIR1, HIGH);
    } else {
      digitalWrite(DIR1, LOW);
    }

    int pwm = map(constrain(abs(sp), 0, 255), 0, 255, 255, 0);
    ledcWrite(PWM1, pwm);
  }

  void calibrateGyroBias() {
    delay(1000);
    for (int i = 0; i < 100; i++) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x43); Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 2, true);
      Wire.read(); Wire.read();
      delay(2);
    }
    long sum = 0;
    for (int i = 0; i < 500; i++) {
      Wire.beginTransmission(MPU6050_ADDR);
      Wire.write(0x43); Wire.endTransmission(false);
      Wire.requestFrom(MPU6050_ADDR, 2, true);
      int16_t gx = Wire.read() << 8 | Wire.read();
      sum += gx;
      delay(2);
    }
    gyroX_offset = sum / 500.0;
    if (abs(gyroX_offset) > 1000) gyroX_offset = 0;
    Serial.print("Gyro offset: "); Serial.println(gyroX_offset);
  }

  void angle_setup() {
    Wire.begin();
    writeTo(MPU6050_ADDR, PWR_MGMT_1,   0);
    writeTo(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
    writeTo(MPU6050_ADDR, GYRO_CONFIG,  0x00);
    writeTo(MPU6050_ADDR, 0x1A,         0x03);
    delay(500);
  }

  void angle_calc(float dt) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    float gyro_rate = (GyX - gyroX_offset) / 131.0;

    float accY = (float)AcY / 16384.0;
    float accZ = (float)AcZ / 16384.0;
    Acc_angleX = -atan2(accY, accZ) * 57.2958 - angle_offset;

    robot_angleX = Gyro_amount * (robot_angleX + gyro_rate * dt)
                + (1.0 - Gyro_amount) * Acc_angleX;

    if (abs(robot_angleX) < 1.5) {
      vertical_edge = true;
    } else if (abs(robot_angleX) > 20.0) {
      vertical_edge = false;
    }
  }

  void writeTo(byte device, byte address, byte value) {
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission(true);
  }

  void beep() {
    digitalWrite(BUZZER, HIGH); delay(100);
    digitalWrite(BUZZER, LOW);  delay(100);
  }

  void checkBattery() {
    long rawSum = 0;
    for (int i = 0; i < 10; i++) rawSum += analogRead(VBAT);
    float voltage = (rawSum / 10.0) / vDividerRatio;

    Serial.print("Batt: "); Serial.print(voltage); Serial.println("V");

    if (voltage <= 10.5)      digitalWrite(BUZZER, HIGH);
    else if (voltage <= 11.1) digitalWrite(BUZZER, !digitalRead(BUZZER));
    else                      digitalWrite(BUZZER, LOW);
  }
