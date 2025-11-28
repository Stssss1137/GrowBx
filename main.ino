#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "telegram_bot.h"

#define PIN_MQ135 A0           
#define PIN_SOIL A3          
#define PIN_LDR_DO 33       
#define PIN_LED 2           
#define PIN_PUMP_RELAY 26      
#define I2C_SDA 21
#define I2C_SCL 22
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

unsigned long lastSerialReport = 0;
const unsigned long SERIAL_REPORT_INTERVAL = 60UL * 1000UL;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000UL; 
int humidityThreshold = 50;
Adafruit_AHTX0 aht;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ----- custom symbols
uint8_t LT[8]  = {B00111, B01111, B11111, B11111, B11111, B11111, B11111, B11111};
uint8_t UB[8]  = {B11111, B11111, B11111, B00000, B00000, B00000, B00000, B00000};
uint8_t RT[8]  = {B11100, B11110, B11111, B11111, B11111, B11111, B11111, B11111};
uint8_t LL[8]  = {B11111, B11111, B11111, B11111, B11111, B11111, B01111, B00111};
uint8_t LB[8]  = {B00000, B00000, B00000, B00000, B00000, B11111, B11111, B11111};
uint8_t LR[8]  = {B11111, B11111, B11111, B11111, B11111, B11111, B11110, B11100};
uint8_t UMB[8] = {B11111, B11111, B11111, B00000, B00000, B00000, B11111, B11111};
uint8_t LMB[8] = {B11111, 0, 0, 0, 0, B11111, B11111, B11111};
const uint8_t digits[10][2][3] = {
  { {0, 1, 2}, {3, 4, 5} }, { {1, 2, 32}, {4, 5, 4} }, { {6, 6, 2}, {3, 4, 4} },
  { {6, 6, 2}, {4, 4, 5} }, { {3, 4, 5}, {32, 32, 5} }, { {0, 6, 6}, {4, 4, 5} },
  { {0, 6, 6}, {3, 4, 5} }, { {0, 1, 2}, {32, 32, 5} }, { {0, 6, 2}, {3, 7, 5} },
  { {0, 6, 2}, {32, 32, 5} }
};


enum Screen { SCREEN_CLOCK = 0, SCREEN_POLIV = 1 };
Screen currentScreen = SCREEN_CLOCK;
bool pumpState = false; 
bool autoMode = true;

void readSensors();
void printSerialReport();
void updateLCD();
void drawBigTime(int hh, int mm);
void drawDigitAt(int col, int digit);
void setPump(bool on);
void checkAutoLogic();
float safeReadAHTHumidity();
float safeReadAHTTemperature();
int readMQ135();
int readSoil();
int soilPercent();
int getSoilPercent(); 

float aht_temp = NAN, aht_hum = NAN;
int mq135_val = -1;
int soil_val = -1;
bool ldr_dark = false;
bool aht_present = false;

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(PIN_LDR_DO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PUMP_RELAY, OUTPUT);
  setPump(false);
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, LT);
  lcd.createChar(1, UB);
  lcd.createChar(2, RT);
  lcd.createChar(3, LL);
  lcd.createChar(4, LB);
  lcd.createChar(5, LR);
  lcd.createChar(6, UMB);
  lcd.createChar(7, LMB);

  aht_present = false;
  if (aht.begin()) {
    Serial.println("AHT20 OK");
    aht_present = true;
  } else {
    Serial.println("AHT20 not found");
  }

  setupWiFiAndBot();
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  lastSerialReport = millis();
  lastSensorRead = 0;
}

// ---------------- loop ----------------
void loop() {
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    checkAutoLogic();
    updateLCD();
  }

  if (now - lastSerialReport >= SERIAL_REPORT_INTERVAL) {
    lastSerialReport = now;
    printSerialReport();
  }
  handleTelegram(); 
  delay(10);
}

void setPump(bool on) {
  pumpState = on;
  digitalWrite(PIN_PUMP_RELAY, on ? LOW : HIGH);
}

void checkAutoLogic() {
  if (!autoMode) return;
  if (soil_val < 0) return;

  int pct = soilPercent();
  if (pct < 0) return;

  if (pct < humidityThreshold) {
    if (!pumpState) {
      setPump(true);
      Serial.printf("Auto: soil %d%% < %d%% -> pump ON\n", pct, humidityThreshold);
    }
  } else {
    if (pumpState) {
      setPump(false);
      Serial.printf("Auto: soil %d%% >= %d%% -> pump OFF\n", pct, humidityThreshold);
    }
  }
}

void readSensors() {
  if (aht_present) {
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp)) {
      aht_hum = humidity.relative_humidity;
      aht_temp = temp.temperature;
    } else {
      aht_hum = NAN;
      aht_temp = NAN;
    }
  } else {
    aht_temp = NAN; aht_hum = NAN;
  }
  #if defined(ARDUINO_ARCH_ESP32)
    mq135_val = analogRead(PIN_MQ135);
  #else
    mq135_val = analogRead(PIN_MQ135);
  #endif
  #if defined(ARDUINO_ARCH_ESP32)
    soil_val = analogRead(PIN_SOIL);
  #else
    soil_val = analogRead(PIN_SOIL);
  #endif

  ldr_dark = (digitalRead(PIN_LDR_DO) == LOW);
  digitalWrite(PIN_LED, ldr_dark ? HIGH : LOW);

  Serial.printf("DBG: aht_temp=");
  if (!isnan(aht_temp)) Serial.printf("%.2f", aht_temp); else Serial.print("N/A");
  Serial.printf(" aht_hum=");
  if (!isnan(aht_hum)) Serial.printf("%.2f", aht_hum); else Serial.print("N/A");
  Serial.printf(" | MQ135=%d soil=%d (pct=%d) LDR=%s\n", mq135_val, soil_val, soilPercent(), ldr_dark ? "LIGHT":"DARK");
}

void printSerialReport() {
  Serial.println("----- sensor report -----");
  if (!isnan(aht_temp)) Serial.printf("AHT Temp: %.2f C, Hum: %.2f %%\n", aht_temp, aht_hum);
  else Serial.println("AHT: N/A");
  // BME removed
  Serial.printf("MQ135 analog: %d\n", mq135_val);
  Serial.printf("Soil analog: %d (pct: %d%%)\n", soil_val, soilPercent());
  Serial.printf("LDR dark: %s\n", ldr_dark ? "YES" : "NO");
  Serial.printf("Pump: %s, mode: %s\n", pumpState ? "ON" : "OFF", autoMode ? "AUTO" : "MANUAL");
  Serial.printf("Hum threshold (hygro %%): %d\n", humidityThreshold);
  Serial.println("-------------------------");
}

void updateLCD() {
  lcd.clear();
  if (currentScreen == SCREEN_CLOCK) {
    time_t rawtime = time(nullptr);
    if (rawtime == ((time_t)0)) {
      lcd.setCursor(3,1);
      lcd.print("No NTP");
      lcd.setCursor(3,2);
      lcd.print("Check WiFi");
      return;
    }
    rawtime += 2 * 3600;
    struct tm * timeinfo = gmtime(&rawtime);
    int hh = timeinfo->tm_hour;
    int mm = timeinfo->tm_min;
    drawBigTime(hh, mm);
    lcd.setCursor(0,3);
    lcd.print("T:");
    if (!isnan(aht_temp)) {
      lcd.print(String(aht_temp, 1) + "C ");
    } else {
      lcd.print("--- ");
    }
    lcd.print("AirH:");
    if (!isnan(aht_hum)) {
      lcd.print(String(aht_hum, 0) + "% ");
    } else {
      lcd.print("--% ");
    }
    lcd.print("Soil:");
    int sp = soilPercent();
    if (sp >= 0) lcd.print(String(sp) + "%");
    else lcd.print("--%");
  } else if (currentScreen == SCREEN_POLIV) {
    lcd.setCursor(0,0);
    lcd.print("Poliv");
    lcd.setCursor(0,1);
    lcd.print("AutoMode:");
    lcd.print(autoMode ? "YES" : "NO ");
    lcd.setCursor(0,2);
    lcd.print("Pump state:");
    lcd.print(pumpState ? "ON " : "OFF");
    lcd.setCursor(0,3);
    lcd.print("Use Telegram for ctrl");
  }
}

void drawBigTime(int hh, int mm) {
  int d1 = hh / 10;
  int d2 = hh % 10;
  int d3 = mm / 10;
  int d4 = mm % 10;
  int totalCols = 3*4 + 1; 
  int startCol = (LCD_COLS - totalCols) / 2; 
  for (int c=0;c< LCD_COLS;c++) {
    lcd.setCursor(c,0); lcd.print(' ');
    lcd.setCursor(c,1); lcd.print(' ');
  }

  drawDigitAt(startCol, d1);
  drawDigitAt(startCol + 3, d2);
  lcd.setCursor(startCol + 6,0);
  lcd.print(" ");
  lcd.setCursor(startCol + 6,1);
  lcd.print(":");
  drawDigitAt(startCol + 7, d3);
  drawDigitAt(startCol + 10, d4);
}

void drawDigitAt(int col, int digit) {
  int r0 = 0, r1 = 1;
  for (int i=0;i<3;i++){
    int idx = digits[digit][0][i];
    lcd.setCursor(col + i, r0);
    if (idx == 32) lcd.print(' ');
    else if (idx <= 7) lcd.write((uint8_t)idx);
    else lcd.print(' ');
  }
  for (int i=0;i<3;i++){
    int idx = digits[digit][1][i];
    lcd.setCursor(col + i, r1);
    if (idx == 32) lcd.print(' ');
    else if (idx <= 7) lcd.write((uint8_t)idx);
    else lcd.print(' ');
  }
}
float safeReadAHTHumidity() { return aht_present ? aht_hum : NAN; }
float safeReadAHTTemperature() { return aht_present ? aht_temp : NAN; }

int soilPercent() {
  if (soil_val < 0) return -1;
  long pct = map(soil_val, 0, 4095, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}
int getSoilPercent() { return soilPercent(); }

float getAHTTemp(){ return safeReadAHTTemperature(); }
float getAHTHum(){ return safeReadAHTHumidity(); }
int getMQ135(){ return mq135_val; }
int getSoil(){ return soil_val; }
bool isDark(){ return ldr_dark; }
bool getPumpState(){ return pumpState; }
bool isAutoMode(){ return autoMode; }

void changePumpFromBot(bool on, String whoName, String whoId) {
  setPump(on);
  autoMode = false;
  Serial.printf("Telegram: %s (%s) set pump %s. autoMode=0\n", whoName.c_str(), whoId.c_str(), on ? "ON":"OFF");
}

void setHumThreshold(int v) {
  humidityThreshold = v;
  Serial.printf("Humidity threshold (hygro %%) set to %d by telegram\n", v);
}

void setAutoModeFromBot(bool on, String whoName, String whoId) {
  autoMode = on;
  Serial.printf("Telegram: %s (%s) set autoMode=%d\n", whoName.c_str(), whoId.c_str(), on ? 1:0);
  if (on) {
    checkAutoLogic();
  }
}
