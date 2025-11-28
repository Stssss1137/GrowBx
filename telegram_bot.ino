#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

extern float getAHTTemp();
extern float getAHTHum();
extern int getMQ135();
extern int getSoil();
extern int getSoilPercent();
extern bool isDark();
extern bool getPumpState();
extern bool isAutoMode();
extern void changePumpFromBot(bool on, String whoName, String whoId);
extern void setHumThreshold(int v);
extern void setAutoModeFromBot(bool on, String whoName, String whoId);
extern int humidityThreshold;

// ---- CONFIG ---- 
const char* WIFI_SSID = "XXXXXXXXX";
const char* WIFI_PASSWORD = "XXXXXXXXXX";
const char* BOT_TOKEN = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

WiFiClientSecure securedClient;
UniversalTelegramBot bot(BOT_TOKEN, securedClient);

unsigned long bot_lasttime = 0;
const unsigned long BOT_CHECK_INTERVAL = 2000;

void setupWiFiAndBot() {
  Serial.printf("Connecting to %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi failed");
  } else {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
  }
  securedClient.setInsecure();

  Serial.println("Telegram bot ready (token in code).");
}
String makeStatusText() {
  String s;
  s += "Status:\n";
  float at = getAHTTemp();
  float ah = getAHTHum();
  s += "AHT temp: ";
  if (!isnan(at)) s += String(at,1) + "C\n"; else s += "N/A\n";
  s += "Air humidity (AHT): ";
  if (!isnan(ah)) s += String(ah,0) + "%\n"; else s += "N/A\n";
  int hygro = getSoilPercent();
  if (hygro >= 0) s += "Soil hygrometer: " + String(hygro) + "%\n";
  else s += "Soil hygrometer: N/A\n";
  s += "MQ135: " + String(getMQ135()) + "\n";
  s += "Soil raw: " + String(getSoil());
  if (hygro >= 0) s += " (" + String(hygro) + "%)\n"; else s += "\n";
  s += "LDR: "; s += isDark() ? "DARK\n" : "LIGHT\n";
  s += "Pump: "; s += getPumpState() ? "ON\n" : "OFF\n";
  s += "Mode: "; s += isAutoMode() ? "AUTO\n" : "MANUAL\n";
  s += "Hum threshold (hygro %): " + String(humidityThreshold) + "\n";
  return s;
}

void handleTelegram() {
  if (millis() - bot_lasttime < BOT_CHECK_INTERVAL) return;
  bot_lasttime = millis();

  int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  while (numNewMessages) {
    for (int i = 0; i < numNewMessages; i++) {
      String chat_id = String(bot.messages[i].chat_id);
      String text = bot.messages[i].text;
      String from_name = bot.messages[i].from_name;
      if (from_name == "") from_name = "unknown";

      Serial.printf("Telegram msg from %s (%s): %s\n", from_name.c_str(), chat_id.c_str(), text.c_str());
      if (text == "/status") {
        String stat = makeStatusText();
        bot.sendMessage(chat_id, stat, "");
      } else if (text == "/start") {
        String greet = "Привіт! GrowBox .\nЯ надішлю статус, керування насосом і дозволю міняти поріг вологості.\nКоманди: /status /pump_on /pump_off /manual /auto /set_hum_threshold";
        bot.sendMessage(chat_id, greet, "");
      } else if (text == "/pump_on") {
        changePumpFromBot(true, from_name, chat_id);
        bot.sendMessage(chat_id, "Pump turned ON (manual).", "");
      } else if (text == "/pump_off") {
        changePumpFromBot(false, from_name, chat_id);
        bot.sendMessage(chat_id, "Pump turned OFF (manual).", "");
      } else if (text == "/manual") {
        setAutoModeFromBot(false, from_name, chat_id);
        bot.sendMessage(chat_id, "Manual mode enabled.", "");
      } else if (text == "/auto") {
        setAutoModeFromBot(true, from_name, chat_id);
        bot.sendMessage(chat_id, "Auto mode enabled. Using thresholds.", "");
      } else if (text.startsWith("/set_hum_threshold")) {
        int sp = text.indexOf(' ');
        if (sp > 0) {
          int val = text.substring(sp + 1).toInt();
          if (val < 0) val = 0;
          if (val > 100) val = 100;
          setHumThreshold(val);
          bot.sendMessage(chat_id, "Humidity threshold set to " + String(val) + " (hygrometer %)", "");
        } else {
          bot.sendMessage(chat_id, "Usage: /set_hum_threshold 50  (value in percent, 0..100, refers to soil hygrometer)", "");
        }
      } else {
        String help = "Commands:\n/start\n/status\n/pump_on\n/pump_off\n/manual\n/auto\n/set_hum_threshold <val> (hygro %)\n";
        bot.sendMessage(chat_id, help, "");
      }
    }
    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
}
