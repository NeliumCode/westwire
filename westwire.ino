#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>

#define OE      D1
#define DCDC    D2
#define VERSION     "v1.2"
#define disable 0
#define TINY_GSM_MODEM_UBLOX

static unsigned long timecount = millis();                    // Counter to reset client count
unsigned int channel = 1;
int mqttConnectionFailures = 0;
long lastReconnectAttempt = 0;
String ID = String(ESP.getChipId(), HEX);
const char* charId = ID.c_str();


// Set serial for for AT commands (to the module)
#define SerialAT Serial

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial1

// or Software Serial for debugging
// #include <SoftwareSerial.h>
// SoftwareSerial SerialMon(D6, D2); // RX, TX

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS 

// Define the serial console for debug prints, if needed
// #define TINY_GSM_DEBUG SerialMon

// Range to attempt to autobaud
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// Add a reception delay - may be needed for a fast processor at a slow baud rate
//#define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
//const char wifiSSID[] = "YourSSID";
//const char wifiPass[] = "YourWiFiPass";

// MQTT details
const char* broker = "mqtt.ivanlab.org";
const char* topicData = "ivanlab/westwire/S";
const char* topicCtrl = "ivanlab/westwire/C";
const char* topicRx = "ivanlab/westwire/CC";


#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
#include <PubSubClient.h>
PubSubClient mqtt(client);

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println(); 
 
  /*
  // Only proceed if incoming message's topic matches
  if (String(topic) == topicLed) {
    ledStatus = !ledStatus;
    digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  }
  */
}

boolean mqttConnect() {
  SerialMon.print(F("Connecting to "));
  SerialMon.println(broker);
  
  String connMsg=String("Sensor " + ID + " connected - "+VERSION+"\n"+" IMEI:"+modem.getIMEI());
  const char* charConnMsg=connMsg.c_str();

  // Connect to MQTT Broker
  boolean status = mqtt.connect(charId);

  // Or, if you want to authenticate MQTT:
  //boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    SerialMon.println(F("....fail connecting MQTT"));
    mqttConnectionFailures++;
    SerialMon.print(" MQTT connection failures=");SerialMon.println(mqttConnectionFailures);
    if (mqttConnectionFailures>=3) reboot();
    return false;
  }
  SerialMon.println(F(" successfully connected MQTT"));
  mqtt.publish(topicCtrl,charConnMsg);
  mqtt.subscribe(topicRx);
  return mqtt.connected();
}

void reboot() {
          SerialMon.println(F("*****REBOOT*****"));
          digitalWrite(DCDC,LOW);
          delay(3000);
          ESP.restart();
}

#include "./functions2.h"

void setup() {

  pinMode(OE,OUTPUT);
  pinMode(DCDC,OUTPUT);               
  digitalWrite(OE,LOW);                           
  digitalWrite(DCDC,HIGH);            

  SerialMon.begin(9600);
  delay(10);

  SerialMon.println(F("Wait for initialization..."));
  
  // Set GSM module baud rate
  TinyGsmAutoBaud(SerialAT,GSM_AUTOBAUD_MIN,GSM_AUTOBAUD_MAX);
  SerialAT.begin(115200);
  delay(1000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println(F("Initializing modem..."));
  
  if (!modem.restart()) {
    SerialMon.println ("Failed to restart modem, retrying");
    reboot();
  }

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  
  #if TINY_GSM_USE_GPRS
    // Unlock your SIM card with a PIN if needed
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
      modem.simUnlock(GSM_PIN);
    }
  #endif

  #if TINY_GSM_USE_WIFI
      // Wifi connection parameters must be set before waiting for the network
    SerialMon.print(F("Setting SSID/password..."));
    if (!modem.networkConnect(wifiSSID, wifiPass)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    SerialMon.println(" success");
  #endif

  SerialMon.print("Waiting for network...");
  
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail to get net in 10 sec");
    delay(1000);
    return;
  }
  
  SerialMon.println(F(" Successfully attached to GSM network"));

  if (modem.isNetworkConnected()) {
    SerialMon.println(F("Network connected"));
  }

  #if TINY_GSM_USE_GPRS
    // GPRS connection parameters are usually set after network registration
      SerialMon.print(F("Connecting to APN "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        reboot();
      }
    SerialMon.println(F(" Successfully established APN context"));

    if (modem.isGprsConnected()) {
      SerialMon.println(F("GPRS connected"));
    }
  #endif


  String ccid = modem.getSimCCID();
  DBG("CCID:", ccid);

  String imei = modem.getIMEI();
  DBG("IMEI:", imei);

  String cop = modem.getOperator();
  DBG("Operator:", cop);

  IPAddress local = modem.localIP();
  DBG("Local IP:", local);

  int csq = modem.getSignalQuality();
  DBG("Signal quality:", csq);

  String pos = modem.getGsmLocation();
  DBG("Position:", pos);

  SerialMon.println("CCID:"+ ccid);
  SerialMon.println("IMEI:"+ imei);
  SerialMon.println("Operator:"+ cop);
  SerialMon.println("Local IP:"+ local);



  wifi_set_opmode(STATION_MODE);            // Promiscuous works only with station mode
  wifi_set_channel(channel);
  wifi_promiscuous_enable(disable);
  wifi_set_promiscuous_rx_cb(promisc_cb);   // Set up promiscuous callback
  wifi_promiscuous_enable(1);


  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  mqttConnect();


}

void loop() {
 // Looping through 2.4 GHz channels
  //SerialMon.println(F("Scanning Wifi Channels"));
  channel = 1;
  wifi_set_channel(channel);
  while (true) {
    nothing_new++;                     // Array is not finite, check bounds and adjust if required
    if (nothing_new > 200) {
      nothing_new = 0;
      channel++;
      if (channel == 15) break;        // Only scan channels 1 to 14
      wifi_set_channel(channel);
    }
    delay(1);                         // critical processing timeslice for NONOS SDK! No delay(0) yield()
  }
 // Checking transmission timer >> MQTT sending
 if (!modem.isGprsConnected() {
   reboot();
 }
 unsigned long t = millis();
 //SerialMon.println("millis - lastReconnect =" + String(t- lastReconnectAttempt));
    if (t - lastReconnectAttempt > 300000L) {
            lastReconnectAttempt = t;
            char buffer[75];

            for (int u = 0; u < clients_known_count; u++){
                String transmision ="{\"ID\":\""+ID+"\",\"mac\":\"";
                for (int i = 0; i < 6; i++) {
                       if (clients_known[u].station[i] < 0x10) {
                          transmision += "0";
                          transmision += String(clients_known[u].station[i], HEX);
                        }else{
                          transmision += String(clients_known[u].station[i], HEX);
                        }
                       if (i<5) transmision += ":";
                    }
               transmision += "\",\"rssi\":\"";
               transmision += String(clients_known[u].rssi, DEC);              
               transmision += "\"}";
               transmision.toCharArray(buffer, transmision.length()+1);
               if(!mqtt.connected()) mqttConnect();
                  if(mqtt.connected()) {
                      mqtt.publish(topicData,buffer);
                      SerialMon.println (transmision);
                    }else{
                      SerialMon.println (F("NETWORK PROBLEMS - MQTT NOT RESPONDING"));
                    }
               }
            clients_known_count=0;
            }
 mqtt.loop();
}