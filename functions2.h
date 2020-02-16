// based on Ray Burnette 20161013 work ( using Arduino 1.6.12 )
// added XBEE transport by Ivan Padilla 20171001
// added MQTT-SN encapsulation by Ivan Padilla 20171001
// added GPS support by Ivan Padilla 20171101

// Expose Espressif SDK functionality
extern "C" {
  #include "user_interface.h"
  typedef void (*freedom_outside_cb_t)(uint8 status);
  int  wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
  void wifi_unregister_send_pkt_freedom_cb(void);
  int  wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

#define sensorID    "S5"

#include <WiFi.h>
#include "./structures.h"

//Wifi Initialization Parameters
#define MAX_APS_TRACKED     1
#define MAX_CLIENTS_TRACKED 500
beaconinfo aps_known[MAX_APS_TRACKED];            // Array to save MACs of known APs
int aps_known_count = 0;                          // Number of known APs
int nothing_new = 0;
clientinfo clients_known[MAX_CLIENTS_TRACKED];    // Array to save data of known CLIENTs
int clients_known_count = 0;                      // Number of known CLIENTs
unsigned long times[MAX_CLIENTS_TRACKED];
static unsigned long timed = millis();

//MQTT-SN Initialization Parameters
#define MQTT_SN_MAX_PACKET_LENGTH     (900)
#define MQTT_SN_TYPE_PUBLISH          (0x0C)
#define MQTT_SN_FLAG_QOS_N1           (0x3 << 5)
#define MQTT_SN_FLAG_RETAIN           (0x1 << 4)
#define MQTT_SN_TOPIC_TYPE_SHORT      (0x02)


// MQTT-SN Message encapsulation and sending
void sendMessage(const char topic[2], byte message[], int messageSize, bool retain=false) {
  uint8_t payload[messageSize+8];                // Total size of MQTT-SN packet to be sent
  // Load the payload with the header
  payload[0] = messageSize+8;                    // header +sizeof message
  payload[1] = MQTT_SN_TYPE_PUBLISH;
  payload[2] = MQTT_SN_FLAG_QOS_N1 | MQTT_SN_TOPIC_TYPE_SHORT;
  if (retain) {
    payload[2] |= MQTT_SN_FLAG_RETAIN;
  }
  payload[3] = topic[0];
  payload[4] = topic[1];
  payload[5] = 0x00;                  // Message ID High
  payload[6] = 0x00;                  // message ID Low
  payload[7] = 0x07;                  // size of header

  //SerialMon.print("Type="); //SerialMon.println(topic[0]);
  //SerialMon.print("Payload size advertised="); //SerialMon.println(payload[0], DEC);
  //SerialMon.print("Payload>");
  // Load the message in the MQTT-SN payload
  for (int i=8;i<messageSize+8;i++){
    payload[i] = message[i-8];
  }

 // //Serialize the MQTT-SN packet
  String stringOne = "";
  int ndx=0;
  //Serial1.write('<'); //SerialMon.print("<");              // startByte
  for (int i=0;i<messageSize+8;i++){
    if (payload[i] < 0x10) {
      stringOne = "0";
      stringOne+= String(payload[i], HEX);
    }else{
      stringOne = String(payload[i], HEX);
    }
    //Serial1.print(stringOne);      //transmit to Particle via //Serial1
    //SerialMon.print(stringOne);//SerialMon.print("|");
    ndx=ndx+2;
  }
   //Serial1.print(">");
   //Serial1.print(">");
   //Serial1.print(">");
   //Serial1.print(">");
   //Serial1.print(">");
   //Serial1.print(">");
   //SerialMon.println(">");                // stopByte
  //SerialMon.print("Caracteres enviados:"); //SerialMon.println(ndx+3);
  //SerialMon.println();
}

// Send array of clientinfo stored to MQTT evey n seconds
void sendArrayOfClients (){
  int length=clients_known_count*(ETH_MAC_LEN+1+4);   //+1 for the rssi +4 time
  char message[length];
  for (int i=0; i<clients_known_count; i++){
    memcpy(&message[i*(ETH_MAC_LEN+1+4)],&clients_known[i].station,ETH_MAC_LEN);
    memcpy(&message[i*(ETH_MAC_LEN+1+4)+6],&clients_known[i].rssi,sizeof(char));
    memcpy(&message[i*(ETH_MAC_LEN+1+4)+7],&times[i],sizeof(unsigned long));
    char rssi = (~message[i*(ETH_MAC_LEN+1+4)+6]+1);
    //SerialMon.print(message[i*(ETH_MAC_LEN+1+4)+6], HEX); //SerialMon.print("->RSSI="); //SerialMon.println(String (-(rssi), DEC));
  }
  //SerialMon.println();
  for (int i=0; i<length; i++) {//SerialMon.print(message[i],HEX);//SerialMon.print("|");}
  } //SerialMon.println();
  clients_known_count=0;
  
  mqtt.publish(topicInit, message);
}

// STORE Known APs
int register_beacon(beaconinfo beacon)
{
  int known = 0;                      // Clear known flag
  for (int u = 0; u < aps_known_count; u++){
    if (! memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) {
      known = 1;
      break;
    }                                 // AP known => Set known flag
  }
  if (! known){                       // AP is NEW, copy MAC to array and return it

    memcpy(&aps_known[aps_known_count], &beacon, sizeof(beacon));
    aps_known_count++;

    if ((unsigned int) aps_known_count >=
        sizeof (aps_known) / sizeof (aps_known[0]) ) {
      //SerialMon.printf("exceeded max aps_known\n");
      aps_known_count = 0;
    }
  }
  return known;
}

//Store known clients
int register_client(clientinfo ci){
  int known = 0;   // Clear known flag

  for (int u = 0; u < clients_known_count; u++){
    if (! memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) {
      known = 1;
      break;
    }
  }
  if (! known){
    memcpy(&clients_known[clients_known_count], &ci, sizeof(ci));
    times[clients_known_count]=millis()-timed;
    clients_known_count++;

    if ((unsigned int) clients_known_count >=
        sizeof (clients_known) / sizeof (clients_known[0]) ) {
      //SerialMon.printf("exceeded max clients_known\n");
      clients_known_count = 0;
    }
  }
  return known;
}

// Print AP info on //Serial interface
void print_beacon(beaconinfo beacon)
{
  if (beacon.err != 0) {
    ////SerialMon.printf("BEACON ERR: (%d)  ", beacon.err);
  } else {
    //SerialMon.printf("BEACON: <=============== [%32s]  ", beacon.ssid);
    //for (int i = 0; i < 6; i++) //SerialMon.printf("%02x", beacon.bssid[i]);
    //SerialMon.printf("   %2d", beacon.channel);
    //SerialMon.printf("   %4d\r\n", beacon.rssi);
  }
}

// Print & Publish client info
void print_client(clientinfo ci)
{
  int u = 0;
  int messageSize = 13;
  char message [messageSize];                   // MAC + RSSI + Lat/Lon
  int known = 0;                        // Clear known flag
  String cliente = "";
  
  
  if (ci.err != 0) {
    //SerialMon.println("Error de ci.err");
  } else {
    
    
    for (int i = 0; i < 6; i++) {
           if (ci.station[i] < 0x10) {
              cliente += "0";
              cliente += String(ci.station[i], HEX);
            }else{
              cliente += String(ci.station[i], HEX);
            }
    }
    
  /*
    if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    mqttConnect();
    }
    SerialMon.println(F("Printing MAC to MQTT"));
    cliente.toCharArray(message, 12);
    message[12]='\0';
    mqtt.publish(topicInit, message);
*/
    
    SerialMon.print(clients_known_count);
    SerialMon.print(F(" clients added to array: Mac="));
    for (int i=0; i<6; i++) 
    SerialMon.print(ci.station[i], HEX);
    SerialMon.println();
  }
}

// WiFi Callback
void promisc_cb(uint8_t *buf, uint16_t len)
{
  int i = 0;
  uint16_t seq_n_new = 0;
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len == 128) {
    //SerialMon.printf("Becon received \n");
  } else {
    //SerialMon.println("Cliente detectado...");
    struct sniffer_buf *sniffer = (struct sniffer_buf*) buf;
    //Is data or QOS?
    if ((sniffer->buf[0] == 0x08) || (sniffer->buf[0] == 0x88)) {
      struct clientinfo ci = parse_data(sniffer->buf, 36, sniffer->rx_ctrl.rssi, sniffer->rx_ctrl.channel);
      if (memcmp(ci.bssid, ci.station, ETH_MAC_LEN)) {
        if (register_client(ci) == 0) {
          print_client(ci);
          nothing_new = 0;
        }
      }
    }
  }
}
