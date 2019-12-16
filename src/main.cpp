#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <pixart.h>
#include <timer.h>
#include <ui.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
#include <EEPROM.h>

const char  *thisDevice = "spacecam";
IPAddress ROSCORE(192,168,0,200);

#define EEPROM_SALT 12662
typedef struct {
  char  ROS_MASTER_URI[33]  = "192.168.0.200";
  char  ROS_PORT[6]         = "11411";
  int   salt                = EEPROM_SALT;
} WMSettings;

WMSettings settings;
WiFiManagerParameter ros_master_text("<br/>ROS MASTER IP:");
WiFiManagerParameter ros_master_url("ROS MASTER", "IP", settings.ROS_MASTER_URI, 33);
WiFiManagerParameter ros_port_text("<br/>SERVER PORT:");
WiFiManagerParameter ros_master_port("ROS PORT", "PORT", settings.ROS_PORT, 6);

Pixart          camera;
Timer           pub_timer;
userInterface   ui;

ros::NodeHandle nh;

enum attributes { x=0, y=1, s=2 };

std_msgs::UInt16MultiArray blobMultiArray;
std_msgs::UInt16MultiArray rangeMultiArray;
std_msgs::MultiArrayDimension mad[2];
char label_pt[]    = "points";
char label_coord[] = "attributes";


ros::Publisher pub_blobs( "/spacecam/trackpoints", &blobMultiArray);

String getSubString(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void saveConfigCallback () {
  strcpy(settings.ROS_MASTER_URI, ros_master_url.getValue());
  strcpy(settings.ROS_PORT, ros_master_port.getValue());
  EEPROM.begin(512);
  EEPROM.put(0, settings);
  EEPROM.end();
}

void configModeCallback (WiFiManager *myWiFiManager) {
}

bool APisConnected(){
  if (WiFi.status() == WL_CONNECTED){
    ui.wifiConnected();
    return true;
  } else {
    ui.wifiDisConnected();
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  ui.init();
  camera.init();

  WiFi.hostname(thisDevice);
  WiFiManager wifiManager;

  EEPROM.begin(512);
  EEPROM.get(0, settings);
  EEPROM.end();
  if (settings.salt != EEPROM_SALT) {
    WMSettings defaults;
    settings = defaults;
  }

  wifiManager.addParameter(&ros_master_text);
  wifiManager.addParameter(&ros_master_url);
  wifiManager.addParameter(&ros_port_text);
  wifiManager.addParameter(&ros_master_port);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(180);

  if (!wifiManager.autoConnect(thisDevice)) {
    ESP.reset();
    delay(1000);
  }
  if (APisConnected()) {
    ROSCORE.fromString(settings.ROS_MASTER_URI);
    nh.getHardware()->setConnection(ROSCORE, atoi(settings.ROS_PORT));
    nh.initNode();
    nh.advertise(pub_blobs);
  }

  mad[0].label  = label_pt;
  mad[0].size   = 4;
  mad[0].stride = 3*4;
  mad[1].label  = label_coord;
  mad[1].size   = 3;
  mad[1].stride = 3;

  blobMultiArray.layout.dim_length = 2;
  blobMultiArray.layout.dim = mad;
  blobMultiArray.layout.data_offset = 0;
  blobMultiArray.data = (uint16_t *)malloc(sizeof(uint16_t)*3*4);
  blobMultiArray.data_length = 3*4;
}

void loop() {

  if ( ui.buttonIsPressed() ) {
    WMSettings defaults;
    settings = defaults;
    WiFiManager wifiManager;
    wifiManager.resetSettings();
    wifiManager.setTimeout(180);
    ui.configMode();
    if (!wifiManager.startConfigPortal(thisDevice)) {
      ESP.reset();
      delay(1000);
    }
  } else {
    // run
    ui.wifiConnected();
    nh.spinOnce();
    if (pub_timer.poll(10)){
      camera.getBlobs();
      for (uint8_t pt=0; pt < 4; pt++){
        blobMultiArray.data[pt*3+x] = camera.blobArray[pt].X;
        blobMultiArray.data[pt*3+y] = camera.blobArray[pt].Y;
        blobMultiArray.data[pt*3+s] = camera.blobArray[pt].Size;
      }
      pub_blobs.publish( &blobMultiArray );
    }
  }
}
