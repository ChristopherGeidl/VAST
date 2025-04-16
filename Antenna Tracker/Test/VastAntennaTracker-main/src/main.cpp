/*
Current Considerations:
- Motors pin out? k ########################################### IMPORTANT
- LoRa pin out? k
- What are the minimum and maximum angles?
- Do the gears reverse direction?(sign error)? k
*/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>
#include "CRC.h"
#include <iomanip>
#include <cstdint>  
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Stepper.h>

double balloonLat;
double balloonLong;
float balloonAlt;

double antennaLat;
double antennaLong;
float antennaAlt;

int setangle; //next elevation
int setheading; //next azimuth

float angle; //current elevation
float heading; //current azimuth

//for delays
unsigned long previousMillis;
unsigned long previousMillis1;

//is this correct
int azimuth_dir_pin = 2;
int azimuth_step_pin = 3;
int elevation_dir_pin = 4;
int elevation_step_pin = 5;

int STEPS = 200;

Stepper azimuth_stepper(STEPS, azimuth_dir_pin, azimuth_step_pin);
Stepper elevation_stepper(STEPS, elevation_dir_pin, elevation_step_pin);
double Stepper_X;
double Stepper_Y;

// Gear ratios for motors
double azimuthGearRatio = 149.2537;  // Gear ratio for azimuth motor
double elevationGearRatio = 150 / 1;  // Gear ratio for elevation motor
double motorDegreeToSteps = 360 / 200; // Degrees per step for stepper motor


float RadiansToDegrees(float r){
  return (r*180)/PI;
}
float DegreeToRadians(float d){
  return (d*PI)/180;
}

double R = 6378137; //m Earth's radius
double e = 0.08181919; //Earth's eccentricity
void desiredAbsolutePose(double B_LAT, double B_LONG, float B_ALT, double A_LAT, double A_LONG, float A_ALT){
  //Convert degrees to radians
  A_LAT = DegreeToRadians(A_LAT);
  A_LONG = DegreeToRadians(A_LONG);
  B_LAT = DegreeToRadians(B_LAT);
  B_LONG = DegreeToRadians(B_LONG);

  //Using Earth-Centered, Earth-Fixed (ECEF) coordinate system
  double Nb = R / sqrt(1 - e*e*sin(B_LAT)*sin(B_LAT));
  double Xb = (Nb + B_ALT) * cos(B_LAT) * cos(B_LONG);
  double Yb = (Nb + B_ALT) * cos(B_LAT) * sin(B_LONG);
  double Zb = (Nb * (1 - e*e) + B_ALT) * sin(B_LAT);

  double Na = R / sqrt(1 - e*e*sin(A_LAT)*sin(A_LAT));
  double Xa = (Na + A_ALT) * cos(A_LAT) * cos(A_LONG);
  double Ya = (Na + A_ALT) * cos(A_LAT) * sin(A_LONG);
  double Za = (Na * (1 - e*e) + A_ALT) * sin(A_LAT);

  //Relative Position in ECEF frame
  double r[3] = {(Xb-Xa), (Yb-Ya), (Zb-Za)};

  //Transform to ENU (East, North, Up) frame
  double m[3][3] = {
    {-sin(A_LONG), cos(A_LONG), 0},
    {-sin(A_LAT)*cos(A_LONG), -sin(A_LAT)*sin(A_LONG), cos(A_LAT)},
    {cos(A_LAT)*cos(A_LONG), cos(A_LAT)*sin(A_LONG), sin(A_LAT)}
  };

  double E = m[0][0]*r[0] + m[0][1]*r[1] + m[0][2]*r[2];
  double N = m[1][0]*r[0] + m[1][1]*r[1] + m[1][2]*r[2];
  double U = m[2][0]*r[0] + m[2][1]*r[1] + m[2][2]*r[2];

  double azimuth = RadiansToDegrees(atan2(E, N));
  azimuth = fmod(azimuth + 360, 360);//Ensure azimuth [0-360)

  double geometric_horizon_angle = RadiansToDegrees(atan2(U, sqrt(E*E+N*N)));
  double horizon_angle_correction = RadiansToDegrees(acos(R / (R + A_ALT)));
  double abs_elevation = geometric_horizon_angle - horizon_angle_correction;

  setheading = azimuth;
  setangle = abs_elevation;
}


//controller MAC
//78:21:84:7D:27:44
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x7D, 0x27, 0x44};
esp_now_peer_info_t peerInfo;
char WIFI_SSID[] = "router";    // Change to a non-const char array
const char* WIFI_PASS = "pass"; // You can leave this one as-is

static const uint32_t GPSBaud = 9600;  
TinyGPSPlus gps;
PacketSerial rfd900;
HardwareSerial uart2(2);
//HardwareSerial uart0(0);
HardwareSerial uart1(1);

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//recieve from RFD900
typedef struct packet {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int lora_bad_packet;
  int rfd_bad_packet;
} packet;
packet myPacket;

//data send to RFD900
typedef struct recieved_data{
    int cutdown_time;
    bool update_cutdown_time;
    bool RunTimer;
    bool trigger_cutdown;
    bool trigger_parachute;   
}recieved_data;
recieved_data txbuf;

//data share with esp
typedef struct espdata {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int lora_bad_packet;
  int rfd_bad_packet;
  int heading;
  int angle;
} espdata;
espdata espTX;

typedef struct ControllerData{
  float x;
  float y;
  char mode;
  int heading_offset;
  int angle_offset;

  int cutdown_time;
  bool update_cutdown_time;
  bool RunTimer;
  bool trigger_cutdown;
  bool trigger_parachute;
} ControllerData;
ControllerData espRX;


//Sends data to RFD900 on ardupilot
void RFDSend(){
    //update data in the txbuf
    txbuf.trigger_cutdown = espRX.trigger_cutdown;
    txbuf.RunTimer = espRX.RunTimer;
    txbuf.cutdown_time = espRX.cutdown_time;
    txbuf.update_cutdown_time = espRX.update_cutdown_time;
    txbuf.trigger_parachute = espRX.trigger_parachute;
    //send the  data
    uint32_t crc = CRC::Calculate(&txbuf, sizeof(txbuf), CRC::CRC_32());
    uint8_t payload[sizeof(txbuf)+sizeof(crc)];
    memcpy(&payload[0], &txbuf, sizeof(txbuf));
    memcpy(&payload[sizeof(txbuf)], &crc, sizeof(crc));
    rfd900.send(payload, sizeof(payload));

}
//Send esp data with ESPNow
void espNowSend(){
  // Send message via ESP-NOW 
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &espTX, sizeof(espTX));
  /* Testing Code
  if (result == ESP_ERR_ESPNOW_NO_MEM) {
    //Serial.println("Sent with success");
    //digitalWrite(2, HIGH);
    //delay(100);
    //digitalWrite(2, LOW);
  }
  else {
    //Serial.println("Error sending the data");

  }
  */
}

//Handles packet received from RFD900 on ardupilot
void RFDPacketReceived(const uint8_t* buffer, size_t size){
  uint8_t buf[size];
  memcpy(buf, buffer, size);
  uint32_t crc1 = CRC::Calculate(buf, sizeof(myPacket), CRC::CRC_32());
  uint32_t crc2;
  memcpy(&crc2, &buf[sizeof(myPacket)], sizeof(crc2));
  if(crc1 == crc2){
    memcpy(&myPacket, &buf[0], sizeof(myPacket));
    
    balloonLat = myPacket.lat;
    balloonLong = myPacket.lng;
    balloonAlt = myPacket.alt/1000;

    espTX.lat = myPacket.lat;
    espTX.lng = myPacket.lng;
    espTX.alt = myPacket.alt;
    espTX.packetcount = myPacket.packetcount;
    espTX.cutdown_time = myPacket.cutdown_time;
    espTX.timer_running = myPacket.timer_running;
    espTX.cutdown_status = myPacket.cutdown_status;
  }
  else{
    espTX.rfd_bad_packet++;
  }  
}

//Reads antenna gps and updates antenna latitude, longitude, and altitude
void read_gps(){
  if(uart1.available() > 0){
    if (gps.encode(uart1.read())){
      if (gps.location.isValid() && gps.altitude.isValid())
      {
        antennaLat = gps.location.lat();
        antennaLong = gps.location.lng();
        antennaAlt = gps.altitude.kilometers();
      }
    }
  }  
}

//Automatically called when ESPNow Data Received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&espRX, incomingData, sizeof(espRX));
  if(espRX.mode == 'm'){
    //StepSendPacket(espRX.x, espRX.y);
    Stepper_X = round(espRX.x);
    Stepper_Y = round(espRX.y);
  }
  
}
//Automattically called when ESPNow Data Sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//Serial.print("\r\nLast Packet Send Status:\t");
//Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//Setsup ESPNow for communication
void espNowSetup(){
  // Set device as a Wi-Fi Station
 
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin();
  //Serial.print(WiFi.softAPmacAddress());
  WiFi.disconnect();
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }
}

//Adjusts the position based on the magnetometer and acclerometer readings
//Sends ESPNow and RFD if necessary
void SensorRead(){
  unsigned long currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 >= 200) {
      previousMillis1 = currentMillis1;
      espNowSend();
      //calculate azmith and elevation angle
      desiredAbsolutePose(balloonLat, balloonLong, balloonAlt, antennaLat, antennaLong, antennaAlt);

      sensors_event_t a, g, temp, m;
      mag.getEvent(&m);
      heading = atan2(m.magnetic.y, m.magnetic.x)* 180/PI;
      
      heading = heading + espRX.heading_offset;
      
      
       if(heading < 0){
        heading = (180 - abs(heading)) + 180;
      }
      
      mpu.getEvent(&a, &g, &temp);
      angle = atan2(a.acceleration.x, a.acceleration.y) * 180/PI;
      angle = angle + espRX.angle_offset;
      
      if(angle < 0){
        angle = (180 - abs(angle)) + 180;
      }
      angle = angle - 90;

      espTX.angle = angle;
      espTX.heading = heading;

      if(espRX.trigger_cutdown != myPacket.cutdown_status || espRX.RunTimer != myPacket.timer_running || espRX.update_cutdown_time){
        RFDSend();
      }
      

  }
}

//Takes in the difference in actual and desired angles and outputs motor steps
void MotorControll(){
  if(espRX.mode == 'a'){
    unsigned long currentMillis = millis();
    int movex;
    int movey;
    //make sure the motor has time to move
    if (currentMillis - previousMillis >= 30) {
      previousMillis = currentMillis;

      //angleChange -> motorSpinChange -> motorSteps (round to int)
      int movex = round((angle-setangle) * azimuthGearRatio * motorDegreeToSteps);

      //bounds of movement for elevation
      if(setangle > 90){
        setangle = 90;
      }
      if(setangle < 10){
        setangle = 10;
      }

      int movey = round((angle-setangle) * elevationGearRatio * motorDegreeToSteps);
      
      //StepSendPacket(movex, movey);
      Stepper_X = movex;
      Stepper_Y = movey;
    }
  }
}
//Takes motor steps and moves motors
void StepperSend(){
  azimuth_stepper.step(Stepper_X);
  elevation_stepper.step(Stepper_Y);
}

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);

  espNowSetup();

  //uart setup
  ledcSetup(1, 1000, 10); //################################################## DONT KNOW IF THIS DOES ANYTHING ##############################################
  ledcAttachPin(25, 1);
  ledcSetup(2, 1000, 10);
  ledcAttachPin(26, 2);

  azimuth_stepper.setSpeed(500); //steps per second
  elevation_stepper.setSpeed(500); //steps per second

  //Serial.begin(57600);

  uart2.begin(57600, SERIAL_8N1, 19, 18);  
  rfd900.setStream(&uart2);
  rfd900.setPacketHandler(&RFDPacketReceived);

  //begin gps
  uart1.begin(GPSBaud, SERIAL_8N1, 17, 16);  
  
  if (!mpu.begin(105)) {
    //Serial.println("Failed to find MPU6050 chip");
    /*
    while (1) {
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(100);
    }
    */
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    //Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    /*
    while(1){
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
      delay(100);
    };
    */
  }
  //Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  espRX.mode = 'm';
  espRX.cutdown_time = 3600;
  espRX.heading_offset = 12;
  espRX.trigger_cutdown = false;
  espRX.update_cutdown_time = false;
  espRX.RunTimer = false;
}

void loop() { //########################################################### loop #################################################################
  rfd900.update();
  read_gps();
  SensorRead();
  MotorControll();
  StepperSend();
}