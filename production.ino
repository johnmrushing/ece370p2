#include <math.h>

#include <SPI.h>
#include <Wire.h>

#include <WiFi101.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

#include <LSM303.h>

#define LIR 9
#define RIR 6

#define LM1 5
#define LM2 11

#define RM1 10
#define RM2 12

#define BASELINE 95
#define WHEELRAD 15

#define BUFFSIZE 1000

#define SPEEDKP 4
#define PHIKP 4

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

char receiveBuffer[BUFFSIZE];

LSM303 compass;

// +x forward; +y downward; +z leftward

WiFiUDP udp;

unsigned int localPort = 5005;

float circRobot = 2 * PI * BASELINE;
float circWheel = 2 * PI * WHEELRAD;
float tickWheelDistance = circWheel / (75.8 * 2);
float tickPhiRad = tickWheelDistance / BASELINE;

float deltaX = ( BASELINE / 2 ) * sin( tickPhiRad );
float deltaY = ( BASELINE / 2 ) - ( ( BASELINE / 2 ) * cos( tickPhiRad ) );

float xGlobal = 0, yGlobal = 0, phiGlobalRad = 0, phiGlobalDeg = 0;

int tickCountLeft = 0, tickCountRight = 0;

float velLeft = 0, velRight = 0;
float errorLeft = 0, errorRight = 0, errorPhi = 0;

float velocityLeft = 0, velocityRight = 0, omega = 0;

float leftMotorSet = 0, rightMotorSet = 0;

float currentHeadingRad, averageHeadingRad, errorHeadingRad;
float currentHeadingDeg, averageHeadingDeg = 0, errorHeadingDeg;
float verticalAcceleration = 0;

int t1 = 0, t2 = 0;

typedef struct cmdPacket{
  double vel;
  double phi;
  int mode;
} cmdPacket;

/*
 * mode 0 = return x,y,head
 * mode 1 = driving command
 * mode 2 = cardinal command
 * mode 3 = stop
 */

cmdPacket _cmdPacket;

typedef struct rtnPacket{
  double x;
  double y;
  double head;
} rtnPacket;

rtnPacket _rtnPacket;

void leftISR()
{
  //Serial.println("leftisr");
  tickCountLeft += 1;
  phiGlobalRad -= tickPhiRad;
  phiGlobalDeg = phiGlobalRad * 180.0 / PI;
  
  xGlobal += ( ( deltaX * cos( phiGlobalRad ) ) + deltaY * sin( phiGlobalRad ) );
  yGlobal -= ( ( deltaX * sin( phiGlobalRad ) ) + deltaY * cos( phiGlobalRad ) );
}

void rightISR()
{
  //Serial.println("rightisr");
  tickCountRight += 1;
  phiGlobalRad += tickPhiRad;
  phiGlobalDeg = phiGlobalRad * 180.0 / PI;

  xGlobal += ( ( deltaX * cos( phiGlobalRad ) ) + deltaY * sin( phiGlobalRad ) );
  yGlobal += ( ( deltaX * sin( phiGlobalRad ) ) + deltaY * cos( phiGlobalRad ) );
}

void leftMotorKP( double speedLeft )
{
  velLeft = tickCountLeft / ( 2 * 75.8 );
  errorLeft = speedLeft - velLeft;

  velocityLeft = SPEEDKP * errorLeft;
}

void rightMotorKP(double speedRight)
{
  velRight = tickCountRight / ( 2 * 75.8 );
  errorRight = speedRight - velRight;

  velocityRight = SPEEDKP * errorRight;
}

void phiKP( double phiDesiredDeg )
{

  averageHeadingDeg = 0.3 * phiGlobalDeg + 0.7 * currentHeadingDeg;

  Serial.println(averageHeadingDeg);

  errorHeadingDeg = phiDesiredDeg - averageHeadingDeg;

  omega = PHIKP * errorHeadingDeg;
}

void timeUpdate()
{
  /*
   * Allows us to update our motor control value every 20 milliseconds (50 Hz)
   * This is done so we 1) don't update uselessly and
   * 2) so we still update even when the wheel is moving slowly
   */
  
  t1 = millis();
  
  if(t1 - t2 >= 20)
  {
    updateMotors();
    t2 = t1;
  }
}

void setupPins()
{
  
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);

  analogWrite(LM1, 0);
  analogWrite(LM2, 0);
  analogWrite(RM1, 0);
  analogWrite(RM2, 0);

  pinMode(LIR, INPUT_PULLUP);
  pinMode(RIR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LIR), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIR), rightISR, RISING);

  pinMode(LED_BUILTIN, OUTPUT); // pickup indicator
  digitalWrite(LED_BUILTIN, LOW);
}

void setupWiFi()
{
  WiFi.setPins(8,7,4,2);

  WiFi.begin(ssid,pass);
  
  Serial.println("wifi setup");
}

void setupPort()
{
  udp.begin(localPort);
  Serial.println("port setup");
}

void setupIMU()
{
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-2779, -3143, -2694};
  compass.m_max = (LSM303::vector<int16_t>){+3376, +2853, +3321};
  Serial.println("imu setup");
}

void pickup()
{
  digitalWrite(LED_BUILTIN, HIGH);
  analogWrite( LM1, 0 );
  analogWrite( LM2, 0 );
  analogWrite( RM1, 0 );
  analogWrite( RM2, 0 );
  while(true);
}

void updateIMU()
{
  //Serial.println("reading IMU");
  compass.read();
  currentHeadingDeg = compass.heading(); // +x axis
  currentHeadingRad = currentHeadingDeg * PI / 180.0;
  
  //Serial.println(currentHeadingDeg);

  if( compass.a.y > -8000 ) // resting is about -16000
    pickup();
  
}

void sendResponse()
{
  _rtnPacket.x = xGlobal;
  _rtnPacket.y = yGlobal;
  _rtnPacket.head = averageHeadingDeg;

  udp.beginPacket( udp.remoteIP(), udp.remotePort() );

  char trasmitBuffer[BUFFSIZE] = { 0 };

  memcpy( trasmitBuffer, &_rtnPacket, sizeof( rtnPacket ) );

  udp.write( trasmitBuffer, sizeof( rtnPacket ) );
  
  udp.endPacket();
}

void readPacket()
{
  int len = udp.read( receiveBuffer, BUFFSIZE );
  if ( len > 0 )
  {
    Serial.println("packet says: ");
    //Serial.print(receiveBuffer);
    memcpy( &_cmdPacket, receiveBuffer, sizeof( cmdPacket ) );
    /*
     * mode 0 = return x,y,head
     * mode 1 = driving command
     * mode 2 = cardinal command
     * mode 3 = stop
     */

    switch ( _cmdPacket.mode )
    {
      case 0:
        Serial.println(_cmdPacket.vel);
        Serial.println(_cmdPacket.phi);
        Serial.println("mode 0 - report");
        
        sendResponse();
        
        break;
      case 1:
        Serial.println(_cmdPacket.vel);
        Serial.println(_cmdPacket.phi);
        Serial.println("mode 1 - drive");
        //parse driving command

        
        
        break;
      case 2:
        Serial.println(_cmdPacket.vel);
        Serial.println(_cmdPacket.phi);
        Serial.println("mode 2 - proemt");
        //parse cardinal command

        
        
        break;
      case 3:
        Serial.println("mode 3");
        //stop
        pickup();
        break;
    }
  }
}

void checkUDP()
{
  int packetSize = udp.parsePacket();
  if(packetSize)
  {
    //Serial.print( "Received packet of size " );
    //Serial.println( packetSize );
    //Serial.print( "From " );
    //IPAddress remoteIp = udp.remoteIP();
    //Serial.print( remoteIp );
    //Serial.print( ", port " );
    //Serial.println( udp.remotePort() );

    readPacket();
  }
}

void updateMotors()
{
  leftMotorSet = ( ( 2 * velocityLeft ) - ( omega * BASELINE ) ) / ( 2 * WHEELRAD);
  rightMotorSet = ( ( 2 * velocityRight ) + ( omega * BASELINE ) ) / ( 2 * WHEELRAD);

  if( leftMotorSet > 255) leftMotorSet = 255;
  else if ( leftMotorSet < 0 ) leftMotorSet = 0;

  if( rightMotorSet > 255) rightMotorSet = 255;
  else if ( rightMotorSet < 0 ) rightMotorSet = 0;

  analogWrite( LM1, leftMotorSet );
  analogWrite( LM2, 0 );
  analogWrite( RM1, rightMotorSet );
  analogWrite( RM2, 0 );
}

void setup()
{

  Serial.begin(9600);
  setupIMU();
  setupPins();
  setupWiFi();
  setupPort();

  analogWrite( LM1, 0 );
  analogWrite( LM2, 0 );
  analogWrite( RM1, 0 );
  analogWrite( RM2, 0 );

  _cmdPacket.vel = 0;
  _cmdPacket.phi = currentHeadingDeg;
  _cmdPacket.mode = 0;
  
}

void loop()
{
  updateIMU();
  
  //checkUDP();

  //rightMotorKP(_cmdPacket.vel);
  //leftMotorKP(_cmdPacket.vel);

  //phiKP(_cmdPacket.phi);

  //timeUpdate();

  Serial.println(xGlobal);
}
