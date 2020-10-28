#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <BH1750FVI.h> 
#include <mlx90615.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

#define POWERLIGHT 0
#define BULB 0

// left one first
#define M1A 7
#define M1B 8
#define M2A 5
#define M2B 6

#define TRIG 4
#define ECHO 3

#define SERVOPIN 2
long duration;
int distance;



uint16_t lux = 0;

uint16_t horn = 0;
uint16_t otemperature = 0;
uint16_t gyro_x = 0;
uint16_t gyro_y = 0;
uint16_t gyro_z = 0;
uint16_t acc = 0;
uint16_t dist = 0;
uint16_t air = 0;
uint16_t angle = 0;
uint16_t light = 0;
uint16_t velocity = 0;
uint16_t power = 1;
uint16_t dataToSatellite[23];
uint16_t incomingDataFromSatellite[8];


Servo myServo;
MLX90615 mlx = MLX90615();
BH1750FVI LightSensor;
MPU6050 mpu6050(Wire);

RF24 radio(10, 9);               // nRF24L01 (CE,CSN)
RF24Network network(radio);      // Include the radio in the network
const uint16_t node01 = 01;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t node00 = 00;      // Address of the other node in Octal format
const uint16_t node02 = 011;


void setup() {
  Serial.begin(250000);

    pinMode(M1A, OUTPUT);
    pinMode(M1B, OUTPUT);
    pinMode(M2A, OUTPUT);
    pinMode(M2B, OUTPUT);
    
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    
    pinMode(POWERLIGHT, OUTPUT);
    pinMode(BULB, OUTPUT);

    pinMode(A2, INPUT);

    myServo.attach(SERVOPIN);

   
    mlx.begin();

    LightSensor.begin();
    LightSensor.SetAddress(Device_Address_H);
    LightSensor.SetMode(Continuous_H_resolution_Mode);
    
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    
    SPI.begin();
    radio.begin();
    network.begin(90, node02);  //(channel, node address)
    radio.setDataRate(RF24_2MBPS);


    dataToSatellite[0] = 0;
    dataToSatellite[1] = 0;
    dataToSatellite[2] = 0;
    dataToSatellite[3] = 0;
    dataToSatellite[4] = 0;
    dataToSatellite[5] = 0;
    dataToSatellite[6] = 0;
    dataToSatellite[7] = 0;
    dataToSatellite[8] = 0;
    dataToSatellite[9] = 0;
    
    
}





void loop() {
    for(int i=0;i<=180; i = i + 3){
        
        Receive();  
        myServo.write(i);
        angle = i;
        distance = calculateDistance();
        updateSensors();
        Transmit();
        engineControl(incomingDataFromSatellite[1],incomingDataFromSatellite[2],incomingDataFromSatellite[3],incomingDataFromSatellite[4]);
     
        
    }
    // Repeats the previous lines from 165 to 15 degrees
    for(int i=180;i>0;i = i - 3){  
        Receive();  
        myServo.write(i);
        angle=i;
        distance = calculateDistance();
        updateSensors();
        Transmit();
        engineControl(incomingDataFromSatellite[1],incomingDataFromSatellite[2],incomingDataFromSatellite[3],incomingDataFromSatellite[4]);
       
        
    }
    
}

void updateSensors()
{
    lux = LightSensor.GetLightIntensity();
    otemperature = mlx.get_object_temp();
    air = analogRead(A2);
    mpu6050.update();
    dist = distance;
    
    gyro_x = mpu6050.getGyroX();
    gyro_y = mpu6050.getGyroY();
    gyro_z = mpu6050.getGyroZ();
    acc = mpu6050.getAccX();
    velocity = computeVelocity();

    dataToSatellite[10] = dist;
    dataToSatellite[11] = angle;
    dataToSatellite[12] = air;
    dataToSatellite[13] = lux;
    dataToSatellite[14] = velocity;
    dataToSatellite[15] = acc;
    dataToSatellite[16] = gyro_x;
    dataToSatellite[17] = gyro_y;
    dataToSatellite[18] = gyro_z;
    dataToSatellite[19] = otemperature;
    dataToSatellite[20] = light;
    dataToSatellite[21] = power;
}

uint16_t computeVelocity()
{
    uint16_t x1,y1,z1, x2,y2,z2,mean1,mean2;

    mpu6050.update();
    x1 = mpu6050.getGyroX();
    y1 = mpu6050.getGyroY();
    z1 = mpu6050.getGyroZ();

    mean1 = (x1+y1+z1)/3;

    delay(10);

    mpu6050.update();
    x1 = mpu6050.getGyroX();
    y1 = mpu6050.getGyroY();
    z1 = mpu6050.getGyroZ();

    mean2 = (x2+y2+z2)/3;

    return (mean2-mean1)/50;
}

void engineControl(uint16_t ahead, uint16_t reverse, uint16_t right, uint16_t left)
{
    if(ahead == 1 )
    {
        digitalWrite(M1A, HIGH);
        digitalWrite(M1B, LOW);
        digitalWrite(M2A, HIGH);
        digitalWrite(M2B, LOW);
    }
    else if(reverse == 1)
    {
        digitalWrite(M1A, LOW);
        digitalWrite(M1B, HIGH);
        digitalWrite(M2A, LOW);
        digitalWrite(M2B, HIGH);
    }
    else if(right == 1)
    {
        digitalWrite(M1A, HIGH);
        digitalWrite(M1B, LOW);
        digitalWrite(M2A, LOW);
        digitalWrite(M2B, HIGH);
    }
    else if(left == 1)
    {
        digitalWrite(M1A, LOW);
        digitalWrite(M1B, HIGH);
        digitalWrite(M2A, HIGH);
        digitalWrite(M2B, LOW);
    }
    else
    {
        digitalWrite(M1A, LOW);
        digitalWrite(M1B, LOW);
        digitalWrite(M2A, LOW);
        digitalWrite(M2B, LOW);
    }
    
}

/*void chkHorn(uint16_t horn)
{

}
*/

void Receive()
{
    network.update();
    //===== Receiving =====//
    while ( network.available() ) {     // Is there any incoming data?
        RF24NetworkHeader header;
        network.read(header, &incomingDataFromSatellite, sizeof(incomingDataFromSatellite)); // Read the incoming data

  }
}

void Transmit()
{
    RF24NetworkHeader header2(node01);     // (Address where the data is going)
    bool ok = network.write(header2, &dataToSatellite, sizeof(dataToSatellite)); // Send the data
}

int calculateDistance(){ 
  
  digitalWrite(TRIG, LOW); 
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance= duration*0.034/2;
  return distance;
}
