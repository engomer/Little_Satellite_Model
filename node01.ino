#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

#include <BH1750FVI.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
BH1750FVI LightSensor;
MPU6050 mpu6050(Wire);

RF24 radio(10, 9);               // nRF24L01 (CE,CSN)
RF24Network network(radio);      // Include the radio in the network
const uint16_t node01 = 01;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t node00 = 00;      // Address of the other node in Octal format
const uint16_t node02 = 011;

void setup() {

  bme.begin(0x76);
  Serial.begin(250000);
  
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_H);
  LightSensor.SetMode(Continuous_H_resolution_Mode);
  
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  SPI.begin();
  radio.begin();
  network.begin(90, node01);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
    
}
int flag = 0;
uint16_t incomingData[22];

uint16_t goToLight = 0;
uint16_t connectToCar = 0;
uint16_t lux = 1;
uint16_t temperature = 1;
uint16_t pressure = 2;
uint16_t hum = 3;
uint16_t gyro_x = 1;
uint16_t gyro_y = 1;
uint16_t gyro_z = 1;
uint16_t dataToGroundStation[20];
uint16_t dataToCar[8];
void loop() {
    Receive();
    updateSensors();
    chk(flag);
    TransmitCar();
    TransmitGS();
}

void updateSensors()
{
    lux = LightSensor.GetLightIntensity();
    temperature = bme.readTemperature();
    hum = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    mpu6050.update();
    gyro_x = mpu6050.getGyroX();
    gyro_y = mpu6050.getGyroY();
    gyro_z = mpu6050.getGyroZ();
}

void Receive()
{
    network.update();
    //===== Receiving =====//
    while ( network.available() ) {     // Is there any incoming data?
        RF24NetworkHeader header;
        network.read(header, &incomingData, sizeof(incomingData)); // Read the incoming data
        //Serial.println(header.from_node);
        if(header.from_node == 9)
        {
          flag = 0;
          
        }
        else if(header.from_node == 0)
        {
          flag = 1;
        }
  }
}

void chk(int flag)
{
  if(flag==1)
  {
    goToLight = incomingData[0];
          connectToCar = incomingData[1];
          //dataToCar[0] = incomingData[2];
          dataToCar[0] = 1;
          dataToCar[1] = incomingData[3];
          dataToCar[2] = incomingData[4];
          dataToCar[3] = incomingData[5];
          dataToCar[4] = incomingData[6];
          //dataToCar[5] = incomingData[7];
          dataToCar[5] = 1;
          dataToCar[6] = incomingData[8];
          dataToCar[7] = 1;
          

          Serial.println("Data From Satellite: ");
          
          for(int i= 0; i < 9; i++)
          {
            Serial.print(incomingData[i]);
            Serial.print("-");
          }
          Serial.println("\n");
  }
  if(flag==0)
  {
    dataToGroundStation[0] = pressure;
          dataToGroundStation[1] = temperature;
          dataToGroundStation[2] = hum;
          dataToGroundStation[3] = gyro_x;
          dataToGroundStation[4] = gyro_y;
          dataToGroundStation[5] = gyro_z;
          dataToGroundStation[6] = lux;
          dataToGroundStation[7] = incomingData[10];
          dataToGroundStation[8] = incomingData[11];
          dataToGroundStation[9] = incomingData[12];
          dataToGroundStation[10] = incomingData[13];
          dataToGroundStation[11] = incomingData[14];
          dataToGroundStation[12] = incomingData[15];
          dataToGroundStation[13] = incomingData[16];
          dataToGroundStation[14] = incomingData[17];
          dataToGroundStation[15] = incomingData[18];
          dataToGroundStation[16] = incomingData[19];
          dataToGroundStation[17] = incomingData[20];
          dataToGroundStation[18] = incomingData[21];
          
          Serial.println("Data From Car: ");
          for(int i=7; i<19; i++)
          {
            Serial.print(dataToGroundStation[i]);
            Serial.print("-");
          }
          Serial.println("\n");
          
  }
}

void TransmitGS()
{
    RF24NetworkHeader header2(node00);     // (Address where the data is going)
    bool ok = network.write(header2, &dataToGroundStation, sizeof(dataToGroundStation)); // Send the data


}

void TransmitCar()
{
    RF24NetworkHeader header4(node02);    // (Address where the data is going)
    bool ok3 = network.write(header4, &dataToCar, sizeof(dataToCar)); // Send the data

}
