#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
//MQ135İ Bİ DÜŞÜN
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

const uint16_t node00 = 00;
const uint16_t node01 = 01;

#define RXLED 0
#define TXLED 0

RF24 radio(10, 9);               // nRF24L01 (CE,CSN)
RF24Network network(radio); 

uint16_t temperature = 0;
uint16_t pressure = 0;
uint16_t hum = 0;
uint16_t air= 0;

uint16_t incomingDataFromSatellite[20];

uint16_t dataToSendSatellite[10];

void setup()
{
    pinMode(A2, INPUT);
    Serial.begin(250000);
    bme.begin(0x76);
    SPI.begin();
    radio.begin();
    network.begin(90, node00);  //(channel, node address)
    radio.setDataRate(RF24_2MBPS);
}

void loop()
{
    Receive();
    SolveSerial();
    
    updateSensors();
    Transmit();
    transmitToSerial();
}

void SolveSerial()
{
    char* token;
    char tempArr[300];
    uint16_t temp[10];
    String a = Serial.readStringUntil('d');
    a.toCharArray(tempArr,300);

    dataToSendSatellite[0] = (uint16_t)(tempArr[0]- '0');
    dataToSendSatellite[1] = (uint16_t)(tempArr[1] - '0');
    dataToSendSatellite[2] = (uint16_t)(tempArr[2] - '0');
    dataToSendSatellite[3] = (uint16_t)(tempArr[3] - '0');
    dataToSendSatellite[4] = (uint16_t)(tempArr[4] - '0');
    dataToSendSatellite[5] = (uint16_t)(tempArr[5] - '0');
    dataToSendSatellite[6] = (uint16_t)(tempArr[6] - '0');
    dataToSendSatellite[7] = (uint16_t)(tempArr[7] - '0');
    dataToSendSatellite[8] = (uint16_t)(tempArr[8] - '0');
    dataToSendSatellite[9] = (uint16_t)(tempArr[9] - '0');




}

void transmitToSerial()
{
    String buff = "";
    buff = String(pressure) + "-" + String(temperature) + "-" + String(hum) + "-" + String(air);
    buff += "/";
    buff += String(incomingDataFromSatellite[0]) + "-" + String(incomingDataFromSatellite[1]) + "-" + String(incomingDataFromSatellite[2]) + "-" + String(incomingDataFromSatellite[3]) + "-" + String(incomingDataFromSatellite[4]) + "-" + String(incomingDataFromSatellite[5]) + "-" + String(incomingDataFromSatellite[6]);
    buff += "/";
    buff += String(incomingDataFromSatellite[7]) + "-" + String(incomingDataFromSatellite[8]) + "-" + String(incomingDataFromSatellite[9]) + "-" + String(incomingDataFromSatellite[10]) + "-" + String(incomingDataFromSatellite[11]) + "-" + String(incomingDataFromSatellite[12]) + "-" + String(incomingDataFromSatellite[13]) + "-" + String(incomingDataFromSatellite[14]) + "-" + String(incomingDataFromSatellite[15]) + "-" + String(incomingDataFromSatellite[16]) + "-" + String(incomingDataFromSatellite[17]) + "-" + String(incomingDataFromSatellite[18]);
    buff += "|";
    Serial.println(buff);
}

void Transmit()
{

    RF24NetworkHeader header(node01);   // (Address where the data is going)
    bool ok = network.write(header, &dataToSendSatellite, sizeof(dataToSendSatellite)); // Send the data

}

void updateSensors()
{
    air = analogRead(A2);
    temperature = bme.readTemperature();
    hum = bme.readHumidity();
    pressure = bme.readPressure(); // 100.0F
}

void Receive()
{
    network.update();
    //===== Receiving =====//
    while ( network.available() ) {     // Is there any incoming data?
        RF24NetworkHeader header;
        //blinkRX();
        network.read(header, &incomingDataFromSatellite, sizeof(incomingDataFromSatellite)); // Read the incoming data
}

}

void blinkTX()
{
    digitalWrite(TXLED, HIGH);
    delay(20);
    digitalWrite(TXLED,LOW);
}

void blinkRX()
{
    digitalWrite(RXLED, HIGH);
    delay(20);
    digitalWrite(RXLED,LOW);
}
