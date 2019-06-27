
/* LoRa to read DS18b20 sensors temperature (watertemp) and send using proprietary protocol to home server
   Programmed to run on a Adafruit Feather 32u4 LoRa RFxx board (433mhz)
   Author: Martin Kristensen
   Version: 1.1
   Date: 18.06.2019

   With default settings, the sensor will send a temperature reading every 10 min for the first 1½ hour - and then once every 6 hours after that.
*/

#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SleepyDog.h>

#define TempSensor  18                                  // on pin D18 (a 4.7K resistor is necessary) DS18S20 signal pin connection to Feather 32u4 RFM95 LoRa Radio board

#define LED         13                                  // pin definition for LED on Feather 32u4 RFM95 LoRa Radio board

#define SS      8   // LoRa Radio pin connections on Feather 32u4 RFM95 LoRa Radio board
#define RST     4
#define DI0     7
#define BAND    433E6

#define VBATPIN     A9                                  // analog pin on Feather 32u4 RFM95 LoRa Radio board used to monitor power supply (battery) voltage
                                                        // Note: no need to connect any other pins through resistors etc. - as this is built in to the Feather board.


typedef union
{
  float floatingPoint;                                  // this union allows floting point numbers to be sent as binary representations over the wireless link
  byte binary[sizeof(floatingPoint)];                   // "sizeof" function takes care of determining the number of bytes needed for the floating point number
} binaryFloat;

binaryFloat waterTemp;                                  // ***************************************************************************
binaryFloat batVolt;                                    // ***************************************************************************


const byte stationID = B00000001;                       // WaterTemp Station ID

byte wirelessPacket[9];                                 // Defines the size of the packet to be sent by the Remote Stations: 2 floats x 4 bytes per float + 1 byte for Remote Station ID

int debugswitch = 0;                                    // Debug = 1 overrides normal sleep pattern and reads+sends messages every 10 sec. Debug = 0 runs the sensor in regular production mode.
int loopIncrease = 1;                                   // Special counter for smart send behaviour
int number_of_sleeper_loops = 0;

void setup()
{
  //  Serial.begin(9600);
  //  while (!Serial) {
  //    delay(1);
  //  }
  //  Serial.println("Booting Feather LoRa Sensor.. (DS18b20 on PIN 18)");

  // while (!Serial);
  LoRa.setPins(SS, RST, DI0);
  //LoRa.setTxPower(23);
  //LoRa.setSpreadingFactor(9); // Default is 7
  if (!LoRa.begin(BAND)) {
    //  Serial.println("ERROR: Starting LoRa failed!");
    while (1);
  }
  //  Serial.println("LoRa init ok");


  delay(100);


  readSensors();                                        // read sensors
  delay(1000);                                          // wait one second

  Watchdog.enable(8000);              // Enable watchdog timer for 8 second reset

}


void loop()
{
  if (debugswitch == 1)  // Trigger and send every 5 sec
  {
    number_of_sleeper_loops = 38; //time between taking a reading is a bit more than 5 min.
  }
  else
  {
    if (loopIncrease < 10) {
      number_of_sleeper_loops = 75; //time between taking a reading is 10 min.
      loopIncrease++;
    } else {
      number_of_sleeper_loops = 2700; //time between taking a reading is 6 hours.
    }
  }

  Watchdog.reset();
  readSensors();
  buildWirelessPacket();
  sendWirelessPacket();

  for (int i = 0; i < number_of_sleeper_loops; i++) {
    Watchdog.sleep(8000);
  }

}

void readSensors(void)  /* reads all sensor values */
{
  waterTemp.floatingPoint = getTemp(TempSensor);        // read water temperature
  batVolt.floatingPoint = battVoltage();                // read battery or power supply voltage

}

void buildWirelessPacket(void)  /* builds wireless packet to be transmitted by LoRa Radio */
{

  for (int n = 0; n < 4; n++)                           // read binary values from binaryFloat union struct into wireless packet
  {
    wirelessPacket[n] = waterTemp.binary[n];
    wirelessPacket[n + 4] = batVolt.binary[n];
  }
  wirelessPacket[8] = stationID;                       // add Remote Station ID to wireless packet

}


void sendWirelessPacket() /* wakes up LoRa Radio and transmits wireless packet */
{
  LoRa.beginPacket();
  LoRa.write(wirelessPacket, sizeof(wirelessPacket));
  LoRa.endPacket();
  LoRa.sleep();
  //  Serial.println("Sent LoRa Package...");
  delay(10);

}


float getTemp(int DS18S20_Pin)  /* read DS18S20 temperature sensor */
{
  OneWire ds(DS18S20_Pin);                              // set signal pin

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr))
  {
    ds.reset_search();
    if ( !ds.search(addr))                              // try a second time if first try fails
    {
      ds.reset_search();
      return -1000;
    }
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    //   Serial.println("CRC is not valid!");
    return -2000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    //   Serial.print("Device is not recognized");
    return -3000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);                                    // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);                                       // read scratchpad


  for (int i = 0; i < 9; i++) {                         // get 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB);                  // using two's compliment
  float TemperatureSum = tempRead / 16;

  return (TemperatureSum);                              // return temperature in degrees C

}

float battVoltage(void)
{
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;                                    // divided by 2 on the Feather Board, so multiply back
  measuredvbat *= 3.3;                                  // multiply by 3.3V, our reference voltage
  measuredvbat /= 1024;                                 // convert to voltage
  return (measuredvbat);
}
