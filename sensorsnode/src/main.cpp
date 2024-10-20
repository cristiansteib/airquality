#include <Arduino.h>
#include <SoftwareSerial.h>

#include "RF24.h"
#include "nRF24L01.h"
#include "../shared/protocol.h"

//#define DEBUG
#include "../shared/pms5003/Pms5003Processor.h"
#include <Wire.h>
#include "radio-client.h"
#include "bme2801-client.h"

#define NODE_ID 1


SoftwareSerial PmsSerial(2, 3);
constexpr size_t max_handlers = 1;


steibPms5003s::AirQualitySensor<max_handlers> qualitySensor(PmsSerial);


#define RADIO_CE_PIN 7
#define RADIO_CSN_PIN 8


uint8_t writeAddress[][6] = { "1Node", "2Node" };
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
RadioClient radioClient(
  radio,
  writeAddress[0]
);

BMETemperatureSensorClient bmeClient;


void sensorHandler(steibPms5003s::SensorData *data) {
  RadioPackage package = createRadioPackage(
    MessageKind::pms5003,
    NODE_ID,
    data,
    sizeof(steibPms5003s::SensorData)
  );
  radioClient.send(&package, sizeof(package));
}


void getAndSendTemp() {
  BME data;
  data.humedityRh = NAN;
  data.pressurePa = NAN;
  data.tempC = NAN;
  bmeClient.read(data.pressurePa, data.tempC , data.humedityRh);
  RadioPackage package = createRadioPackage(
    MessageKind::temperature,
    NODE_ID,
    &data,
    sizeof(BME)
  );
  radioClient.send(&package, sizeof(package));
}


void setup() {
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  PmsSerial.begin(9600);  // PMS sensor Serial port
  Wire.begin(); // Initialize For i2c
  Serial.begin(9600); // Serial port for debugging purposes.

  // Note: 
  //    The radio client must start functioning before anything else
  //    to properly handle sensoR measurement data.
  radioClient.waitRadioBegins(1000, Serial);
  radioClient.init();

  bmeClient.waitBMEBegins(1000, Serial);
  bmeClient.printChipModel(Serial);

  qualitySensor.addObserver(&sensorHandler);
}


void conditionalSendTemperatureData() {
  static unsigned long last_send = 0;
  if (millis() - last_send > 2000) {
      last_send = millis();
      getAndSendTemp();
  }
}

void loop() {
  // Note: 
  //    The PMS sensor's Serial port may enter an overflow state.
  //    To recover from this, all bytes in the buffer are consumed,
  //    meaning all data in the buffer will be discarded.
  //    This PMS processor (AirQualitySensor) attempts to align frames
  //    in case of incorrect starting headers, but this may not be 
  //    sufficient if a buffer overflow occurs.
  //    This management is here couse AirQualitySensor only works with 
  //    the Stream interface and not with SoftwareSerial interface.
  if (PmsSerial.overflow()) {
      while (PmsSerial.available()){
      PmsSerial.read();
      }
      PmsSerial.flush();
  }
  qualitySensor.loop();
  conditionalSendTemperatureData();
}
