
#ifndef RADIO_CLIENT_H
#define RADIO_CLIENT_H

#include "RF24.h"
#include "nRF24L01.h"
#include <SPI.h>

class RadioClient {
    typedef enum {
        UNKNONWN = 0,
        YES,
        NO
    } message_stats_send;


    struct MessageStats {
        uint8_t automaticRetransmitionCount;
        uint8_t roundTrip;
        uint8_t powerLevelAmplifier;
        // lower date rate increase the distance.
        uint8_t dataRate;
        bool failureDetected = false;
        message_stats_send sent = UNKNONWN;
    };

    public:

        RadioClient(RF24 &radio,  uint8_t* writePipeAddress)
            : radio(radio), writePipeAddress(writePipeAddress) {
            };

        void init() {
            radio.setRetries(100, 15);
            radio.setPALevel(RF24_PA_MAX);
            radio.setDataRate(RF24_250KBPS);
            radio.openWritingPipe(writePipeAddress);
            radio.stopListening();
        }

        void waitRadioBegins(unsigned long intervalMs, Stream &stream) {
            stream.println(F("Waiting radio."));
            if (!radio.begin()) {
                stream.println(F("radio hw not responding!"));
                delay(intervalMs);
            }
            while (!radio.begin()) {
                stream.print(".");
                delay(intervalMs);
            }
            stream.println(F("Radio is ready."));
        }

        MessageStats send(const void *buf, size_t size) {
            unsigned long start = millis();
            MessageStats stats;

            stats.powerLevelAmplifier = 99;
            stats.dataRate = 0;

            stats.sent = radio.write(buf, size) ? YES : NO;
            stats.roundTrip = millis() - start;

            stats.failureDetected = radio.failureDetected;
            stats.automaticRetransmitionCount = radio.getARC();
            stats.powerLevelAmplifier = radio.getPALevel();
            stats.dataRate = radio.getDataRate();
            return stats;
        }


        void printStats(Stream &stream,MessageStats stats) {
            stream.print("auto retries: ");
            stream.println(stats.automaticRetransmitionCount);
            stream.print("sent: ");
            stream.println(stats.sent == YES ? "Si" : "No");

            stream.print("Roundtrip: ");
            stream.print(stats.roundTrip);
            stream.println(" ms");

            stream.print("PA Level: ");
            switch (stats.powerLevelAmplifier) {
                case RF24_PA_MIN:
                stream.println("MIN");
                break;
                case RF24_PA_LOW:
                stream.println("LOW");
                break;
                case RF24_PA_HIGH:
                stream.println("HIGH");
                break;
                case RF24_PA_MAX:
                stream.println("MAX");
                break;
            }

            stream.print("data rate: ");
            switch (stats.dataRate) {
                case RF24_250KBPS:
                stream.println("250 KBPS");
                break;
                case RF24_1MBPS:
                stream.println("1 MBPS");
                break;
                case RF24_2MBPS:
                stream.println("2 MBPS");
                break;
            }

            stream.print("Failure: ");
            stream.println(stats.failureDetected ? "Yes" : "No");
        }
    private:
        RF24 radio;
        uint8_t *writePipeAddress;


};

#endif