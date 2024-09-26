
#include <SoftwareSerial.h>



namespace steibPms5003s {

    struct SensorData {
        uint16_t framelen;
        uint16_t pm10_standard, pm25_standard, pm100_standard;
        uint16_t pm10_env, pm25_env, pm100_env;
        uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
        uint16_t reserved; // Byte reserved by protocol.
        uint16_t checksum;
    };


    class AirQualitySensor {

        public:
            AirQualitySensor(Stream& sensorStream)
                : pmsStrem(sensorStream) {};

            bool readPMSdata(Stream *s, SensorData &data);

        private:
            Stream& pmsStrem;
            void parseStream();
    };

}