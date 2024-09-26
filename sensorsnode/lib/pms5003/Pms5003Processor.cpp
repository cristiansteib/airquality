#include "Pms5003Processor.h"

namespace steibPms5003s {


        bool AirQualitySensor::readPMSdata(Stream *s, steibPms5003s::SensorData &data) {
        if (s->available() < 32) {
            return false;
        }

        // Verify the first bytes message (0x42 y 0x4D) to considere is correctly routed
        if (s->read() != 0x42 || s->read() != 0x4D) {
            s->flush();
            return false;
        }


        uint8_t buffer[30];
        if (s->readBytes(buffer, 30) != 30) {
            // Couldn't read all bytes
            return false;
        }

        // Caculate the checksum
        uint16_t checksum = 0x42 + 0x4D;
        for (uint8_t i = 0; i < 28; i++) {
            checksum += buffer[i];
        }

        // Adjust to bytes big-endian
        data.framelen = (buffer[0] << 8) | buffer[1];
        data.pm10_standard = (buffer[2] << 8) | buffer[3];
        data.pm25_standard = (buffer[4] << 8) | buffer[5];
        data.pm100_standard = (buffer[6] << 8) | buffer[7];
        data.pm10_env = (buffer[8] << 8) | buffer[9];
        data.pm25_env = (buffer[10] << 8) | buffer[11];
        data.pm100_env = (buffer[12] << 8) | buffer[13];
        data.particles_03um = (buffer[14] << 8) | buffer[15];
        data.particles_05um = (buffer[16] << 8) | buffer[17];
        data.particles_10um = (buffer[18] << 8) | buffer[19];
        data.particles_25um = (buffer[20] << 8) | buffer[21];
        data.particles_50um = (buffer[22] << 8) | buffer[23];
        data.particles_100um = (buffer[24] << 8) | buffer[25];
        data.reserved = (buffer[26] << 8) | buffer[27];
        data.checksum = (buffer[28] << 8) | buffer[29];

        if (data.framelen != 28) {
            // Invalid frame length");
            return false;
        }

        if (checksum != data.checksum) {
            return false;
        }
        return true;
    }
}
