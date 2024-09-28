

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define PRINT_FRAME_HEX(frame, length)            \
    do {                                          \
        Serial.print("Frame data in hex: ");      \
        for (size_t i = 0; i < length; i++) {     \
            if ((frame)[i] < 0x10) {              \
                Serial.print('0');                \
            }                                     \
            Serial.print((frame)[i], HEX);        \
            Serial.print(' ');                    \
        }                                         \
        Serial.println();                         \
    } while (0)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define PRINT_FRAME_HEX(frame, length)
#endif


#ifndef AIR_QUALITY_SENSOR_H
#define AIR_QUALITY_SENSOR_H

namespace steibPms5003s {

    struct SensorData {
        uint16_t pm10_standard, pm25_standard, pm100_standard;
        uint16_t pm10_env, pm25_env, pm100_env;
        uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
        uint16_t reserved; // Byte reserved by protocol.

        void reset() {
            pm10_standard = 0;
            pm25_standard = 0;
            pm100_standard = 0;
            pm10_env = 0;
            pm25_env = 0;
            pm100_env = 0;
            particles_03um = 0;
            particles_05um = 0;
            particles_10um = 0;
            particles_25um = 0;
            particles_50um = 0;
            particles_100um = 0;
            reserved = 0;
        }

        String toString() const {
            String result = "PM10 (std): " + String(pm10_standard) + "\n";
            result += "PM2.5 (std): " + String(pm25_standard) + "\n";
            result += "PM100 (std): " + String(pm100_standard) + "\n";
            result += "PM10 (env): " + String(pm10_env) + "\n";
            result += "PM2.5 (env): " + String(pm25_env) + "\n";
            result += "PM100 (env): " + String(pm100_env) + "\n";
            result += "p > 0.3um: " + String(particles_03um) + "\n";
            result += "p > 0.5um: " + String(particles_05um) + "\n";
            result += "p > 1.0um: " + String(particles_10um) + "\n";
            result += "p > 2.5um: " + String(particles_25um) + "\n";
            result += "p > 5.0um: " + String(particles_50um) + "\n";
            result += "p > 10um: " + String(particles_100um) + "\n";
            return result;
        }
    };

    template <size_t max_handlers>
    class AirQualitySensor {
        public:
            static constexpr uint8_t FRAME_STARTING_BYTE_1 = 0x42;
            static constexpr uint8_t FRAME_STARTING_BYTE_2 = 0x4D;
            static constexpr uint8_t FRAME_LENGHT = 32;

            AirQualitySensor(Stream& sensorStream)
                : sensorStream(sensorStream) {
                    static_assert(max_handlers > 0, "max_handlers size must be greater than 0");
            };


            void loop() {
                steibPms5003s::SensorData data = {};
                int retry = 3;
                uint8_t buffer[FRAME_LENGHT];
                while (retry--) {
                    if (this->readPMSdata(data, buffer)) {
                        this->notifyNewValue(data);
                        break;
                    }
                    delay(500);
                    DEBUG_PRINTLN("--retry");
                }
            }

            void addObserver(void (*fn)(SensorData *data) ) {
                if (observersCount < max_handlers) {
                    this->observers[0] = fn;
                    observersCount++;
                }
            }


        private:
            Stream& sensorStream;
            void (*observers[max_handlers])(SensorData *data);
            size_t observersCount;
    
            /**
             * @brief Reads a complete frame from the sensor, assuming the initial bytes have been read correctly
             * 
             * This private method reads an entire frame from the sensor and stores the data in a provided buffer.
             * @param frame The pointer to the buffer where the raw data frame will be stored.
             * @param data Reference to a SensorData structure where the processed sensor values will be saved.
             * 
             * @return true if the frame wasa read and processed successfully, false otherwise.
             */
            bool readFrame(uint8_t *frame, steibPms5003s::SensorData &data) {
                PRINT_FRAME_HEX(frame, FRAME_LENGHT);
                uint16_t checksum = 0;
                for (size_t i = 0; i < FRAME_LENGHT -2; i++) { // avoid sum the last 2 bytes (checksun)
                    checksum += frame[i];
                }
                // ensure data is clean;
                data.reset();

                if (frame[0] != AirQualitySensor::FRAME_STARTING_BYTE_1 || frame[1] != AirQualitySensor::FRAME_STARTING_BYTE_2) {
                    DEBUG_PRINTLN("Frame header is incorrect!");
                    return false;
                }

                uint16_t framelen = (frame[2] << 8) | frame[3]; // Bytes 2 y 3 para la longitud del frame (según protocolo).

                if (framelen != 28) {
                    // Longitud de frame inválida
                    DEBUG_PRINT("invalid framelen ");
                    DEBUG_PRINTLN(framelen);
                    return false;
                }

                data.pm10_standard = (frame[4] << 8) | frame[5];
                data.pm25_standard = (frame[6] << 8) | frame[7];
                data.pm100_standard = (frame[8] << 8) | frame[9];
                data.pm10_env = (frame[10] << 8) | frame[11];
                data.pm25_env = (frame[12] << 8) | frame[13];
                data.pm100_env = (frame[14] << 8) | frame[15];
                data.particles_03um = (frame[16] << 8) | frame[17];
                data.particles_05um = (frame[18] << 8) | frame[19];
                data.particles_10um = (frame[20] << 8) | frame[21];
                data.particles_25um = (frame[22] << 8) | frame[23];
                data.particles_50um = (frame[24] << 8) | frame[25];
                data.particles_100um = (frame[26] << 8) | frame[27];
                data.reserved = (frame[28] << 8) | frame[29]; // Bytes 28 y 29 (Reservado)
                uint16_t expectedChecksum = (frame[30] << 8) | frame[31]; // Bytes 30 y 31 (Checksum)

                if (checksum != expectedChecksum) {
                    DEBUG_PRINTLN("Bad checksum ");
                    DEBUG_PRINT(expectedChecksum);
                    DEBUG_PRINT(" != ");
                    DEBUG_PRINTLN(checksum);
                    return false;
                }

                return true;
            }

            bool resynchronizeStream() {
                bool found = false;
                DEBUG_PRINT("Resynchronizing: discarding bytes...");

                // Mientras no se haya encontrado el byte de inicio y haya datos en el buffer
                while (!found && this->sensorStream.available()) {
                    if (this->sensorStream.peek() == AirQualitySensor::FRAME_STARTING_BYTE_1) {
                        // Si encontramos el primer byte de inicio, intentamos leer el segundo byte
                        this->sensorStream.read(); // Consumir el primer byte
                        if (this->sensorStream.peek() == AirQualitySensor::FRAME_STARTING_BYTE_2) {
                            // Encontramos el encabezado completo, estamos alineados
                            this->sensorStream.read(); // Consumir el segundo byte
                            found = true;
                            DEBUG_PRINTLN("OK");
                        }
                    } else {
                        // discard byte
                        this->sensorStream.read();
                    }
                }

                DEBUG_PRINTLN("end");

                if (!found) {
                    DEBUG_PRINTLN("Desynchronized");
                    this->sensorStream.flush();
                }

                return found;
            }

            bool readPMSdata(steibPms5003s::SensorData &data, uint8_t *buffer) {
                if (this->sensorStream.available() < AirQualitySensor::FRAME_LENGHT) {
                    DEBUG_PRINTLN("Not enough data available.");
                    return false;
                }

                // Check if the buffer contains more data than the frame size. This could 
                // occur if the stream is not read at regular intervals.
                // Failing to read the stream periodically may result in buffer overflow
                // and desynchronization.

                // Resynchronize if there are more bytes than the frame size in the buffer
                if (this->sensorStream.available() > AirQualitySensor::FRAME_LENGHT) {
                    if (!resynchronizeStream()) {
                        return false;
                    }

                    // Verificar que haya datos suficientes después de la resincronización
                    if (this->sensorStream.available() < AirQualitySensor::FRAME_LENGHT -2) {

                        DEBUG_PRINT("Not enough data available after resynchronization. ");
                        DEBUG_PRINTLN(this->sensorStream.available());

                        return false;
                    }

                    buffer[0] = AirQualitySensor::FRAME_STARTING_BYTE_1;
                    buffer[1] = AirQualitySensor::FRAME_STARTING_BYTE_2;
                    if (this->sensorStream.readBytes(&buffer[2], FRAME_LENGHT-2) != FRAME_LENGHT-2) {
                        // Couldn't read all bytes
                        DEBUG_PRINT("failed to extract all frame bytes after sync");
                        return false;
                    }
                    PRINT_FRAME_HEX(buffer, FRAME_LENGHT);
                } else {
                    if (this->sensorStream.readBytes(buffer, FRAME_LENGHT) != FRAME_LENGHT) {
                        // Couldn't read all bytes
                        DEBUG_PRINT("failed to extract all frame bytes");
                        return false;
                    }
                    PRINT_FRAME_HEX(buffer, FRAME_LENGHT);
                }

                return this->readFrame(buffer, data);
            }


            /**
             * @brief Notify each observer about the data obtained from the sensor.
             * 
             * This private methods is responsible to call each handler to store or read the data.
             * @param data The reference to the struct where the last data is available.
             * 
             * @return true if the frame wasa read and processed successfully, false otherwise.
             */
            void notifyNewValue(steibPms5003s::SensorData &data) {
                for (uint8_t idx = 0; idx < observersCount; ++idx){
                    if (this->observers[idx] !=nullptr){
                        this->observers[idx](&data);                
                    }
                }
            }
        };
}

#endif