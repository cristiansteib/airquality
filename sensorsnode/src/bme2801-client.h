#ifndef BME_2801_CLIENT_H
#define BME_2801_CLIENT_H
#include <BME280I2C.h> 

class BMETemperatureSensorClient {

    public:

    void waitBMEBegins(unsigned long intervalMs, Stream &stream) {
        while(!bme.begin()){
            stream.println("Could not find BME280 sensor!");
            delay(intervalMs);
        } 
    }

    void printChipModel(Stream &stream){
        switch(bme.chipModel()) {
            case BME280::ChipModel_BME280:
            stream.println(F("Found BME280 sensor! Success."));
            break;
            case BME280::ChipModel_BMP280:
            stream.println(F("Found BMP280 sensor! No Humidity available."));
            break;
            default:
            stream.println(F("Found UNKNOWN sensor! Error!"));
        }
    }

    /**
     * @brief Retrieves temperature, pressure, and humidity from the BME280 sensor.
     *
     * @param[out] pressurePa Pressure in Pascals (Pa).
     * @param[out] tempC Temperature in Celsius (°C).
     * @param[out] humidityRh Relative humidity in percentage (%RH).
     */
    void read(float& pressurePa, float& tempC, float& humedityRh) {
        bme.read(
            pressurePa,
            tempC,
            humedityRh,
            BME280::TempUnit_Celsius,
            BME280::PresUnit_Pa
        );
    }

    private:
        // Default : forced mode, standby time = 1000 ms
        // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
        BME280I2C bme;    
        
};

#endif
