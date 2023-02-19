
//Condicionales 
  /**
   * Temp Sensor defines
   */
  #if TEMP_SENSOR_0 == -3
    #define HEATER_0_USES_MAX6675
    #define MAX6675_IS_MAX31855
    #define MAX6675_TMIN -270
    #define MAX6675_TMAX 1800
  #elif TEMP_SENSOR_0 == -2
    #define HEATER_0_USES_MAX6675
    #define MAX6675_TMIN 0
    #define MAX6675_TMAX 1024
  #elif TEMP_SENSOR_0 == -1
    #define HEATER_0_USES_AD595
  #elif TEMP_SENSOR_0 == 0
    #undef HEATER_0_MINTEMP
    #undef HEATER_0_MAXTEMP
  #elif TEMP_SENSOR_0 > 0
    #define THERMISTORHEATER_0 TEMP_SENSOR_0
    #define HEATER_0_USES_THERMISTOR
  #endif

