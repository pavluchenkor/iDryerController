
#include <Arduino.h>
#include <stdint.h>
#include <src/Configuration.h>
#include <src/Conditionals.h>
#include <src/Thermistortables.h>
#include <thermistor.h>

#define HOTENDS  1
#define ARRAY_6(v1, v2, v3, v4, v5, v6, ...) { v1, v2, v3, v4, v5, v6 }
#define ARRAY_5(v1, v2, v3, v4, v5, ...)     { v1, v2, v3, v4, v5 }
#define ARRAY_4(v1, v2, v3, v4, ...)         { v1, v2, v3, v4 }
#define ARRAY_3(v1, v2, v3, ...)             { v1, v2, v3 }
#define ARRAY_2(v1, v2, ...)                 { v1, v2 }
#define ARRAY_1(v1, ...)                     { v1 }
#define _ARRAY_N(N, ...) ARRAY_ ##N(__VA_ARGS__)
#define ARRAY_N(N, ...) _ARRAY_N(N, __VA_ARGS__)
#define ARRAY_BY_HOTENDS(...) ARRAY_N(HOTENDS, __VA_ARGS__)
#define ARRAY_BY_HOTENDS1(v1) ARRAY_BY_HOTENDS(v1, v1, v1, v1, v1, v1)
#define PGM_RD_W(x)   (short)pgm_read_word(&x)

static void* heater_ttbl_map[] = ARRAY_BY_HOTENDS((void*)HEATER_0_TEMPTABLE, (void*)HEATER_1_TEMPTABLE, (void*)HEATER_2_TEMPTABLE, (void*)HEATER_3_TEMPTABLE, (void*)HEATER_4_TEMPTABLE);
static uint8_t heater_ttbllen_map[] = ARRAY_BY_HOTENDS(HEATER_0_TEMPTABLE_LEN, HEATER_1_TEMPTABLE_LEN, HEATER_2_TEMPTABLE_LEN, HEATER_3_TEMPTABLE_LEN, HEATER_4_TEMPTABLE_LEN);

thermistor::thermistor(int pin, int sensorNumber)
{
  pinMode(pin, INPUT);
  _pin = pin;
  _sensorNumber = sensorNumber;
}
float thermistor::analog2temp() {
  uint8_t e = _sensorNumber;
  int raw  =  0;
  for(int j=1;j<=OVERSAMPLENR;j++){
    raw += analogRead(_pin);
  }
  if (heater_ttbl_map[e] != NULL) {
    float celsius = 0;
    uint8_t i;
    short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map[e]);
    for (i = 1; i < heater_ttbl_map[e]; i++) {
      if (PGM_RD_W((*tt)[i][0]) > raw) {
        celsius = PGM_RD_W((*tt)[i - 1][1]) +
                  (raw - PGM_RD_W((*tt)[i - 1][0])) *
                  (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i - 1][1])) /
                  (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i - 1][0]));
        break;
      }
    }
    
    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]){ celsius = PGM_RD_W((*tt)[i - 1][1]);}

    return celsius;
  }
}
