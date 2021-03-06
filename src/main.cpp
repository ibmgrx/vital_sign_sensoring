/**
 * @file main.cpp
 * @author Ibrahim (ibrahim.io@yahoo.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SparkFun_GridEYE_Arduino_Library.h>
#include <SharpIR.h>
#include <MAX30100_PulseOximeter.h>
#include <DHT.h>

#define ERROR            1
#define OK               0
#define PIN_SharpIR      A0
#define PIN_ECG_DATA     A1
#define PIN_Sound        6
#define PIN_ECG_PLUS     9
#define PIN_ECG_MIN      10
#define PIN_DHT          11
#define I2C_SPEED_FAST   400000
#define INIT_OK          0
#define INTERVAL_SPO2    1000
#define INTERVAL_GET     100
#define INTERVAL_PRINT   10

// #define DHTTYPE DHT11
#define DHTTYPE DHT22
//#define DHTTYPE DHT21

#define AMG
// #define MLX

const int ledPin              = LED_BUILTIN;
int ledState                  = LOW;
unsigned long previousMillis  = 0,
              previousMillis2 = 0,
              previousMillis3 = 0; 

//GLOBAL VARIABLE
String deviceName = "Sensoring_v2";
bool      state_led           = 0;
uint8_t   status_init         = 0;
uint8_t   data_pHigh          = 0,
          data_pLow           = 0;
uint16_t  data_ecg            = 0, 
          data_sound          = 0;
float     data_temperature    = 0.0,
          data_distance       = 0.0,
          data_envTemp        = 0.0,
          data_envHum         = 0.0;
int16_t   data_irLed          = 0,
          data_redLed         = 0;
uint16_t  dist                = 0, 
          sampling_dist       = 0;

//INSTANCE
#ifdef MLX
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#endif
#ifdef AMG
GridEYE grideye;
#endif

SharpIR sensor( SharpIR::GP2Y0A21YK0F, PIN_SharpIR );
DHT dht(PIN_DHT, DHTTYPE);
PulseOximeter pox;

//PROTOTYPE
uint8_t init_TempSens();
uint8_t init_DistSens();
uint8_t init_ECGSens();
uint8_t init_HeartSPO2Sens();
uint8_t init_EnvSens();
uint8_t init_SoundSens();
void print_Data(void);
void get_TempSens(float*value);
void get_DistSens(float*value);
void get_ECGSens(uint16_t *value, uint8_t *value2, uint8_t *value3);
void get_HeartSPO2Sens(int16_t*value1, int16_t*value2);
void get_EnvSens(float*value1, float*value2);
void get_SoundSens(uint16_t*value);

void setup() {
    pinMode(ledPin, OUTPUT);
    Serial.begin(115200);
    Serial.println("[PROJECT]    : Vital Sensing");
    status_init += init_TempSens();
    status_init += init_DistSens();
    status_init += init_ECGSens();
    status_init += init_HeartSPO2Sens();
    status_init += init_EnvSens();
    status_init += init_SoundSens();
    if(status_init == 0){
      Serial.println("[STATUS]     : DEVICE READY");
    }
    else{
      Serial.println("[STATUS]     : DEVICE ERROR");
    }
    delay(100);
}

void loop() {
  pox.update();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= INTERVAL_GET) {
      previousMillis = currentMillis;  
      get_TempSens(&data_temperature);
      get_DistSens(&data_distance);
      get_ECGSens(&data_ecg, &data_pHigh, &data_pLow);
      get_EnvSens(&data_envTemp, &data_envHum);
      get_SoundSens(&data_sound);
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
    }
    if (currentMillis - previousMillis3 >= INTERVAL_SPO2){
      previousMillis3 = currentMillis;
      get_HeartSPO2Sens(&data_irLed, &data_redLed);
    }
    if (currentMillis - previousMillis2 >= INTERVAL_PRINT) {
      previousMillis2 = currentMillis; 
      print_Data();
    }
}
//am2302
void print_Data(void){
    Serial.println("{\"deviceName\":\"" + deviceName +
                "\", \"temp\":\"" + String(data_temperature) +
                "\", \"dist\":\"" + String(data_distance) +
                "\", \"spo2\":\"" + String(data_irLed) +
                "\", \"bpm\":\"" + String(data_redLed) +
                "\", \"ecg\":\"" + String(data_ecg) +
                "\", \"pHigh\":\"" + String(data_pHigh) +
                "\", \"pLow\":\"" + String(data_pLow) +
                "\", \"envTemp\":\"" + String(data_envTemp) +
                "\", \"envHum\":\"" + String(data_envHum) +
                "\", \"mic\":\"" + String(data_sound) +
                "\", \"errorCode\":\"" + String(status_init) +
                "\"}");
}

uint8_t init_TempSens(){
    Serial.println("==== INIT HEAD TEMP SENS ====");
    #ifdef MLX
    if (!mlx.begin()) {
      Serial.println("[STATUS]     : HEAD TEMP ERROR");
      return ERROR;
    }
    #endif
    #ifdef AMG
    grideye.begin();
    #endif
    
    return OK;
}
void get_TempSens(float *value){
  #ifdef MLX
    // float ambientTempC = mlx.readAmbientTempC();
    float objectTempC  = mlx.readObjectTempC();
    // float ambientTempF = mlx.readAmbientTempF();
    // float objectTempF  = mlx.readObjectTempF();
    *value = objectTempC;
    if (isnan(*value)){
      *value = 0;
    }
  #endif
  
  #ifdef AMG
  float testPixelValue = 0;
  float hotPixelValue = 0;
  for(unsigned char i = 0; i < 64; i++){
    testPixelValue = grideye.getPixelTemperature(i);
      if(testPixelValue > hotPixelValue){
        hotPixelValue = testPixelValue;
      }
  }
  *value = hotPixelValue;
  if (isnan(*value)){
    *value = 0;
  }
  #endif

}

uint8_t init_DistSens(){
    Serial.println("==== INIT DISTANCE SENS ====");
    pinMode(PIN_SharpIR, INPUT);
    return OK;
}
void get_DistSens(float *value){
    for (uint8_t i = 0; i<20; i++){
      dist = sensor.getDistance();
      sampling_dist += dist;
    }
    *value = sampling_dist/20;
    sampling_dist = 0;
}

uint8_t init_ECGSens(){
    Serial.println("==== INIT ECG SENS ====");
    pinMode(PIN_ECG_PLUS, INPUT);
    pinMode(PIN_ECG_MIN, INPUT);
    pinMode(PIN_ECG_DATA, INPUT);
    return OK;
}
void get_ECGSens(uint16_t *value, uint8_t *value2, uint8_t *value3){
    uint8_t p_high = digitalRead(PIN_ECG_PLUS);
    uint8_t p_low = digitalRead(PIN_ECG_MIN);
    *value2 = p_high;
    *value3 = p_low;
    if((p_high == 1)||(p_low == 1)) *value = 0;
    else *value = analogRead(A0);
}

uint8_t init_HeartSPO2Sens(){
    Serial.println("==== INIT SPO2 SENS ====");
   if (!pox.begin()) {
        Serial.println("ERR");
        return ERROR;
    } else {
        Serial.println("OK");
    }
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    return OK;
}

void get_HeartSPO2Sens(int16_t *value1, int16_t *value2){
  // *value1 = random(95,100);
  // *value2 = random(80,100);
  // *value1 = particleSensor.getRed();
  // *value2 = particleSensor.getIR();
  *value1 = pox.getSpO2();
  if(*value1>100)*value1=100;
  *value2 = pox.getHeartRate();
}

uint8_t init_EnvSens(){
    Serial.println("==== INIT DHT SENS ====");
    dht.begin();
    return OK;
}
void get_EnvSens(float*value1, float*value2){
  *value1 = dht.readTemperature();
  *value2 = dht.readHumidity();
  if (isnan(*value1) || isnan(*value2)) {
    *value1 = 0;
    *value2 = 0;
  }
}

uint8_t init_SoundSens(){
  pinMode(PIN_Sound,INPUT);
  return 0;
}
void get_SoundSens(uint16_t*value){
    *value = digitalRead(PIN_Sound);
}