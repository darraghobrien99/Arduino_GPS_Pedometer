#include <TinyGPS++.h>   
#include <Arduino.h>
#include "wiring_private.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include "Firebase_Arduino_WiFiNINA.h"
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

int status = WL_IDLE_STATUS;

WiFiSSLClient client;

//Firebase settings
#define FIREBASE_HOST "dog-tracker-b4357-default-rtdb.europe-west1.firebasedatabase.app"
#define FIREBASE_AUTH "v9M4GL83W7qGZ0i9VPagYfsI39riwHJU1RKSUyfh"
//#define WIFI_SSID "TP-LINK_3848"
//#define WIFI_PASSWORD "52211331"
#define WIFI_SSID "Redmi"
#define WIFI_PASSWORD "12345678"
#define CLEAR_STEP      true
#define NOT_CLEAR_STEP  false


//Create a instance of class LSM6DS3
LSM6DS3 pedometer(I2C_MODE, 0x6A);    //I2C device address 0x6A

FirebaseData firebaseData;
FirebaseData stepData;

    float lat,lng; 
    int counter = 0;     
    int startStepCount;
    //uint16_t stepCount2;
    Uart gpsSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
    TinyGPSPlus gps;

    void SERCOM0_Handler()
    {
    gpsSerial.IrqHandler();
    }

    void setup(){

      pinPeripheral(5, PIO_SERCOM_ALT);
      pinPeripheral(6, PIO_SERCOM_ALT);
      Serial.begin(9600); // connect serial
      while(!SerialUSB &&  (millis() < 20000));

      if (pedometer.begin() != 0) {
       Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
   }

    //Configure LSM6DS3 as pedometer
    if (0 != config_pedometer(NOT_CLEAR_STEP)) {
        Serial.println("Configure pedometer fail!");
    }
   Serial.println("Success to Configure pedometer!");


      if(WiFi.status() == WL_NO_MODULE){
        Serial.println("Communication failed");
        while(true);
      }

      connect_to_wifi();
      Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);



  if (Firebase.getInt(firebaseData, "/steps")) {

    //Success, then read the payload value

    //Make sure payload value returned from server is integer
   //This prevent you to get garbage data
   if (stepData.dataType() == "int") {
     startStepCount = stepData.intData();
     Serial.println("Start Step Count " + startStepCount);
      Firebase.setInt(firebaseData, "/steps", startStepCount);
    }
    

  } else {
    //Failed, then print out the error detail
    Serial.println(firebaseData.errorReason());
  }

 
  
      Firebase.reconnectWiFi(true);
      gpsSerial.begin(9600); // connect gps sensor

    }

  //Setup pedometer mode
int config_pedometer(bool clearStep) {
    uint8_t errorAccumulator = 0;
    uint8_t dataToWrite = 0;  //Temporary variable

   //Setup the accelerometer******************************
   dataToWrite = 0;

   //  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
   dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
   dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;


    // Step 1: Configure ODR-26Hz and FS-2g
   errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

   // Step 2: Set bit Zen_G, Yen_G, Xen_G, FUNC_EN, PEDO_RST_STEP(1 or 0)
    if (clearStep) {
       errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
   } else {
        errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);
    }

    // Step 3:  Enable pedometer algorithm
   errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

   //Step 4: Step Detector interrupt driven to INT1 pin, set bit INT1_FIFO_OVR
   errorAccumulator += pedometer.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);

    return errorAccumulator;
}
  

  

    void connect_to_wifi(){
      while(status != WL_CONNECTED){
        Serial.print("Attempting to connect");
        Serial.print(WIFI_SSID);

        status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        delay(1000);
        Serial.println("Connected");
      }
      }

     
    void loop(){
      if(WiFi.status() != WL_CONNECTED){
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }
      while(gpsSerial.available()){ // check for gps data
       if(gps.encode(gpsSerial.read())){
        if(gps.location.isUpdated()){
           float lat =  (gps.location.lat());
           float lng = (gps.location.lng());

           Serial.print("Latitude = ");
           Serial.println(lat, 7);
           Serial.print("Longitude = ");
           Serial.println(lng, 7);
           
           char location_str[25] = {0};
          // store string of lat,lng
          sprintf(location_str, "%f, %f", lat, lng);
          String lat_string = String(lat, 8);
          String lng_string = String(lng, 8);
          String location_string = (lat_string + "," + lng_string);
          Firebase.setString(firebaseData,"/location/lat", lat_string);
          Firebase.setString(firebaseData,"/location/lng", lng_string);


       }
      }
    }

   

   uint8_t dataByte = 0;
   uint16_t stepCount2 = startStepCount;

   pedometer.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
   stepCount2 = (dataByte << 8) & 0xFFFF;

   pedometer.readRegister(&dataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
   stepCount2 |=  dataByte;

    Serial.print("Step: ");
    Serial.println(stepCount2);
    Firebase.setInt(firebaseData,"/steps",stepCount2);

 
}
