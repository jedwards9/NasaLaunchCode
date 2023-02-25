/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory


// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22



int pictureNumber = 0;
char code = 0;
String picTime;
String message;
int pic_num = 0;
bool flipON = 0;
bool received = 0;
camera_config_t config;
esp_err_t err;
camera_fb_t *fb = NULL;

void setup() {
  // put your setup code here, to run once:
  pic_Setup();
  Serial.begin(115200);
}

void loop() {
  //put your main code here, to run repeatedly:
//  pic_Setup();
//  delay(50);
//  take_Pic("Test");
  //Serial.begin(115200);
  received = 0;
  while(received == 0){
    if(Serial.available()){
        Serial.println("Message Received: ");
        message = Serial.readStringUntil('\n');
        Serial.println(message);
        if(message.substring(2,9) != "PICTURE"){
          received = 1;
          return;
        }
        code = message[0];
        picTime = message.substring(9,message.length()-1);
        received = 1;
        delay(100);
    }
    else
      Serial.println("Nothing Yet.");  
  }
  //Serial.println(picTime);
  //delay(500);
  Serial.println(message.substring(2,9));
  if(message.substring(2,9) == "PICTURE"){
    take_Pic(picTime);
  }
  delay(10000);
  sensor_t * s = esp_camera_sensor_get();

  /*
  switch(code){
    case 1:
      take_Pic(picTime);
      break;
    case 2:
      s->set_special_effect(s, 2); //Grayscale
      break;
    case 3:
      s->set_special_effect(s, 0); //Back to Color
      break;
    case 4:
      s->set_special_effect(s, 4); //Apply Filter
      break;
    case 5:
      s->set_special_effect(s, 0); //Remove Filter
      break;
     case 6:
      if(flipON){
        s->set_vflip(s, 0); //Turn Flip OFF
      }
      else{
        s->set_vflip(s, 1); //Turn Flip ON
      }
      flipON = !flipON;
      break;
      
  }*/
  //Serial.end();
  delay(500);
}
    


//  if(pic_num <= 3){
//    take_Pic();
//    if(pic_num == 0){
//      sensor_t * s = esp_camera_sensor_get();
//    //s->set_vflip(s, 1);
//      s->set_special_effect(s, 2);  
//    }
//    if(pic_num == 1){
//      sensor_t * s = esp_camera_sensor_get();
//    s->set_vflip(s, 1);
//      s->set_special_effect(s, 0);  
//    }
//    if(pic_num == 2){
//      sensor_t * s = esp_camera_sensor_get();
//    s->set_vflip(s, 0);
//      s->set_special_effect(s, 4);  //0 – No Effect
//    }
//    
//    pic_num = pic_num + 1;          //1 – Negative
//    }                               //2 – Grayscale
//}                                   //3 – Red Tint
//                                    //4 – Green Tint
//                                    //5 – Blue Tint
//                                    //6 – Sepia



void take_Pic(String picTime){
fb = esp_camera_fb_get();  
          if(!fb) {
            Serial.println("Camera capture failed");
            return;
          }
          // initialize EEPROM with predefined size
        EEPROM.begin(EEPROM_SIZE);
        pictureNumber = EEPROM.read(0) + 1;
      
        // Path where new picture will be saved in SD Card
        String path = "/" + picTime +".jpg";
      
        fs::FS &fs = SD_MMC; 
        Serial.printf("Picture file name: %s\n", path.c_str());
        
        File file = fs.open(path.c_str(), FILE_WRITE);
        if(!file){
          Serial.println("Failed to open file in writing mode");
        } 
        else {
          file.write(fb->buf, fb->len); // payload (image), payload length
          Serial.printf("Saved file to path: %s\n", path.c_str());
          EEPROM.write(0, pictureNumber);
          EEPROM.commit();
          
        }
        file.close();
        esp_camera_fb_return(fb);
}



void pic_Setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  //Serial.begin(115200);
  //Serial.setDebugOutput(true);
  //Serial.println();
  
  //camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }
}
