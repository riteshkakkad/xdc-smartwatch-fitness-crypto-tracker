#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <lvgl.h>
#include <CST816S.h>
#include <ui/ui.h>
#include <TFT_eSPI.h>
#include <WiFiManager.h>
#include "time.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "ArduinoJson.h"
#include "SensorQMI8658.hpp"
#include <vector>

using namespace std;

#define ENABLE_LOGGING 1

#define SENSOR_SDA  6
#define SENSOR_SCL  7
#define IMU_INT1  4
#define IMU_INT2  3

#define TFT_MISO -1
// #define TFT_MISO 12
#define TFT_MOSI 11 // In some display driver board, it might be written as "SDA" and so on.
#define TFT_SCLK 10
#define TFT_CS   9  // Chip select control pin
#define TFT_DC   8  // Data Command control pin
#define TFT_RST  14  // Reset pin (could connect to Arduino RESET pin)
#define TFT_BL   2  // LED back-light

#define TOUCH_SDA 6
#define TOUCH_SCL 7
#define TOUCH_RST 13
#define TOUCH_IRQ 5

#define BAT_VOLTAGE_PIN 1
#define Measurement_offset 0.992857  

// 10 seconds in us
uint64_t DISPLAY_ON_TIME = 10 * 1000000;

// crypto API parameters
const char*  server = "https://pro-api.coinmarketcap.com/v2/cryptocurrency/quotes/latest?id=2634";
const char*  apiKey = "125646da-9cb9-431c-a815-e3d9e0c6f831";
const char* test_root_ca= \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n" \
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n" \
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n" \
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n" \
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n" \
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n" \
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n" \
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n" \
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n" \
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n" \
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n" \
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n" \
"rqXRfboQnoZsG4q5WTP468SQvvG5\n" \
"-----END CERTIFICATE-----\n";

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

#define SCREEN_BUFFER_SIZE (240 * 240)  
enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight /5 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);
CST816S touch(6, 7,13,5);	// sda, scl, rst, irq
SensorQMI8658 qmi;
IMUdata acc;
IMUdata gyr;

// parameters for NTP Time
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 1;

// wifi credentials
String ssid = "";
String password = "";
bool isInitialSetup = false;
bool firstTImeStartDisplayTimeout = true;

// Display timeout timer
bool displayTimeout = false;
bool canDisplayTurnOff = false;
const esp_timer_create_args_t display_timeout_timer_args = {
  .callback = [](void *arg) { 
    displayTimeout = true;
  },
  .name = "display timeout timer"
};
esp_timer_handle_t dispay_timeout_timer;

struct homeScreenCommand{
  uint8_t command; // 0 : time/date/battery/steps/temp, 1:cryptoRate
  uint8_t hour;
  uint8_t minute;
  uint8_t day;
  uint8_t month;
  uint8_t weekday;
  uint8_t ampm;
  int stepCount;
  String batteryLvl;
  String cryptoRate;
  String percent_change_24h;
  bool is_percent_change_24h_negative;
};

struct menuScreenCommand{
  uint8_t command; 
  uint8_t temp;
};  

struct wifiScreenCommand{
  uint8_t command; // 0 - text, 1 - arc, 2 - back button
  char* text;
  lv_color_t textColor;
  bool hideCancelBtn;
  uint8_t arcValue;
  lv_color_t arcColor;
};

struct guiCommand {
  uint8_t command; // 0 - screen change, 1 - home screen, 2 - menu screen, 3 - wifi screen , 4 - set canDisplayTurnOff
  homeScreenCommand homeScreen;
  menuScreenCommand menuScreen;
  wifiScreenCommand wifiScreen;
  lv_screen_load_anim_t moveDir;
  uint8_t screen;
  uint8_t setOndemardWifi;
};

struct commandArray{
  std::vector <guiCommand> commands;
  uint8_t size;
};

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

// UI Mutex
SemaphoreHandle_t xMutex;

// task handlers
TaskHandle_t handleOnDemand;
TaskHandle_t controlTaskHandle;

void* allocate_psram(size_t size);
void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap);
static uint32_t my_tick_get_cb (void);
void GUITask(void *pvParameters);
void GUIControlTask(void *pvParameters);
void ControlTask(void *pvParameters);
void onDemandWiFiTask(void *pvParameters);
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data);

static void increase_lvgl_tick(void* arg) {
  lv_tick_inc(portTICK_PERIOD_MS);
}

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */

#ifdef ENABLE_LOGGING
  Serial.println("Hello Arduino!");

  Serial.print("Heap size: ");
  Serial.println(ESP.getHeapSize());
  Serial.print("Heap free size: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("PSRAM size: ");
  Serial.println(ESP.getPsramSize());
  Serial.print("PSRAM free size: ");
  Serial.println(ESP.getFreePsram());
#endif

  // TODO: check later
  // // turn off the Bluetooth
  // btStop();

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);

  // initialize the UI mutex
  xMutex = xSemaphoreCreateMutex();

  // create GUI task runs in code 0 
  xTaskCreatePinnedToCore(GUITask, "GUITask", 28096, NULL, 3, NULL, 0);
  // delay(500);
  // xTaskCreatePinnedToCore(GUIUpdateTask, "GUIUpdateTask", 8096, NULL, 3, NULL, 1);
}

void loop()
{
  vTaskDelete(NULL);
  // delay(1000);
}

/*Set tick routine needed for LVGL internal timings*/
static uint32_t my_tick_get_cb (void) { return millis(); }

/* Display flushing */
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap)
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  TFT_eSPI* tftDriver = (TFT_eSPI *)lv_display_get_driver_data(disp);

  tftDriver->startWrite();
  tftDriver->setAddrWindow( area->x1, area->y1, w, h );
  tftDriver->pushColors( ( uint16_t * )pixelmap, w * h, true );
  tftDriver->endWrite();


  lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data)
{
  // uint16_t touchX = 0, touchY = 0;

  bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

  CST816S *touchA = (CST816S *)lv_indev_get_driver_data(indev_driver);


  // touchX = touchA->data.x;
  // touchY = touchA->data.y;

  touched = touchA->available();

  if (!touched)
  {
      data->state = LV_INDEV_STATE_REL;
  }
  else
  { 
    if(esp_timer_is_active(dispay_timeout_timer)){
      esp_timer_stop(dispay_timeout_timer);
    }
    esp_timer_start_once(dispay_timeout_timer, DISPLAY_ON_TIME);
    
    data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
    data->point.x = 230 - touchA->data.y;
    data->point.y = touchA->data.x ;
  }
}

void* allocate_psram(size_t size) {
  void* ptr = ps_malloc(size);
  return ptr;
}

void GUITask(void *pvParameters){
  // for update the tempurature from the sensor
  unsigned long currentTime = 0;
  unsigned long previousTime = 0;
  unsigned long tempUpdateInterval = 3000;
  // set backlight LED pin as output
  pinMode(TFT_BL, OUTPUT);

  bool displayHasTurnedOff = false;

  // initialize TFT
  tft.init();          /* TFT init */
  tft.setRotation(3); /* Landscape orientation, flipped */

  touch.begin();

  lv_init();

  /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(my_tick_get_cb);

  #if LV_USE_LOG != 0
      lv_log_register_print_cb( my_print ); /* register print function for debugging */
  #endif

  // lv_color_t* buf1 = (lv_color_t*)allocate_psram(SCREENBUFFER_SIZE_PIXELS);
  // lv_color_t* buf2 = (lv_color_t*)allocate_psram(SCREENBUFFER_SIZE_PIXELS);

  static lv_disp_t* disp;
  disp = lv_display_create( screenWidth, screenHeight );
  lv_display_set_buffers( disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb( disp, my_disp_flush );
  // lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_driver_data( disp, &tft );

  static lv_indev_t* indev;
  indev = lv_indev_create();
  lv_indev_set_type( indev, LV_INDEV_TYPE_POINTER );
  lv_indev_set_read_cb( indev, my_touchpad_read );
  lv_indev_set_driver_data( indev, &touch );

  //Dim the TFT backlight
  analogWrite(TFT_BL, 255/2); // values go from 0 to 255,

  ui_init();

#ifdef ENABLE_LOGGING
  Serial.println();
  Serial.println();
  Serial.println("Setup done");
  Serial.println();
  Serial.println();
  Serial.print("Heap size: ");
  Serial.println(ESP.getHeapSize());
  Serial.print("Heap free size: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("PSRAM size: ");
  Serial.println(ESP.getPsramSize());
  Serial.print("PSRAM free size: ");
  Serial.println(ESP.getFreePsram());
#endif

#ifdef ENABLE_LOGGING
  Serial.println("Setup done");
#endif

  #if IMU_INT1 > 0
      qmi.setPins(IMU_INT1);
  #endif

  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
    Serial.println("Failed to find QMI8658 - check your wiring!");
    while (1) {
        delay(1000);
    }
  }

#ifdef ENABLE_LOGGING
  /* Get chip id*/
  Serial.print("Device ID:");
  Serial.println(qmi.getChipID(), HEX);
#endif

  if (qmi.selfTestAccel()) {
      Serial.println("Accelerometer self-test successful");
  } else {
      Serial.println("Accelerometer self-test failed!");
  }

  if (qmi.selfTestGyro()) {
      Serial.println("Gyroscope self-test successful");
  } else {
      Serial.println("Gyroscope self-test failed!");
  }

  qmi.configAccelerometer(
      /*
      * ACC_RANGE_2G
      * ACC_RANGE_4G
      * ACC_RANGE_8G
      * ACC_RANGE_16G
      * */
      SensorQMI8658::ACC_RANGE_4G,
      /*
      * ACC_ODR_1000H
      * ACC_ODR_500Hz
      * ACC_ODR_250Hz
      * ACC_ODR_125Hz
      * ACC_ODR_62_5Hz
      * ACC_ODR_31_25Hz
      * ACC_ODR_LOWPOWER_128Hz
      * ACC_ODR_LOWPOWER_21Hz
      * ACC_ODR_LOWPOWER_11Hz
      * ACC_ODR_LOWPOWER_3H
      * */
      SensorQMI8658::ACC_ODR_1000Hz,
      /*
      *  LPF_MODE_0     //2.66% of ODR
      *  LPF_MODE_1     //3.63% of ODR
      *  LPF_MODE_2     //5.39% of ODR
      *  LPF_MODE_3     //13.37% of ODR
      *  LPF_OFF        // OFF Low-Pass Fitter
      * */
      SensorQMI8658::LPF_MODE_0);

  qmi.configGyroscope(
      /*
      * GYR_RANGE_16DPS
      * GYR_RANGE_32DPS
      * GYR_RANGE_64DPS
      * GYR_RANGE_128DPS
      * GYR_RANGE_256DPS
      * GYR_RANGE_512DPS
      * GYR_RANGE_1024DPS
      * */
      SensorQMI8658::GYR_RANGE_64DPS,
      /*
      * GYR_ODR_7174_4Hz
      * GYR_ODR_3587_2Hz
      * GYR_ODR_1793_6Hz
      * GYR_ODR_896_8Hz
      * GYR_ODR_448_4Hz
      * GYR_ODR_224_2Hz
      * GYR_ODR_112_1Hz
      * GYR_ODR_56_05Hz
      * GYR_ODR_28_025H
      * */
      SensorQMI8658::GYR_ODR_896_8Hz,
      /*
      *  LPF_MODE_0     //2.66% of ODR
      *  LPF_MODE_1     //3.63% of ODR
      *  LPF_MODE_2     //5.39% of ODR
      *  LPF_MODE_3     //13.37% of ODR
      *  LPF_OFF        // OFF Low-Pass Fitter
      * */
      SensorQMI8658::LPF_MODE_3);

  /*
  * If both the accelerometer and gyroscope sensors are turned on at the same time,
  * the output frequency will be based on the gyroscope output frequency.
  * The example configuration is 896.8HZ output frequency,
  * so the acceleration output frequency is also limited to 896.8HZ
  * */
  qmi.enableGyroscope();
  qmi.enableAccelerometer();

  // Print register configuration information
  qmi.dumpCtrlRegister();

  #if IMU_INT1 > 0
  // If you want to enable interrupts, then turn on the interrupt enable
  qmi.enableINT(SensorQMI8658::INTERRUPT_PIN_1, true);
  qmi.enableINT(SensorQMI8658::INTERRUPT_PIN_2, false);
  #endif

#ifdef ENABLE_LOGGING
  Serial.println("Read data now...");
#endif

  // create the control task
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 18096, NULL, 5, &controlTaskHandle, 1);

  esp_timer_create(&display_timeout_timer_args, &dispay_timeout_timer);
  
  newDisplayOnTime = false;

  while(1){
    // take ui mutex
    xSemaphoreTake(xMutex, portMAX_DELAY);
    // update the tempurature from the sensor
    currentTime = millis();
    if(currentTime - previousTime > tempUpdateInterval){
      previousTime = currentTime;
      float temp = qmi.getTemperature_C();
      // set the temperature
      char tempStr[10];
      sprintf(tempStr, "%d°C", (int)temp);
      lv_label_set_text(ui_tempLbl, tempStr);
    }

    lv_timer_handler(); /* let the GUI do its work */

    if(newDisplayOnTime){
      newDisplayOnTime = false;
      // print the slider value
      uint32_t value =  lv_roller_get_selected(ui_Roller1);
      Serial.printf("New display on time: %d\n", value);
      switch(value){
        case 0:
          DISPLAY_ON_TIME = 10 * 1000000; // 10s
          break;
        case 1:
          DISPLAY_ON_TIME = 30 * 1000000; // 30s
          break;
        case 2:
          DISPLAY_ON_TIME = 60 * 1000000; // 1min
          break;
        case 3:
          DISPLAY_ON_TIME = 300 * 1000000; // 5min
          break;
        default:
          break;
      }

      canDisplayTurnOff = true;

    }

    if(canDisplayTurnOff){
      canDisplayTurnOff = false;
      Serial.println("Display timer started");
      if(esp_timer_is_active(dispay_timeout_timer)){
        esp_timer_stop(dispay_timeout_timer);
      }
      // start timer for 10s
      esp_timer_start_once(dispay_timeout_timer, DISPLAY_ON_TIME);
    }

    if(displayTimeout){
      Serial.println("Display timeout");
      displayTimeout = false;
      analogWrite(TFT_BL, 0);
      _ui_screen_change(&ui_HomeScn, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_HomeScn_screen_init);
      displayHasTurnedOff = true;
    }

    if(displayHasTurnedOff){
      qmi.getAccelerometer(acc.x, acc.y, acc.z);
      if(touch.available() || (acc.x>-0.01 && acc.x<0.20 && acc.y>0.40 && acc.y<0.8 && acc.z>-0.9 && acc.z<-0.7)){
        displayHasTurnedOff = false;
        uint32_t brightness = lv_slider_get_value(ui_brightnessSlider);
        // convert that value to 0-255 range
        brightness =	(brightness * 255 / 100);
        
        if(brightness < 30) brightness = 30;
        analogWrite(TFT_BL,  brightness);

        canDisplayTurnOff = true;
      }
    }

    // check if on demand wifi is set
    if(setOndemardWifi){
      // stop the control task
      vTaskDelete(controlTaskHandle);
      setOndemardWifi = false;
      canDisplayTurnOff = false;

      // set the arc value to 100
      lv_arc_set_value(ui_wifiTimerArc, 100);
      lv_obj_set_style_arc_color(ui_wifiTimerArc, lv_color_hex(0xFFFFFF) , LV_PART_INDICATOR | LV_STATE_DEFAULT);

      lv_label_set_text(ui_Label4,"Connect to \n\nCryptoWatch v1.0 WiFi ");
      lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

      // unhide the back button
      lv_obj_remove_flag(ui_backBtn2, LV_OBJ_FLAG_HIDDEN);

      // change the screen to wifi screen
      _ui_screen_change(&ui_WifiScn, LV_SCR_LOAD_ANIM_OVER_LEFT, 500, 0, &ui_WifiScn_screen_init);

      // create the on demand wifi task
      xTaskCreatePinnedToCore(onDemandWiFiTask, "onDemandWiFiTask", 10096, NULL, 5, &handleOnDemand, 1);

    }

    // abord the on demand wifi task
    if(cancelOnDemandWifi){
      // stop the on demand wifi task, if running
      if(handleOnDemand != NULL){
        Serial.println("on demand wifi task deleted");
        vTaskDelete(handleOnDemand);
      }
      cancelOnDemandWifi = false;
      canDisplayTurnOff = true;

      // change the screen to home screen
      _ui_screen_change(&ui_MenuScn, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0, &ui_MenuScn_screen_init);

      
      // create the control task with handle exceptions
      xTaskCreatePinnedToCore(ControlTask, "ControlTask", 18096, NULL, 5, &controlTaskHandle, 1);
    }

    xSemaphoreGive(xMutex);
    delay(5);

  }
}

void onDemandWiFiTask(void *pvParameters){
  WiFiManager wm;
  commandArray commands;

  Serial.println("On Demand WiFi started");
  bool res = wm.startConfigPortal(" CryptoWatch v1.0 ");

  if(res){
    Serial.println("WiFi connected");
    // save the wifi credentials
    ssid = wm.getWiFiSSID();
    password = wm.getWiFiPass();

    // turn off the WiFi
    WiFi.mode(WIFI_OFF);

    // stop the portal
    wm.stopConfigPortal();

    // hide the back button
    guiCommand command;
    command.command = 3;
    command.wifiScreen.command = 2;
    command.wifiScreen.hideCancelBtn = true;
    commands.commands.push_back(command);

    // set the arc value to 100
    command.command = 3;
    command.wifiScreen.command = 1;
    command.wifiScreen.arcValue = 100;
    command.wifiScreen.arcColor = lv_color_hex(0x00FFF5);
    commands.commands.push_back(command);

    // set the text to Connected
    command.command = 3;
    command.wifiScreen.command = 0;
    command.wifiScreen.text = "New WiFi Added";
    command.wifiScreen.textColor = lv_color_hex(0x00FFF5);
    commands.commands.push_back(command);

    // create the GUI control task
    xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);

    delay(1000);

    // change the screen to menu screen
    command.command = 0;
    command.screen = 2;
    command.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
    commands.commands.push_back(command);

    // // set canDisplayTurnOff to true
    // command.command = 4;
    // commands.commands.push_back(command);

    // create the GUI control task
    xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);


  }else{
    // WiFi turn off
    WiFi.mode(WIFI_OFF);

    Serial.println("WiFi not connected");

    // stop the portal
    wm.stopConfigPortal();

    // hide the back button
    guiCommand command;
    command.command = 3;
    command.wifiScreen.command = 2;
    command.wifiScreen.hideCancelBtn = true;
    commands.commands.push_back(command);

    // set the arc value to 100
    command.command = 3;
    command.wifiScreen.command = 1;
    command.wifiScreen.arcValue = 100;
    command.wifiScreen.arcColor = lv_color_hex(0xFF0000);
    commands.commands.push_back(command);

    // set the text to Connected
    command.command = 3;
    command.wifiScreen.command = 0;
    command.wifiScreen.text = "WiFi Not Connected\n\nPlease try again";
    command.wifiScreen.textColor = lv_color_hex(0xFF0000);
    commands.commands.push_back(command);

    // create the GUI control task
    xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);

    delay(1000);

    // change the screen to menu screen
    command.command = 0;
    command.screen = 2;
    command.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
    commands.commands.push_back(command);

    //  // set canDisplayTurnOff to true
    //  command.command = 4;
    //  commands.commands.push_back(command);

    // create the GUI control task
    xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);
  }

  // restart the control task
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 18096, NULL, 5, &controlTaskHandle, 1);

  // delete the on demand wifi task
  vTaskDelete(NULL);
}

void GUIControlTask(void *pvParameters){
  // get the array of GUICommands from parameters
  commandArray* commands = (commandArray*)pvParameters;

  // take ui mutex
  xSemaphoreTake(xMutex, portMAX_DELAY);

  // run the GUI commands one by one
  while(commands->commands.size() > 0){
    guiCommand command = commands->commands[0];
    commands->commands.erase(commands->commands.begin());

     // update the time, ampmLbl and dateLbl on the screen
     switch(command.command){
      case 0:
      {
        switch(command.screen){
          case 1:
            _ui_screen_change(&ui_HomeScn, command.moveDir, 500, 0, &ui_HomeScn_screen_init);
            break;
          case 2:
            _ui_screen_change(&ui_MenuScn, command.moveDir, 500, 0, &ui_MenuScn_screen_init);
            break;
          case 3:
            _ui_screen_change(&ui_WifiScn, command.moveDir, 500, 0, &ui_WifiScn_screen_init);
            break;
        }
        break;
      }
      case 1:
      {
        switch(command.homeScreen.command){
          case 0:
            char time[10];
            sprintf(time, "%02d:%02d", command.homeScreen.hour, command.homeScreen.minute);
            lv_label_set_text(ui_timeLbl, time);
            if(command.homeScreen.ampm){
              lv_label_set_text(ui_ampmLbl, "PM");
            }else{
              lv_label_set_text(ui_ampmLbl, "AM");
            }
      
            // date - month-date Weekday
            char dateStr[15];
            sprintf(dateStr, "%02d-%02d %s", command.homeScreen.month, command.homeScreen.day, (command.homeScreen.weekday == 0) ? "SUN" : (command.homeScreen.weekday == 1) ? "MON" : (command.homeScreen.weekday == 2) ? "TUE" : (command.homeScreen.weekday == 3) ? "WED" : (command.homeScreen.weekday == 4) ? "THU" : (command.homeScreen.weekday == 5) ? "FRI" : "SAT");
            lv_label_set_text(ui_dateLbl, dateStr);

            // set the step count
            char stepCountStr[10];
            sprintf(stepCountStr, "%d", command.homeScreen.stepCount);
            lv_label_set_text(ui_stepCountLbl, stepCountStr);

            // // set the temperature
            // char tempStr[10];
            // sprintf(tempStr, "%d°", command.homeScreen.temp);
            // lv_label_set_text(ui_tempLbl, tempStr);

            // set the battery level
            lv_label_set_text(ui_Label6, command.homeScreen.batteryLvl.c_str());
            break;
          case 1:
            // set the crypto rate
            lv_label_set_text(ui_cryptoRateLbl, command.homeScreen.cryptoRate.c_str());

            // set the percent change 24h
            if(command.homeScreen.is_percent_change_24h_negative){
              lv_obj_set_style_text_color(ui_cryptoPercentageLbl, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
              lv_obj_set_style_text_color(ui_cryptoRateLbl, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
              lv_image_set_src(ui_arrowImg, &ui_img_down_triangle_png);
              lv_obj_set_y(ui_cryptoPercentageLbl, -10);
              lv_obj_set_y(ui_arrowImg, 6);
            }else{
              lv_obj_set_style_text_color(ui_cryptoPercentageLbl, lv_color_hex(0x00FF55), LV_PART_MAIN | LV_STATE_DEFAULT);
              lv_obj_set_style_text_color(ui_cryptoRateLbl, lv_color_hex(0x00FF55), LV_PART_MAIN | LV_STATE_DEFAULT);
              lv_image_set_src(ui_arrowImg, &ui_img_up_triangle_png);
              lv_obj_set_y(ui_cryptoPercentageLbl, 6);
              lv_obj_set_y(ui_arrowImg, -10);
            }
            lv_label_set_text(ui_cryptoPercentageLbl, command.homeScreen.percent_change_24h.c_str());
            break;
        }
        break;
      }
      case 2:
      {
        canDisplayTurnOff = true;
        break;
      }
      case 3:
      {
        switch(command.wifiScreen.command){
          case 0:
            lv_label_set_text(ui_Label4, command.wifiScreen.text);
            lv_obj_set_style_text_color(ui_Label4, command.wifiScreen.textColor, LV_PART_MAIN | LV_STATE_DEFAULT);
            break;
          case 1:
            lv_arc_set_value(ui_wifiTimerArc, command.wifiScreen.arcValue);
            lv_obj_set_style_arc_color(ui_wifiTimerArc, command.wifiScreen.arcColor , LV_PART_INDICATOR | LV_STATE_DEFAULT);
            break;
          case 2:
            if(command.wifiScreen.hideCancelBtn){
              lv_obj_add_flag(ui_backBtn2, LV_OBJ_FLAG_HIDDEN);
            }else{
              lv_obj_remove_flag(ui_backBtn2, LV_OBJ_FLAG_HIDDEN);
            }
            break;
        }
        break; 
      }
      
    }
  
  } 

  // give ui mutex
  xSemaphoreGive(xMutex);
  vTaskDelete(NULL);
}

void ControlTask(void *pvParameters){
  Serial.println("Control Task started");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  WiFiManager wm;
  struct tm timeinfo;
  commandArray commands;
  guiCommand command;
  float previousCryptoRate = 0;
  unsigned long  lastTimeCrypto = 0;
  unsigned long  intervalCrypto = 1000;
  unsigned long timeupdateInterval = 1000;
  unsigned long lastTimeUpdate = 0;
  unsigned long currentTime = 0;

  delay(1000);

  // check if the wifi is connected
  if (!isInitialSetup){
    // hide the back button
    command.command = 3;
    command.wifiScreen.command = 2;
    command.wifiScreen.hideCancelBtn = true;
    commands.commands.push_back(command);

    // set the arc value to 100
    command.command = 3;
    command.wifiScreen.command = 1;
    command.wifiScreen.arcValue = 100;
    command.wifiScreen.arcColor = lv_color_hex(0xFFFFFF);
    commands.commands.push_back(command);

    // set the text to connecting
    command.command = 3;
    command.wifiScreen.command = 0;
    command.wifiScreen.text = "Connect to \n\nCryptoWatch v1.0 WiFi ";
    command.wifiScreen.textColor = lv_color_hex(0xFFFFFF);
    commands.commands.push_back(command);

    // set the screen to wifi screen
    command.command = 0;
    command.screen = 3;
    command.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
    commands.commands.push_back(command);

    // create the GUI control task
    xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);

    // wm.resetSettings();

    // connect to wifi
    bool res = wm.autoConnect("CryptoWatch v1.0");

    if(!res){
      Serial.println("Failed to connect to WiFi and hit timeout");
      wm.resetSettings();
      ESP.restart();
    }else{

      // Assign the SSID and password to the global variables
      ssid = wm.getWiFiSSID();
      password = wm.getWiFiPass();
      Serial.printf("Connected to %s password %s\n", ssid, password);
    

      // set the arc color to green
      command.command = 3;
      command.wifiScreen.command = 1;
      command.wifiScreen.arcValue = 100;
      command.wifiScreen.arcColor = lv_color_hex(0x00FF55);
      commands.commands.push_back(command);

      // set the text to connected
      command.command = 3;
      command.wifiScreen.command = 0;
      command.wifiScreen.text = "Configuring the\n\nCryptoWatch v1.0";
      command.wifiScreen.textColor = lv_color_hex(0xFFFFFF);
      commands.commands.push_back(command);

      // create the GUI control task
      xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);

      while(1){
        if(!getLocalTime(&timeinfo)){
          Serial.println("Failed to obtain time");
          // continue;
        }else{
          // change text to configured
          command.command = 3;
          command.wifiScreen.command = 0;
          command.wifiScreen.text = "Configured\n\nCryptoWatch v1.0";
          command.wifiScreen.textColor = lv_color_hex(0x00FF55);
          commands.commands.push_back(command);

          // create the GUI control task
          xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);
          delay(1000);

          Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
          // get epoch time
          time_t epochTime = mktime(&timeinfo);
          Serial.print("Epoch Time: ");
          Serial.println(epochTime);

          // Convert the UNIX timestamp to a tm structure
          struct tm *tm = localtime(&epochTime);

          // Set the RTC time using the settimeofday function
          struct timeval tv;
          tv.tv_sec = epochTime;
          tv.tv_usec = 0;
          settimeofday(&tv, 0);

          // change the screen to home screen
          command.command = 0;
          command.screen = 1;
          command.moveDir = LV_SCR_LOAD_ANIM_MOVE_LEFT;
          commands.commands.push_back(command);

          // create the GUI control task
          xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);

          // turn off the WiFi
          WiFi.disconnect(true);
          WiFi.mode(WIFI_OFF);

          wm.stopConfigPortal();

          isInitialSetup = true;
          break;
        }
        delay(1000);
        Serial.println("Initial Configuration is running");
    }
    }

  Serial.println("Comes to until Loop");
  }

  while(1){
    // get current time
    currentTime = millis();
    // update the time every minute
    if(currentTime - lastTimeUpdate > timeupdateInterval){
      lastTimeUpdate = currentTime;
      // change the time update interval to 1 minute
      timeupdateInterval = 60000;
      Serial.println("Updating Time");
      
      time_t now;
      time(&now);
      struct tm *tm = localtime(&now);
      
      // get hour, minute and date, month and weekday, am/pm seperately
      command.command = 1;
      command.homeScreen.command = 0;
      command.homeScreen.hour = tm->tm_hour;
      command.homeScreen.minute = tm->tm_min;
      command.homeScreen.day = tm->tm_mday;
      command.homeScreen.month = tm->tm_mon + 1;
      command.homeScreen.weekday = tm->tm_wday;
      command.homeScreen.ampm = tm->tm_hour > 12 ? 0 : 1;
      command.homeScreen.hour = command.homeScreen.hour % 12;
      if(command.homeScreen.hour == 0){
        command.homeScreen.hour = 12;
      }
      command.homeScreen.stepCount = 100;
      
      // get battery level
      int Volts = analogReadMilliVolts(BAT_VOLTAGE_PIN);
      // battery level = (Volts * 3.0 / 1000.0) / Measurement_offset;
      // map the battery level to 0-100, which ranges from 3.0V to 4.2V
      int batteryLevel = map(Volts, 3000, 4200, 0, 100);
      command.homeScreen.batteryLvl = String(batteryLevel); 
      
      commands.commands.push_back(command);
      // create the GUI control task
      xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);
      
      // // update the time every minute
      // lastTimeUpdate = millis();
    }
    
    // update the crypto rate every 10 seconds
    if(currentTime - lastTimeCrypto > intervalCrypto){
      lastTimeCrypto = currentTime;
      // change crypto rate every 10 seconds
      intervalCrypto = 60000;
      Serial.println("Updating Crypto Rate");
      
      // turn on the WiFi
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      
      // wait for WiFi to connect
      while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.println("Connecting to WiFi...");
      }

      // read the crypto rate
      WiFiClientSecure *client = new WiFiClientSecure;

      if(client){
        // set secure client with certificate
        client->setCACert(test_root_ca);
      }
      // read the crypto rate from API
      if(client) {
        //create an HTTPClient instance
        HTTPClient https;
        https.addHeader("X-CMC_PRO_API_KEY", apiKey);
        //Initializing an HTTPS communication using the secure client
        if (https.begin(*client, server)) {  // HTTPS
          // start connection and send HTTP header
          int httpCode = https.GET();
          // httpCode will be negative on error
          if (httpCode > 0) {
#ifdef ENABLE_LOGGING
          // HTTP header has been send and Server response header has been handled
          Serial.printf("[HTTPS] GET... code: %d\n", httpCode);
#endif
          // file found at server
            if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
              // print server response payload
              String payload = https.getString();
              /*
              API Response ->
                {"status":{"timestamp":"2025-03-22T12:52:02.847Z",
                "error_code":0,"error_message":null,"elapsed":42,
                "credit_count":1,"notice":null},"data":{"2634":{"id":2634,
                "name":"XDC Network","symbol":"XDC","slug":"xdc-network",
                "num_market_pairs":132,"date_added":"2018-04-12T00:00:00.000Z",
                "tags":[{"slug":"enterprise-solutions","name":"Enterprise Solutions",
                "category":"INDUSTRY"},{"slug":"masternodes","name":"Masternodes",
                "category":"CATEGORY"},{"slug":"smart-contracts","name":"Smart Contracts",
                "category":"CATEGORY"},{"slug":"xdc-ecosystem","name":"XDC Ecosystem",
                "category":"PLATFORM"},{"slug":"real-world-assets","name":"Real World Assets",
                "category":"CATEGORY"},{"slug":"layer-1","name":"Layer 1","category":"CATEGORY"}],
                "max_supply":null,"circulating_supply":15696558123.6,"total_supply":37980093159.6,
                "is_active":1,"infinite_supply":false,"platform":null,"cmc_rank":61,"is_fiat":0,
                "self_reported_circulating_supply":15696549621,"self_reported_market_cap":1135324043.7378092,
                "tvl_ratio":null,"last_updated":"2025-03-22T12:50:00.000Z","quote":{"USD":{"price":0.07232952917365286,
                "volume_24h":30813718.79009731,"volume_change_24h":-21.7263,
                "percent_change_1h":0.22585061,"percent_change_24h":-2.94016178,
                "percent_change_7d":5.90558087,"percent_change_30d":-18.89977369,
                "percent_change_60d":-38.29822556,"percent_change_90d":-4.66137102,
                "market_cap":1135324658.726864,"market_cap_dominance":0.0411,
                "fully_diluted_market_cap":2747082256.21,"tvl":null,
                "last_updated":"2025-03-22T12:50:00.000Z"}}}}}
              */

              // parse the JSON response
              DynamicJsonDocument doc(2048);
              DeserializationError error = deserializeJson(doc, payload);

              if (error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.c_str());
                command.homeScreen.cryptoRate = "--:--";
              } else {
                // get the price of the crypto
                float price = doc["data"]["2634"]["quote"]["USD"]["price"];
                previousCryptoRate = price;
                float percent_change_24h = doc["data"]["2634"]["quote"]["USD"]["percent_change_24h"];
#ifdef ENABLE_LOGGING
                Serial.printf("Crypto Rate: %f\n", price);
#endif
                // price = price * 1000;
                // if crypto rate is in 0-9 range, set decimal points to 6
                if(price >= 0 && price < 10){
                  command.homeScreen.cryptoRate = String(price, 6);
                }else if(price >= 10 && price < 100){
                  command.homeScreen.cryptoRate = String(price, 5);
                }else if(price >= 100 && price < 1000){
                  command.homeScreen.cryptoRate = String(price, 4);
                }else if(price >= 1000 && price < 10000){
                  command.homeScreen.cryptoRate = String(price, 3);
                }else if(price >= 10000 && price < 100000){
                  command.homeScreen.cryptoRate = String(price, 2);
                }else if(price >= 100000 && price < 1000000){
                  command.homeScreen.cryptoRate = String(price, 1);
                }else if(price >= 1000000){
                  command.homeScreen.cryptoRate = String(price, 0);
                }

                // add '$' to the crypto rate
                command.homeScreen.cryptoRate = command.homeScreen.cryptoRate + "$" ;

                // percent_change_24h = "5.5%" remove the negativiy mark of the percent_change_24h

                if(percent_change_24h >= 0){
                  command.homeScreen.is_percent_change_24h_negative = false;
                  command.homeScreen.percent_change_24h = String(percent_change_24h, 1) + "%";
                }else{
                  command.homeScreen.percent_change_24h = String(abs(percent_change_24h), 1) + "%";
                  command.homeScreen.is_percent_change_24h_negative = true;
                }
            
              }
            }
          }
          else {
            Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
            command.homeScreen.cryptoRate = previousCryptoRate;
          }
          https.end();
        }
      }
      else {
        Serial.printf("[HTTPS] Unable to connect\n");
        command.homeScreen.cryptoRate = "--:--";
      }

      command.command = 1;
      command.homeScreen.command = 1;
      
      commands.commands.push_back(command);

      if(firstTImeStartDisplayTimeout){
        firstTImeStartDisplayTimeout = false;
        // start the display timeout timer
        command.command = 2;
        commands.commands.push_back(command);
      }

      // create the GUI control task
      xTaskCreatePinnedToCore(GUIControlTask, "GUIControlTask", 4096, &commands, 5, NULL, 0);

      // turn off the WiFi
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);

    }
    
    delay(1000);
  }

}