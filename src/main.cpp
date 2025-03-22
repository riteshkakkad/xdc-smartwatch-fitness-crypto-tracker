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

#define LVGL_TICK_PERIOD_MS 5

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
uint32_t lastTimeCrypto = 0;
uint32_t intervalCrypto = 10000;

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

#define SCREEN_BUFFER_SIZE (240 * 240)  
enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight /4 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);
CST816S touch(6, 7,13,5);	// sda, scl, rst, irq

// parameters for NTP Time
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 1;
int timeupdateInterval = 1000;
uint8_t lastTimeUpdate = 0;

// parameters for on demand wifi
unsigned int  timeout   = 120; // seconds to run for
unsigned int  startTime = millis();
bool portalRunning      = false;
bool startAP            = true;
int arcUpdateInterval   = 1200;
int lastArcUpdate       = 0; 
uint8_t arcValue       = 0;
bool deviceConnected   = false;

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

struct homeScreenCommand{
  uint8_t command; // 0 : time/date/battery/steps/temp, 1:cryptoRate
  uint8_t hour;
  uint8_t minute;
  uint8_t day;
  uint8_t month;
  uint8_t weekday;
  uint8_t ampm;
  int stepCount;
  int temp;
  int batteryLvl;
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

struct guiCommandQueue {
  uint8_t command; // 0 - screen change, 1 - home screen, 2 - menu screen, 3 - wifi screen , 4 - onDemand
  homeScreenCommand homeScreen;
  menuScreenCommand menuScreen;
  wifiScreenCommand wifiScreen;
  lv_screen_load_anim_t moveDir;
  uint8_t screen;
  uint8_t setOndemardWifi;
};

struct commandFromGUI{
  uint8_t command; // 0 - set wifi on demand, 1 - cancel wifi on demand
  uint8_t setOndemardWifi;
  uint8_t cancelOnDemandWifi;
};

QueueHandle_t commandQueue;
QueueHandle_t onDemandQueue;

// task handlers
TaskHandle_t handleOnDemand;

void* allocate_psram(size_t size);
void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap);
static uint32_t my_tick_get_cb (void);
void GUITask(void *pvParameters);
void GUIUpdateTask(void *pvParameters);
void OndemandWiFiTask(void *pvParameters);
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data);

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */
  Serial.println("Hello Arduino!");

  Serial.print("Heap size: ");
  Serial.println(ESP.getHeapSize());
  Serial.print("Heap free size: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("PSRAM size: ");
  Serial.println(ESP.getPsramSize());
  Serial.print("PSRAM free size: ");
  Serial.println(ESP.getFreePsram());

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);

  // initialize the command queue
  commandQueue = xQueueCreate(50, sizeof(guiCommandQueue));
  onDemandQueue = xQueueCreate(10, sizeof(commandFromGUI));

  // create GUI task runs in code 0 
  xTaskCreatePinnedToCore(GUITask, "GUITask", 18096, NULL, 3, NULL, 0);
  // delay(500);
  // xTaskCreatePinnedToCore(GUIUpdateTask, "GUIUpdateTask", 8096, NULL, 3, NULL, 1);
}

void loop()
{
  vTaskDelete(NULL);
}

/*Set tick routine needed for LVGL internal timings*/
static uint32_t my_tick_get_cb (void) { return millis(); }

/* Display flushing */
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap)
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  tft.startWrite();
  tft.setAddrWindow( area->x1, area->y1, w, h );
  tft.pushColors( ( uint16_t * )pixelmap, w * h, true );
  tft.endWrite();


  lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data)
{
  uint16_t touchX = 0, touchY = 0;

  bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

  // touch points need to rotate 90 degrees

  touchX = touch.data.x;
  touchY = touch.data.y;

  touched = touch.available();

  if (!touched)
  {
      data->state = LV_INDEV_STATE_REL;
  }
  else
  {
      data->state = LV_INDEV_STATE_PR;

      /*Set the coordinates*/
    data->point.x = 230 - touchY;
    data->point.y = touchX ;

    // Serial.print( "Data x " );
    // Serial.println(touchX);

    // Serial.print( "Data y " );
    // Serial.println( touchY );
  }
}

void* allocate_psram(size_t size) {
  void* ptr = ps_malloc(size);
  return ptr;
}

void GUITask(void *pvParameters){

  // set backlight LED pin as output
  pinMode(TFT_BL, OUTPUT);

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

  static lv_indev_t* indev;
  indev = lv_indev_create();
  lv_indev_set_type( indev, LV_INDEV_TYPE_POINTER );
  lv_indev_set_read_cb( indev, my_touchpad_read );

  //Dim the TFT backlight
  analogWrite(TFT_BL, 255/2); // values go from 0 to 255,

  ui_init();

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

  guiCommandQueue qdata;

  xTaskCreatePinnedToCore(GUIUpdateTask, "GUIUpdateTask", 28096, NULL, 3, NULL, 1);

  printf("Setup done\n");

//   const esp_timer_create_args_t lvgl_tick_timer_args = {
//     .callback = [](void *arg)
//     {lv_tick_inc(LVGL_TICK_PERIOD_MS);
//     lv_timer_handler(); },
//     .name = "lvgl_tick"};
// esp_timer_handle_t lvgl_tick_timer = nullptr;
// ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
// ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

  while(1){
    if(xQueueReceive(commandQueue, &qdata, 0) == pdTRUE){
      // update the time, ampmLbl and dateLbl on the screen
      switch(qdata.command){
        case 0:
          switch(qdata.screen){
            case 1:
              _ui_screen_change(&ui_HomeScn, qdata.moveDir, 500, 0, &ui_HomeScn_screen_init);
              break;
            case 2:
              _ui_screen_change(&ui_MenuScn, qdata.moveDir, 500, 0, &ui_MenuScn_screen_init);
              break;
            case 3:
              _ui_screen_change(&ui_WifiScn, qdata.moveDir, 500, 0, &ui_WifiScn_screen_init);
              break;
          }
          break;
        case 1:
          switch(qdata.homeScreen.command){
            case 0:
              char time[10];
              sprintf(time, "%02d:%02d", qdata.homeScreen.hour, qdata.homeScreen.minute);
              lv_label_set_text(ui_timeLbl, time);
              if(qdata.homeScreen.ampm){
                lv_label_set_text(ui_ampmLbl, "PM");
              }else{
                lv_label_set_text(ui_ampmLbl, "AM");
              }
        
              // date - month-date Weekday
              char dateStr[15];
              sprintf(dateStr, "%02d-%02d %s", qdata.homeScreen.month, qdata.homeScreen.day, (qdata.homeScreen.weekday == 0) ? "SUN" : (qdata.homeScreen.weekday == 1) ? "MON" : (qdata.homeScreen.weekday == 2) ? "TUE" : (qdata.homeScreen.weekday == 3) ? "WED" : (qdata.homeScreen.weekday == 4) ? "THU" : (qdata.homeScreen.weekday == 5) ? "FRI" : "SAT");
              lv_label_set_text(ui_dateLbl, dateStr);

              // set the step count
              char stepCountStr[10];
              sprintf(stepCountStr, "%d", qdata.homeScreen.stepCount);
              lv_label_set_text(ui_stepCountLbl, stepCountStr);

              // set the temperature
              char tempStr[10];
              sprintf(tempStr, "%d°", qdata.homeScreen.temp);
              lv_label_set_text(ui_tempLbl, tempStr);

              // set the battery level
              lv_bar_set_value(ui_Bar1, qdata.homeScreen.batteryLvl, LV_ANIM_OFF);
              break;
            case 1:
              // set the crypto rate
              lv_label_set_text(ui_cryptoRateLbl, qdata.homeScreen.cryptoRate.c_str());

              // set the percent change 24h
              if(qdata.homeScreen.is_percent_change_24h_negative){
                lv_obj_set_style_text_color(ui_cryptoPercentageLbl, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_image_set_src(ui_arrowImg, &ui_img_down_triangle_png);
              }else{
                lv_obj_set_style_text_color(ui_cryptoPercentageLbl, lv_color_hex(0x00FF55), LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_image_set_src(ui_arrowImg, &ui_img_up_triangle_png);
              }
              lv_label_set_text(ui_cryptoPercentageLbl, qdata.homeScreen.percent_change_24h.c_str());
              break;
          }
          break;

        case 3:
          switch(qdata.wifiScreen.command){
            case 0:
              lv_label_set_text(ui_Label4, qdata.wifiScreen.text);
              lv_obj_set_style_text_color(ui_Label4, qdata.wifiScreen.textColor, LV_PART_MAIN | LV_STATE_DEFAULT);
              break;
            case 1:
              lv_arc_set_value(ui_wifiTimerArc, qdata.wifiScreen.arcValue);
              lv_obj_set_style_arc_color(ui_wifiTimerArc, qdata.wifiScreen.arcColor , LV_PART_INDICATOR | LV_STATE_DEFAULT);
              break;
            case 2:
              if(qdata.wifiScreen.hideCancelBtn){
                lv_obj_add_flag(ui_backBtn2, LV_OBJ_FLAG_HIDDEN);
              }else{
                lv_obj_remove_flag(ui_backBtn2, LV_OBJ_FLAG_HIDDEN);
              }
              break;
          }
          break; 
        case 4:
          setOndemardWifi = qdata.setOndemardWifi;
          break;     
          
      }
    }

    if(setOndemardWifi){
      setOndemardWifi = false;
      // send onDemandQueue 
      commandFromGUI data;
      data.command = 0;
      data.setOndemardWifi = 1;
      xQueueSend(onDemandQueue, &data, 0);
    }

    if(cancelOnDemandWifi){
      cancelOnDemandWifi = false;
      // send onDemandQueue 
      commandFromGUI data;
      data.command = 1;
      data.cancelOnDemandWifi = 1;
      xQueueSend(onDemandQueue, &data, 0);
    }
    lv_timer_handler(); /* let the GUI do its work */
    delay(5);
  }
}

void GUIUpdateTask(void *pvParameters){
  guiCommandQueue qdata;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  WiFiManager wm;
  struct tm timeinfo;
  commandFromGUI onDemandCommand;
  WiFiClientSecure *client;
  //create an HTTPClient instance
  HTTPClient https;

  // check if the wifi is connected
  if (WiFi.status() != WL_CONNECTED) {
    
    delay(500);
    // hide the back button 
    qdata.command = 3;
    qdata.wifiScreen.command = 2;
    qdata.wifiScreen.hideCancelBtn = true;
    xQueueSend(commandQueue, &qdata, 0);

    // set the arc value to 100
    qdata.command = 3;
    qdata.wifiScreen.command = 1;
    qdata.wifiScreen.arcValue = 100;
    qdata.wifiScreen.arcColor = lv_color_hex(0xFFFFFF);
    xQueueSend(commandQueue, &qdata, 0);

    // change text of ui_Label4 as connected
    qdata.command = 3;
    qdata.wifiScreen.command = 0;
    qdata.wifiScreen.text = "Connect to \n\nCryptoWatch v1.0 WiFi ";
    qdata.wifiScreen.textColor = lv_color_hex(0xFFFFFF);
    xQueueSend(commandQueue, &qdata, 0);

    // change to screen 3
    qdata.command = 0;
    qdata.screen = 3;
    qdata.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
    xQueueSend(commandQueue, &qdata, 0);
    
    // wm.resetSettings(); 

    bool res = wm.autoConnect(" CryptoWatch v1.0 ");

    if(res) {
      // delete wm 
      wm.stopConfigPortal();
      wm.stopWebPortal();
      Serial.println("connected...yeey :)");

      // set arc color to green
      qdata.command = 3;
      qdata.wifiScreen.command = 1;
      qdata.wifiScreen.arcColor = lv_color_hex(0x00FF55);
      xQueueSend(commandQueue, &qdata, 0);
      // change text of ui_Label4 as connected
      // qdata.command = 3;
      // qdata.wifiScreen.command = 0;
      // sprintf(qdata.wifiScreen.text, "Connected to WiFi\n%s", WiFi.SSID().c_str());
      // qdata.wifiScreen.textColor = lv_color_hex(0x00FF55);
      // xQueueSend(commandQueue, &qdata, 0);
      // delay(1000);

      // qdata.command = 3;
      // qdata.wifiScreen.command = 0;
      // sprintf(qdata.wifiScreen.text, "Connected to WiFi\n%s", WiFi.SSID().c_str());
      // qdata.wifiScreen.textColor = lv_color_hex(0xFFFFFF);
      // xQueueSend(commandQueue, &qdata, 0);
      // delay(1000);


      qdata.command = 3;
      qdata.wifiScreen.command = 0;
      qdata.wifiScreen.text = "Configuring the\nCryptoWatch v1.0";
      qdata.wifiScreen.textColor = lv_color_hex(0xFFFFFF);
      xQueueSend(commandQueue, &qdata, 0);
      delay(1000);

      while(1){
        if(!getLocalTime(&timeinfo)){
          Serial.println("Failed to obtain time");
          // continue;
        }else{
          // change text of ui_Label4
          qdata.command = 3;
          qdata.wifiScreen.command = 0;
          qdata.wifiScreen.text = "Configured the\nCryptoWatch v1.0";
          qdata.wifiScreen.textColor = lv_color_hex(0x00FF55);
          xQueueSend(commandQueue, &qdata, 0);
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

          // set the http clients
          client = new WiFiClientSecure;

          if(client){
            // set secure client with certificate
            client->setCACert(test_root_ca);
          }

          lastTimeUpdate = millis();
          break;
        } 
        delay(1000);
      }

      // change the screen HomeScn
      qdata.command = 0;
      qdata.screen = 1;
      qdata.moveDir = LV_SCR_LOAD_ANIM_MOVE_LEFT;
      xQueueSend(commandQueue, &qdata, 0);

      // update the arc color
      qdata.command = 3;
      qdata.wifiScreen.command = 1;
      qdata.wifiScreen.arcColor = lv_color_hex(0xFFFFFF);
      xQueueSend(commandQueue, &qdata, 0);

      // change text of ui_Label4
      qdata.command = 3;
      qdata.wifiScreen.command = 0;
      sprintf(qdata.wifiScreen.text, "Connect to \n\nCryptoWatch v1.0 WiFi ");
      qdata.wifiScreen.textColor = lv_color_hex(0xFFFFFF);
      xQueueSend(commandQueue, &qdata, 0);

      // unhide the back button
      qdata.command = 3;
      qdata.wifiScreen.command = 2;
      qdata.wifiScreen.hideCancelBtn = false;
      xQueueSend(commandQueue, &qdata, 0);
      // delay(100);
      
    } else {
      Serial.println("not connected... :(");

      // TODO: handle this error.
      lv_label_set_text(ui_Label4, "Not Connected to WiFi\n Try again in 3");
      delay(500);
      lv_label_set_text(ui_Label4, "Not Connected to WiFi\n Try again in 2");
      delay(500);
      lv_label_set_text(ui_Label4, "Not Connected to WiFi\n Try again in 1");
      delay(500);

      // Restart the ESP
      ESP.restart();
    }
    
  }

  while(1){
    if(xQueueReceive(onDemandQueue, &onDemandCommand, 0) == pdTRUE){
      switch(onDemandCommand.command){
        case 0:
          // set arc value to 0
          qdata.command = 3;
          qdata.wifiScreen.command = 1;
          qdata.wifiScreen.arcValue = 100;
          xQueueSend(commandQueue, &qdata, 0);

          // change the screen to WifiScn
          qdata.command = 0;
          qdata.screen = 3;
          qdata.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
          xQueueSend(commandQueue, &qdata, 0);

          // create a task for on demand wifi
          xTaskCreatePinnedToCore(OndemandWiFiTask, "OndemandWiFiTask", 8096, NULL, 3, &handleOnDemand, 0);
          break;
        
        case 1:
          // delete the task
          vTaskDelete(handleOnDemand);

          // change screen to MenuScn
          qdata.command = 0;
          qdata.screen = 2;
          qdata.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
          xQueueSend(commandQueue, &qdata, 0);
          break;
      }
    }
    
    // update the time every 1 second
    if(millis() - lastTimeUpdate > timeupdateInterval){
      lastTimeUpdate = millis();
      
      time_t now;
      time(&now);
      struct tm *tm = localtime(&now);

      // get hour, minute and date, month and weekday, am/pm seperately
      qdata.homeScreen.hour = tm->tm_hour;
      qdata.homeScreen.minute = tm->tm_min;
      qdata.homeScreen.day = tm->tm_mday;
      qdata.homeScreen.month = tm->tm_mon + 1;
      qdata.homeScreen.weekday = tm->tm_wday;
      qdata.homeScreen.ampm = qdata.homeScreen.hour >= 12 ? 1 : 0;
      qdata.homeScreen.hour = qdata.homeScreen.hour % 12;
      if(qdata.homeScreen.hour == 0){
        qdata.homeScreen.hour = 12;
      }
      // read the step counter
      qdata.homeScreen.stepCount = 100;
      // read the temperature
      qdata.homeScreen.temp = 30;
      // read the battery level
      int Volts = analogReadMilliVolts(BAT_VOLTAGE_PIN);
      qdata.homeScreen.batteryLvl = (Volts * 3.0 / 1000.0) / Measurement_offset;
      Serial.printf("Battery Level: %d\n", qdata.homeScreen.batteryLvl);
      
      qdata.command = 1;
      qdata.homeScreen.command = 0;
      xQueueSend(commandQueue, &qdata, 0);
    }

    if(millis() - lastTimeCrypto > intervalCrypto){
      lastTimeCrypto = millis();
      // read the crypto rate
      // read the crypto rate from API
      if(client) {
        https.addHeader("X-CMC_PRO_API_KEY", apiKey);
        //Initializing an HTTPS communication using the secure client
        if (https.begin(*client, server)) {  // HTTPS
          // start connection and send HTTP header
          int httpCode = https.GET();
          // httpCode will be negative on error
          if (httpCode > 0) {
          // HTTP header has been send and Server response header has been handled
          Serial.printf("[HTTPS] GET... code: %d\n", httpCode);
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
              DynamicJsonDocument doc(1024);
              DeserializationError error = deserializeJson(doc, payload);

              if (error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.c_str());
                qdata.homeScreen.cryptoRate = "--:--";
              } else {
                // get the price of the crypto
                float price = doc["data"]["2634"]["quote"]["USD"]["price"];
                float percent_change_24h = doc["data"]["2634"]["quote"]["USD"]["percent_change_24h"];
                Serial.printf("Crypto Rate: %f\n", price);
                qdata.homeScreen.cryptoRate = String(price, 5);
                // percent_change_24h = "5.5%" remove the negativiy mark of the percent_change_24h

                if(percent_change_24h >= 0){
                  qdata.homeScreen.is_percent_change_24h_negative = false;
                  qdata.homeScreen.percent_change_24h = String(percent_change_24h, 2) + "%";
                }else{
                  qdata.homeScreen.percent_change_24h = String(abs(percent_change_24h), 2) + "%";
                  qdata.homeScreen.is_percent_change_24h_negative = true;
                }
            
              }
            }
          }
          else {
            Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
            qdata.homeScreen.cryptoRate = "--:--";
          }
          https.end();
        }
      }
      else {
        Serial.printf("[HTTPS] Unable to connect\n");
        qdata.homeScreen.cryptoRate = "--:--";
      }

      qdata.command = 1;
      qdata.homeScreen.command = 1;
      xQueueSend(commandQueue, &qdata, 0);
    }
    delay(1000);
  }
}

void OndemandWiFiTask(void *pvParameters){
  WiFiManager wm;
  guiCommandQueue qdata;

  Serial.println("On Demand WiFi started");
  wm.setConfigPortalBlocking(false);
  wm.startConfigPortal(" CryptoWatch v1.0 ");

  startTime = millis();
  // is auto timeout portal running
  while(1){
    wm.process(); // do processing

    // check for timeout
    if((millis()-startTime) > (timeout*1000)){
      Serial.println("portaltimeout");
      wm.stopConfigPortal();

      // move to MenuScn
      qdata.command = 0;
      qdata.screen = 2;
      qdata.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
      xQueueSend(commandQueue, &qdata, 0);

      // set arc value to 100
      qdata.command = 3;
      qdata.wifiScreen.command = 1;
      qdata.wifiScreen.arcValue = 100;
      xQueueSend(commandQueue, &qdata, 0);

      // delete the task
      vTaskDelete(NULL);
      
    }
  }
}
