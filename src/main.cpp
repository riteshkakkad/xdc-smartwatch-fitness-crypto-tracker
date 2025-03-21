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

#define LVGL_TICK_PERIOD_MS 5

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

#define SCREEN_BUFFER_SIZE (240 * 240)  
enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight /4 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);
// Arduino_DataBus *bus = new Arduino_HWSPI( TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO );
// Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST /* RST */);
CST816S touch(6, 7,13,5);	// sda, scl, rst, irq
WiFiManager wm;
struct tm timeinfo;
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP);

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

uint8_t current_screent = 1;

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

struct homeScreenCommand{
  uint8_t command; // 0 - time/date
  uint8_t hour;
  uint8_t minute;
  uint8_t day;
  uint8_t month;
  uint8_t weekday;
  uint8_t ampm;
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

void* allocate_psram(size_t size);
void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap);
static uint32_t my_tick_get_cb (void);
void GUITask(void *pvParameters);
void GUIUpdateTask(void *pvParameters);
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data);


static void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value((lv_obj_t *)obj, v);
}

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
  analogWrite(TFT_BL, 255); // values go from 0 to 255,

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

  xTaskCreatePinnedToCore(GUIUpdateTask, "GUIUpdateTask", 8096, NULL, 3, NULL, 1);

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

    bool res = wm.autoConnect("CryptoWatch v1.0");

    if(res) {
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

      // change text of ui_Label4
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

  commandFromGUI onDemandCommand;

  while(1){
    if(xQueueReceive(onDemandQueue, &onDemandCommand, 0) == pdTRUE){
      switch(onDemandCommand.command){
        case 0:
          // set arc value to 0
          qdata.command = 3;
          qdata.wifiScreen.command = 1;
          qdata.wifiScreen.arcValue = 0;
          xQueueSend(commandQueue, &qdata, 0);

          // change the screen to WifiScn
          qdata.command = 0;
          qdata.screen = 3;
          qdata.moveDir = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
          xQueueSend(commandQueue, &qdata, 0);


          Serial.println("On Demand WiFi started");
          wm.setConfigPortalBlocking(false);
          wm.startConfigPortal("CryptoWatch v1.0");

          portalRunning = true;
          startTime = millis();
          lastArcUpdate = millis();
          arcValue = 0;
          break;
        
        case 1:
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

      qdata.command = 1;
      qdata.homeScreen.command = 0;
      xQueueSend(commandQueue, &qdata, 0);
    }
    
    // is auto timeout portal running
    if(portalRunning){
      wm.process(); // do processing

      // update the arc every 1 second
      if(millis() - lastArcUpdate > arcUpdateInterval){
        arcValue++;
        lastArcUpdate = millis();
        qdata.command = 3;
        qdata.wifiScreen.command = 1;
        qdata.wifiScreen.arcValue = arcValue;
        xQueueSend(commandQueue, &qdata, 0);
      }

      // check for timeout
      if((millis()-startTime) > (timeout*1000)){
        Serial.println("portaltimeout");
        portalRunning = false;
        if(startAP){
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
        } 
      }
    }
  }
}
