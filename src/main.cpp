#include <Arduino.h>
#include <lvgl.h>
#include <CST816S.h>
#include <ui/ui.h>
#include <TFT_eSPI.h>

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

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

#define SCREEN_BUFFER_SIZE (240 * 240 * 2)  
enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);
// Arduino_DataBus *bus = new Arduino_HWSPI( TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO );
// Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST /* RST */);
CST816S touch(6, 7,13,5);	// sda, scl, rst, irq

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

// void drawColourBitMap(uint16_t red, uint16_t green, uint16_t blue, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
// {
//   uint16_t *bitmap = (uint16_t *)malloc(w * h * sizeof(uint16_t));
//   for (uint16_t i = 0; i < w * h; i++)
//   {
//     bitmap[i] = RGB565(red, green, blue);
//   }
//   // gfx->drawBitmap(x, y, bitmap, w, h, 0, 0);
//   free(bitmap);
// }

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
      data->point.x = 240 - touchY;
      data->point.y = touchX ;

      Serial.print( "Data x " );
      Serial.println(touchX);

      Serial.print( "Data y " );
      Serial.println( touchY );
    }
}

/*Set tick routine needed for LVGL internal timings*/
static uint32_t my_tick_get_cb (void) { return millis(); }

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */
  Serial.println("Hello Arduino!");

  // set backlight LED pin as output
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

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

  static lv_disp_t* disp;
  disp = lv_display_create( screenWidth, screenHeight );
  lv_display_set_buffers( disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb( disp, my_disp_flush );
  // lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);

  static lv_indev_t* indev;
  indev = lv_indev_create();
  lv_indev_set_type( indev, LV_INDEV_TYPE_POINTER );
  lv_indev_set_read_cb( indev, my_touchpad_read );

  // lv_obj_t *label = lv_label_create( lv_screen_active() );
  // lv_label_set_text( label, "Hello Anjana, I'm LVGL!" );
  // lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
  ui_init();

  Serial.println("Setup done");
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
  delay(5); /* let this time pass */
}