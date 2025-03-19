// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 9.1.0
// Project name: SquareLine_Project

#include "ui.h"

void ui_MenuScn_screen_init(void)
{
    ui_MenuScn = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_MenuScn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_MenuScn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_MenuScn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Image3 = lv_image_create(ui_MenuScn);
    lv_image_set_src(ui_Image3, &ui_img_brightness_png);
    lv_obj_set_width(ui_Image3, LV_SIZE_CONTENT);   /// 32
    lv_obj_set_height(ui_Image3, LV_SIZE_CONTENT);    /// 32
    lv_obj_set_x(ui_Image3, -70);
    lv_obj_set_y(ui_Image3, -65);
    lv_obj_set_align(ui_Image3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Image3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_remove_flag(ui_Image3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_brightnessSlider = lv_slider_create(ui_MenuScn);
    lv_slider_set_value(ui_brightnessSlider, 50, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_brightnessSlider) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_brightnessSlider, 0,
                                                                                                     LV_ANIM_OFF);
    lv_obj_set_width(ui_brightnessSlider, 120);
    lv_obj_set_height(ui_brightnessSlider, 10);
    lv_obj_set_x(ui_brightnessSlider, 18);
    lv_obj_set_y(ui_brightnessSlider, -65);
    lv_obj_set_align(ui_brightnessSlider, LV_ALIGN_CENTER);
    lv_obj_set_style_radius(ui_brightnessSlider, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_brightnessSlider, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_brightnessSlider, 50, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_brightnessSlider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_brightnessSlider, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_brightnessSlider, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_brightnessSlider, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    //Compensating for LVGL9.1 draw crash with bar/slider max value when top-padding is nonzero and right-padding is 0
    if(lv_obj_get_style_pad_top(ui_brightnessSlider, LV_PART_MAIN) > 0) lv_obj_set_style_pad_right(ui_brightnessSlider,
                                                                                                       lv_obj_get_style_pad_right(ui_brightnessSlider, LV_PART_MAIN) + 1, LV_PART_MAIN);
    ui_Label1 = lv_label_create(ui_MenuScn);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -50);
    lv_obj_set_y(ui_Label1, -16);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "Disp. ON Time");
    lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Label1, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label1, &ui_font_inter16sb, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_addNewWiFibtn = lv_button_create(ui_MenuScn);
    lv_obj_set_width(ui_addNewWiFibtn, 150);
    lv_obj_set_height(ui_addNewWiFibtn, 40);
    lv_obj_set_x(ui_addNewWiFibtn, 0);
    lv_obj_set_y(ui_addNewWiFibtn, 34);
    lv_obj_set_align(ui_addNewWiFibtn, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_addNewWiFibtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_addNewWiFibtn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_addNewWiFibtn, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_addNewWiFibtn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_addNewWiFibtn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_addNewWiFibtn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_addNewWiFibtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_addNewWiFibtn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_addNewWiFibtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_addNewWiFibtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_addNewWiFibtn);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "Add New WiFi");
    lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui_Label2, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_Label2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label2, &ui_font_inter16sb, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Roller1 = lv_roller_create(ui_MenuScn);
    lv_roller_set_options(ui_Roller1, "10s\n30s\n1min\n5mins", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_height(ui_Roller1, 40);
    lv_obj_set_width(ui_Roller1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_Roller1, 66);
    lv_obj_set_y(ui_Roller1, -17);
    lv_obj_set_align(ui_Roller1, LV_ALIGN_CENTER);

    lv_obj_set_style_text_color(ui_Roller1, lv_color_hex(0x000000), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Roller1, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Roller1, lv_color_hex(0xFFFFFF), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Roller1, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_backBtn = lv_button_create(ui_MenuScn);
    lv_obj_set_width(ui_backBtn, 100);
    lv_obj_set_height(ui_backBtn, 40);
    lv_obj_set_x(ui_backBtn, 0);
    lv_obj_set_y(ui_backBtn, 81);
    lv_obj_set_align(ui_backBtn, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_backBtn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_remove_flag(ui_backBtn, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_backBtn, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_backBtn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_backBtn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_backBtn, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_backBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_backBtn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_backBtn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_backBtn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label3 = lv_label_create(ui_backBtn);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Back");
    lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label3, &ui_font_inter16sb, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_brightnessSlider, ui_event_brightnessSlider, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_addNewWiFibtn, ui_event_addNewWiFibtn, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_backBtn, ui_event_backBtn, LV_EVENT_ALL, NULL);

}
