#include "main.h"
#include "display/lv_conf.h"
#include "config.hpp"
#include <string>


 static lv_res_t color_action(lv_obj_t * btn)
 {
  lv_obj_t * btn2 = (lv_obj_t *) lv_obj_get_free_ptr(btn);

  static lv_style_t redstyle;
  lv_style_copy(&redstyle, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
  redstyle.body.main_color = LV_COLOR_RED;
  redstyle.body.grad_color = LV_COLOR_RED;
  redstyle.body.border.color = LV_COLOR_GRAY;
  redstyle.body.border.width = 2;
  redstyle.text.color = LV_COLOR_WHITE;

 static lv_style_t bluestyle;
 lv_style_copy(&bluestyle, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
 bluestyle.body.main_color = LV_COLOR_BLUE;
 bluestyle.body.grad_color = LV_COLOR_BLUE;
 bluestyle.body.border.color = LV_COLOR_GRAY;
 bluestyle.body.border.width = 2;
 bluestyle.text.color = LV_COLOR_WHITE;

    lv_obj_t * label1 = lv_obj_get_child(btn, NULL); /*The label is the only child*/
    if(blueSide == true){
      lv_obj_set_style(btn, &redstyle);
      blueSide = false;
      lv_label_set_text(label1, "Red");
      lv_obj_set_style(btn2, &redstyle);

    } else{
      lv_obj_set_style(btn, &bluestyle);
      blueSide = true;
      lv_label_set_text(label1, "Blue");
      lv_obj_set_style(btn2, &bluestyle);

    }
    return LV_RES_OK;
 }

 static lv_res_t side_action(lv_obj_t * btn)
 {
   static lv_style_t redstyle;
   lv_style_copy(&redstyle, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
   redstyle.body.main_color = LV_COLOR_RED;
   redstyle.body.grad_color = LV_COLOR_RED;
   redstyle.body.border.color = LV_COLOR_GRAY;
   redstyle.body.border.width = 2;
   redstyle.text.color = LV_COLOR_WHITE;

   static lv_style_t bluestyle;
   lv_style_copy(&bluestyle, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
   bluestyle.body.main_color = LV_COLOR_BLUE;
   bluestyle.body.grad_color = LV_COLOR_BLUE;
   bluestyle.body.border.color = LV_COLOR_GRAY;
   bluestyle.body.border.width = 2;
   bluestyle.text.color = LV_COLOR_WHITE;

   lv_obj_t * label1 = lv_obj_get_child(btn, NULL); /*The label is the only child*/
       if(farSide == true){
         farSide = false;
         lv_label_set_text(label1, "Close");
       } else{
         farSide = true;
         lv_label_set_text(label1, "Far");
       }
       if(blueSide == true){
         lv_obj_set_style(btn, &bluestyle);
       }else{
         lv_obj_set_style(btn, &redstyle);
       }

   return LV_RES_OK;
 }

 static lv_res_t display_auton(lv_obj_t * btn)
 {
   lv_obj_t * mbox1 = lv_mbox_create(lv_scr_act(), NULL);
   lv_obj_set_size(mbox1, 450, 500);
   lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 10, 10); /*Align to the center*/

   std::string color;//initialize strings
   std::string side;
   std::string parkk;

   if (blueSide == true){
    color = "Blue";
  } else{
    color = "Red";
  }

  if(farSide == true){
   side = "Far";
  }else{
   side = "Close";
  }

  if(park == true){
    parkk = " WITH Park ";
  }else{
    parkk = " WITHOUT Park ";
  }

  std::string concat = color + " " + side + parkk + "selected"; //create string to display

  const char* c_data = concat.c_str( );//make string into char array cuz set_text only take char*
   lv_mbox_set_text(mbox1, c_data);
   lv_mbox_start_auto_close(mbox1, 2500);
   return LV_RES_OK;
 }

 static lv_res_t parkyes(lv_obj_t * btn)
 {
   static lv_style_t parkstyle;
   lv_style_copy(&parkstyle, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
   parkstyle.body.main_color = LV_COLOR_YELLOW;
   parkstyle.body.grad_color = LV_COLOR_YELLOW;
   parkstyle.body.border.color = LV_COLOR_GRAY;
   parkstyle.body.border.width = 2;
   parkstyle.text.color = LV_COLOR_BLACK;
   lv_obj_t * label1 = lv_obj_get_child(btn, NULL); /*The label is the only child*/
   if(park == true){
     park = false;
     lv_label_set_text(label1, "No Park");
   }else{
     park = true;
     lv_label_set_text(label1, "Park");
   }
   lv_obj_set_style(btn, &parkstyle);
   return LV_RES_OK;
 }


void initialize() {/*Create a three buttons, color, side, display auton */

     static lv_style_t initredstyle;
     lv_style_copy(&initredstyle, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
     initredstyle.body.main_color = LV_COLOR_RED;
     initredstyle.body.grad_color = LV_COLOR_RED;
     initredstyle.body.border.color = LV_COLOR_GRAY;
     initredstyle.body.border.width = 2;
     initredstyle.text.color = LV_COLOR_WHITE;

     static lv_style_t parkstyle;
     lv_style_copy(&parkstyle, &lv_style_plain);    /*Copy a built-in style to initialize the new style*/
     parkstyle.body.main_color = LV_COLOR_YELLOW;
     parkstyle.body.grad_color = LV_COLOR_YELLOW;
     parkstyle.body.border.color = LV_COLOR_GRAY;
     parkstyle.body.border.width = 2;
     parkstyle.text.color = LV_COLOR_BLACK;

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);         /*Create a button on the currently loaded screen*/
    lv_obj_set_style(btn1, &initredstyle);
    lv_obj_t * label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Red");
		lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, color_action); /*Set function to be called when the button is released*/
    lv_obj_align(btn1, label, LV_ALIGN_OUT_BOTTOM_LEFT, -30, 20);  /*Align below the label*/


    /*Copy the previous button*/
    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), btn1);        /*Second parameter is an object to copy*/
    lv_obj_set_style(btn2, &initredstyle);
    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label, "Close");
    lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, side_action);
    lv_obj_align(btn2, btn1, LV_ALIGN_OUT_RIGHT_MID, 35, 0);    /*Align next to the prev. button.*/

    lv_obj_set_free_ptr(btn1, btn2);

    lv_obj_t * btn3 = lv_btn_create(lv_scr_act(), NULL);
    //lv_obj_set_style(btn3, &lv_style_plain);
    lv_btn_set_action(btn3, LV_BTN_ACTION_CLICK, display_auton);
    lv_obj_align(btn3, btn2, LV_ALIGN_OUT_RIGHT_TOP, 35,-60);
    label = lv_label_create(btn3, NULL);
    lv_label_set_text(label, "Display \nAuton");

    lv_obj_t * btn4 = lv_btn_create(lv_scr_act(), NULL);
    label = lv_label_create(btn4, NULL);
    lv_label_set_text(label, "Park");
    lv_obj_set_style(btn4, &parkstyle);
    lv_btn_set_action(btn4, LV_BTN_ACTION_CLICK, parkyes);
    lv_obj_align(btn4, btn2, LV_ALIGN_OUT_RIGHT_TOP, 35,60);


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
