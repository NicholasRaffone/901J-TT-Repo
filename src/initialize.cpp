#include "main.h"
#include "display/lv_conf.h"
#include "config.hpp"
#include <string>
#include <cstdio>
/*
 static lv_res_t color_action(lv_obj_t * btn)
 {
  lv_obj_t * btn2 = (lv_obj_t *) lv_obj_get_free_ptr(btn);

  static lv_style_t redstyle;
  lv_style_copy(&redstyle, &lv_style_plain);    //Copy a built-in style to initialize the new style
  redstyle.body.main_color = LV_COLOR_RED;
  redstyle.body.grad_color = LV_COLOR_RED;
  redstyle.body.border.color = LV_COLOR_GRAY;
  redstyle.body.border.width = 2;
  redstyle.text.color = LV_COLOR_WHITE;

   static lv_style_t bluestyle;
   lv_style_copy(&bluestyle, &lv_style_plain);    //Copy a built-in style to initialize the new style
   bluestyle.body.main_color = LV_COLOR_BLUE;
   bluestyle.body.grad_color = LV_COLOR_BLUE;
   bluestyle.body.border.color = LV_COLOR_GRAY;
   bluestyle.body.border.width = 2;
   bluestyle.text.color = LV_COLOR_WHITE;

    lv_obj_t * label1 = lv_obj_get_child(btn, NULL); //The label is the only child
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
   lv_style_copy(&redstyle, &lv_style_plain);
   redstyle.body.main_color = LV_COLOR_RED;
   redstyle.body.grad_color = LV_COLOR_RED;
   redstyle.body.border.color = LV_COLOR_GRAY;
   redstyle.body.border.width = 2;
   redstyle.text.color = LV_COLOR_WHITE;

   static lv_style_t bluestyle;
   lv_style_copy(&bluestyle, &lv_style_plain);
   bluestyle.body.main_color = LV_COLOR_BLUE;
   bluestyle.body.grad_color = LV_COLOR_BLUE;
   bluestyle.body.border.color = LV_COLOR_GRAY;
   bluestyle.body.border.width = 2;
   bluestyle.text.color = LV_COLOR_WHITE;

   lv_obj_t * label1 = lv_obj_get_child(btn, NULL);
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
   lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 10, 10); //Align to the center

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
   lv_style_copy(&parkstyle, &lv_style_plain);    //Copy a built-in style to initialize the new style
   parkstyle.body.main_color = LV_COLOR_YELLOW;
   parkstyle.body.grad_color = LV_COLOR_YELLOW;
   parkstyle.body.border.color = LV_COLOR_GRAY;
   parkstyle.body.border.width = 2;
   parkstyle.text.color = LV_COLOR_BLACK;
   lv_obj_t * label1 = lv_obj_get_child(btn, NULL); //The label is the only child
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

*/
lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;

lv_style_t * myButtonStyleREL; //relesed style
lv_style_t * myButtonStylePR; //pressed style
lv_obj_t * scr2 = lv_obj_create(NULL,NULL);
lv_obj_t * scr1 = lv_obj_create(NULL,NULL);
lv_obj_t * line1;
lv_obj_t * testButton;
int x = 0;
static lv_res_t btn_click_action(lv_obj_t * btn)
{

    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        char buffer[100];
		sprintf(buffer, "AUTON", pros::millis());
		lv_label_set_text(myLabel, buffer);
      if (x == 0 ){
        lv_scr_load(scr2);
        lv_obj_set_parent(myButton, scr2);
        x = 1;
      } else if (x == 1) {
        lv_scr_load(scr1);
        lv_obj_set_parent(myButton, scr1);
        x = 0;

      }
      printf("test %i", x);
    }

    return LV_RES_OK;
}
lv_obj_t * createBtn(lv_obj_t * parent, lv_coord_t x, lv_coord_t y, lv_coord_t width, lv_coord_t height, //function courtesy of 81K: https://team81k.github.io/ProsLVGLTutorial/
    int id, const char * title)
{
    lv_obj_t * btn = lv_btn_create(parent, NULL);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, width, height);
    lv_obj_set_free_num(btn, id);

    lv_obj_t * label = lv_label_create(btn, NULL);
    lv_label_set_text(label, title);
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

    return btn;
}

lv_style_t * createBasicStyle(lv_style_t style_temp, lv_color_t mainColor, lv_color_t gradColor, int radius, lv_color_t textColor){ //sets up style for buttons pretty much

  lv_style_t * basicStyle = (lv_style_t *)malloc(sizeof(lv_style_t));

  lv_style_copy(basicStyle,&style_temp);
  basicStyle->body.main_color = mainColor;
  basicStyle->body.grad_color = gradColor;
  basicStyle->body.radius = radius;
  basicStyle->text.color = textColor;


  return basicStyle;

}

void initialize() {/*Create a three buttons, color, side, display auton */
 lv_scr_load(scr1);


  //lv_style_copy(myButtonStyleREL, &lv_style_plain);
  /*myButtonStyleREL.body.main_color = LV_COLOR_SILVER;
  myButtonStyleREL.body.grad_color = LV_COLOR_BLUE;
  myButtonStyleREL.body.radius = 2;
  myButtonStyleREL.text.color = LV_COLOR_GREEN;*/
  /*myButtonStyleREL.body.main_color = LV_COLOR_SILVER;
  myButtonStyleREL->body.grad_color = LV_COLOR_BLUE;
  myButtonStyleREL->body.radius = 2;
  myButtonStyleREL->text.color = LV_COLOR_GREEN;*/
  /*lv_style_copy(&myButtonStylePR, &lv_style_plain);
  myButtonStylePR.body.main_color = LV_COLOR_BLUE;
  myButtonStylePR.body.grad_color = LV_COLOR_GREEN;
  myButtonStylePR.body.radius = 2;
  myButtonStylePR.text.color = LV_COLOR_SILVER;*/

  //myButton = lv_btn_create(lv_scr_act(), NULL); //create button, lv_scr_act() is deafult screen object
  //lv_obj_set_free_num(myButton, 0); //set button is to 0
  //lv_obj_set_size(myButton, 200, 50); //set the button size
  //myButtonLabel = lv_label_create(myButton, NULL); //create label and puts it inside of the button
  //lv_label_set_text(myButtonLabel, "Click the Button"); //sets label text

  myButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_BLUE,2,LV_COLOR_GREEN);
  myButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_BLUE,LV_COLOR_GREEN,2,LV_COLOR_SILVER);


    myButton = createBtn(lv_scr_act(), 0,0,200,50, 0, "click the button");
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action); //set function to be called on button click
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, myButtonStyleREL); //set the relesed style
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, myButtonStylePR); //set the pressed style
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_RIGHT_MID, 10, 10); //set the position to top mid



    myLabel = lv_label_create(scr1, NULL); //create label and puts it on the screen
    lv_label_set_text(myLabel, "NONE"); //sets label text
    lv_obj_align(myLabel, NULL, LV_ALIGN_OUT_RIGHT_TOP, -150, 0); //set the position to center
    //TOP LINE
    static lv_style_t style_line1;
    lv_style_copy(&style_line1, &lv_style_plain);
    style_line1.line.color = LV_COLOR_SILVER;
    style_line1.line.width = 2;
    static lv_point_t line_points[] = { {0, 25}, {400, 25}};

    line1 = lv_line_create(scr1, NULL);
    lv_line_set_style(line1,&style_line1);
    lv_line_set_points(line1, line_points, 2); /*Set the points*/

    //lv_obj_align(line1, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);



   /*Create a Label on the currently active screen*/
   lv_obj_t * label1 =  lv_label_create(scr1, NULL);

   /*Modify the Label's text*/
   lv_label_set_text(label1, "Hello world!");

   /* Align the Label to the center
    * NULL means align on parent (which is the screen now)
    * 0, 0 at the end means an x, y offset after alignment*/
   lv_obj_align(label1, NULL, LV_ALIGN_OUT_LEFT_TOP, 100, 0);

   testButton = createBtn(scr1, 100, 100, 100, 50, 1, "TEST");
   lv_obj_align(testButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 40,100); //set the position to top mid
/*
     static lv_style_t initredstyle;
     lv_style_copy(&initredstyle, &lv_style_plain);    //Copy a built-in style to initialize the new style
     initredstyle.body.main_color = LV_COLOR_RED;
     initredstyle.body.grad_color = LV_COLOR_RED;
     initredstyle.body.border.color = LV_COLOR_GRAY;
     initredstyle.body.border.width = 2;
     initredstyle.text.color = LV_COLOR_WHITE;

     static lv_style_t parkstyle;
     lv_style_copy(&parkstyle, &lv_style_plain);
     parkstyle.body.main_color = LV_COLOR_YELLOW;
     parkstyle.body.grad_color = LV_COLOR_YELLOW;
     parkstyle.body.border.color = LV_COLOR_GRAY;
     parkstyle.body.border.width = 2;
     parkstyle.text.color = LV_COLOR_BLACK;

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);         //Create a button on the currently loaded screen
    lv_obj_set_style(btn1, &initredstyle);
    lv_obj_t * label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Red");
		lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, color_action); //Set function to be called when the button is released
    lv_obj_align(btn1, label, LV_ALIGN_OUT_BOTTOM_LEFT, -30, 20);  //Align below the label


    //Copy the previous button
    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), btn1);        //Second parameter is an object to copy
    lv_obj_set_style(btn2, &initredstyle);
    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label, "Close");
    lv_btn_set_action(btn2, LV_BTN_ACTION_CLICK, side_action);
    lv_obj_align(btn2, btn1, LV_ALIGN_OUT_RIGHT_MID, 35, 0);    //Align next to the prev. button.

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
*/

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
