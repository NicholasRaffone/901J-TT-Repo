#include "main.h"
#include "display/lv_conf.h"
#include "config.hpp"
#include <string>
#include <cstdio>

lv_obj_t * redAutonButton;
lv_obj_t * blueAutonButton;
lv_obj_t * skillsAutonButton;
lv_obj_t * debugButton;
lv_obj_t * batteryLabel;
lv_obj_t * autonState;
lv_style_t * redButtonStyleREL; //released style
lv_style_t * redButtonStylePR; //pressed style
lv_style_t * blueButtonStyleREL; //released style
lv_style_t * blueButtonStylePR; //pressed style
lv_style_t * skillsButtonStyleREL; //released style
lv_style_t * skillsButtonStylePR; //pressed style
lv_style_t * debugButtonStyleREL; //released style
lv_style_t * debugButtonStylePR; //pressed style
lv_obj_t * scr2 = lv_obj_create(NULL,NULL);
lv_obj_t * scr1 = lv_obj_create(NULL,NULL);
lv_obj_t * line1;
lv_obj_t * testButton;
lv_obj_t * autonLabel;

int screenNum = 0;

void updateBattery(){ //don't want to use task since might slow down brain too much
  char buffer[100];
  sprintf(buffer, "BATTERY: %d", (int)pros::battery::get_capacity());
  lv_label_set_text(batteryLabel, buffer);
}

lv_obj_t * drawRectangle( int x, int y, int width, int height, lv_color_t fillColor, lv_color_t borderColor ) { //function courtesy of jpearman, https://www.vexforum.com/t/lvgl-how-to-draw-a-rectangle-using-lvgl/50977/5, modified slightly
  lv_obj_t * obj1 = lv_obj_create(lv_scr_act(), NULL);

  lv_style_t *style1 = (lv_style_t *)malloc( sizeof( lv_style_t ));
  lv_style_copy(style1, &lv_style_plain_color);    /*Copy a built-in style to initialize the new style*/
  style1->body.empty = 0;
  style1->body.main_color = fillColor;
  style1->body.border.color = borderColor;
  style1->body.border.width = 1;
  style1->body.border.part = LV_BORDER_FULL;

  lv_obj_set_style(obj1, style1);
  lv_obj_set_pos(obj1, x, y);
  lv_obj_set_size(obj1, width, height);

  return obj1;
}

static lv_res_t btn_click_action_screen(lv_obj_t * btn) //function courtesy of team81k,
{

    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons

    if(id == 0)
    {
        updateBattery();
      if (screenNum == 0 ){
        lv_scr_load(scr2);
        lv_obj_set_parent(line1, scr2);
        lv_obj_set_parent(autonLabel,scr2);
        lv_obj_set_parent(batteryLabel,scr2);
        //lv_obj_set_parent(autonState,scr2);

        screenNum = 1;
      } else if (screenNum == 1) {
        lv_scr_load(scr1);
        lv_obj_set_parent(line1, scr1);
        lv_obj_set_parent(autonLabel,scr1);
        lv_obj_set_parent(batteryLabel,scr1);
        //lv_obj_set_parent(autonState,scr1);


        screenNum = 0;

      }
      printf("test %i", screenNum);
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

lv_style_t * createBasicStyle(lv_style_t style_temp, lv_color_t mainColor, lv_color_t gradColor, int radius, lv_color_t textColor){ //sets up style for buttons pretty much, made by Joey Wildman

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
 autonState = drawRectangle( 0, 0, 210, 25, LV_COLOR_GRAY,LV_COLOR_WHITE);


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

    redButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_RED,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    redButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_RED);

    redAutonButton = createBtn(lv_scr_act(), 0,0,200,80, 0, "RED AUTON");
    lv_btn_set_action(redAutonButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(redAutonButton, LV_BTN_STYLE_REL, redButtonStyleREL); //set the relesed style
    lv_btn_set_style(redAutonButton, LV_BTN_STYLE_PR, redButtonStylePR); //set the pressed style
    lv_obj_align(redAutonButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 470, 40); //set the position to top mid


    blueButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_BLUE,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    blueButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_BLUE);

    blueAutonButton = createBtn(lv_scr_act(), 0,0,200,80, 0, "BLUE AUTON");
    lv_btn_set_action(blueAutonButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(blueAutonButton, LV_BTN_STYLE_REL, blueButtonStyleREL); //set the relesed style
    lv_btn_set_style(blueAutonButton, LV_BTN_STYLE_PR, blueButtonStylePR); //set the pressed style
    lv_obj_align(blueAutonButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 210, 40); //set the position to top mid


    skillsButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_GREEN,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    skillsButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_GREEN);

    skillsAutonButton = createBtn(lv_scr_act(), 0,0,200,80, 0, "SKILLS");
    lv_btn_set_action(skillsAutonButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(skillsAutonButton, LV_BTN_STYLE_REL, skillsButtonStyleREL); //set the relesed style
    lv_btn_set_style(skillsAutonButton, LV_BTN_STYLE_PR, skillsButtonStylePR); //set the pressed style
    lv_obj_align(skillsAutonButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 210, 140); //set the position to top mid


    debugButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_MAGENTA,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    debugButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_MAGENTA);

    debugButton = createBtn(lv_scr_act(), 0,0,200,80, 0, "DEBUG");
    lv_btn_set_action(debugButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(debugButton, LV_BTN_STYLE_REL, debugButtonStyleREL); //set the relesed style
    lv_btn_set_style(debugButton, LV_BTN_STYLE_PR, debugButtonStylePR); //set the pressed style
    lv_obj_align(debugButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 470, 140); //set the position to top mid

    batteryLabel = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    updateBattery();
    lv_obj_align(batteryLabel, NULL, LV_ALIGN_OUT_RIGHT_TOP, -150, 0); //set the position to center

    //TOP LINE
    static lv_style_t style_line1;
    lv_style_copy(&style_line1, &lv_style_plain);
    style_line1.line.color = LV_COLOR_SILVER;
    style_line1.line.width = 2;
    static lv_point_t line_points[] = { {0, 25}, {480, 25}};
    line1 = lv_line_create(lv_scr_act(), NULL);
    lv_line_set_style(line1,&style_line1);
    lv_line_set_points(line1, line_points, 2); /*Set the points*/

    //lv_obj_align(line1, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);



   /*Create a Label on the currently active screen*/
   autonLabel =  lv_label_create(lv_scr_act(), NULL);
   /*Modify the Label's text*/
   lv_label_set_text(autonLabel, "NO AUTON SELECTED");

   /* Align the Label to the center
    * NULL means align on parent (which is the screen now)
    * 0, 0 at the end means an x, y offset after alignment*/
   lv_obj_set_pos(autonLabel, 3, 4);



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
