#include "main.h"
#include "display/lv_conf.h"
#include "config.hpp"
#include <string>
#include <cstdio>

lv_obj_t * redAutonButton;
lv_obj_t * blueAutonButton;
lv_obj_t * skillsAutonButton;
lv_obj_t * debugButton;
lv_obj_t * goBackButton;
lv_obj_t * redAutonButtons [4];
lv_obj_t * blueAutonButtons [4];
lv_obj_t * skillsAutonButtons [4];



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
lv_style_t * goBackStyleREL; //released style
lv_style_t * goBackStylePR; //pressed style
lv_obj_t * redScr = lv_obj_create(NULL,NULL);
lv_obj_t * blueScr = lv_obj_create(NULL,NULL);
lv_obj_t * skillsScr = lv_obj_create(NULL,NULL);
lv_obj_t * debugScr = lv_obj_create(NULL,NULL);
lv_obj_t * mainScr = lv_obj_create(NULL,NULL);



lv_obj_t * line1;
lv_obj_t * testButton;
lv_obj_t * autonLabel;
lv_obj_t * debugLabel [6];


int screenNum = 0;
int screenLoad [4] = {0,0,0,0};
char selectedAutonDesc [100];
bool isDebug = false;

void updateBattery(){ //don't want to use task since might slow down brain too much, just updates label displaying battery percentage
  char buffer[100];
  sprintf(buffer, "BATTERY: %d", (int)pros::battery::get_capacity());
  lv_label_set_text(batteryLabel, buffer);
}

void loadDefaultObj(lv_obj_t * scr){ //loads objects that are rendered in every screen, also loads screen from parameter
  lv_scr_load(scr);
  lv_obj_set_parent(line1, scr);
  lv_obj_set_parent(autonLabel,scr);
  lv_obj_set_parent(batteryLabel,scr);
}


lv_obj_t * drawRectangle( int x, int y, int width, int height, lv_color_t fillColor, lv_color_t borderColor ) { //function courtesy of jpearman, https://www.vexforum.com/t/lvgl-how-to-draw-a-rectangle-using-lvgl/50977/5, modified slightly, creates rectangle
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

lv_obj_t * createBtn(lv_obj_t * parent, lv_coord_t x, lv_coord_t y, lv_coord_t width, lv_coord_t height, //function courtesy of 81K: https://team81k.github.io/ProsLVGLTutorial/ creates buttons
    int id, const char * title)
{
    lv_obj_t * btn = lv_btn_create(parent, NULL); //constructor
    lv_obj_set_pos(btn, x, y); //attributes
    lv_obj_set_size(btn, width, height);
    lv_obj_set_free_num(btn, id);

    lv_obj_t * label = lv_label_create(btn, NULL); //label for button
    lv_label_set_text(label, title);
    lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

    return btn;
}

lv_style_t * createBasicStyle(lv_style_t style_temp, lv_color_t mainColor, lv_color_t gradColor, int radius, lv_color_t textColor){ //sets up style for buttons pretty much, made by Joey Wildman

  lv_style_t * basicStyle = (lv_style_t *)malloc(sizeof(lv_style_t)); //allocate memory for lvgl style object

  lv_style_copy(basicStyle,&style_temp); //copy style from parameter
  basicStyle->body.main_color = mainColor;
  basicStyle->body.grad_color = gradColor;
  basicStyle->body.radius = radius;
  basicStyle->text.color = textColor;


  return basicStyle;

}
static lv_res_t btn_click_auton(lv_obj_t * btn){ //handles auton selection when auton button pressed
  updateBattery();
  selectedAuton = lv_obj_get_free_num(btn);
  printf("%d\r\n", selectedAuton);
  lv_obj_t * tester = lv_obj_get_child(btn,NULL); //gets label, which is a child of button
  char buffer[100];
  sprintf(buffer, "%s", lv_label_get_text(tester)); //gets label text
  lv_label_set_text(autonLabel, buffer);  //updates label that displays selected auton




    lv_obj_t * mbox1 = lv_mbox_create(lv_scr_act(), NULL); //creates pop-up
    lv_obj_set_size(mbox1, 450, 500);
    lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, -10);
    sprintf(selectedAutonDesc, "%s has been selected", lv_label_get_text(tester)); //gets label text
    lv_mbox_set_text(mbox1, selectedAutonDesc); //displays selected auton
    lv_mbox_start_auto_close(mbox1, 2000); //closes after two seconds
  return LV_RES_OK;
}

void debug_task(void * p){

    char buffer[100];
    sprintf(buffer,"Xpos: %f",mainPosition.x);
    lv_label_set_text(debugLabel [0], buffer);
    sprintf(buffer,"Ypos: %f",mainPosition.y);
    lv_label_set_text(debugLabel [1], buffer);
    sprintf(buffer,"Angle: %f",mainPosition.angle);
    lv_label_set_text(debugLabel [2], buffer);
    sprintf(buffer,"l: %f",leftenc.get());
    lv_label_set_text(debugLabel [3], buffer);
    sprintf(buffer,"r: %f",rightenc.get());
    lv_label_set_text(debugLabel [4], buffer);
    sprintf(buffer,"b: %f",backenc.get());
    lv_label_set_text(debugLabel [5], buffer);

  /*printf("Xpos: %f\r\n",mainPosition.x);
  printf("Ypos: %f\r\n",mainPosition.y);
  printf("Angle: %f\r\n",mainPosition.angle);
  printf("l: %f\r\n",leftenc.get());
  printf("r: %f\r\n",rightenc.get());
  printf("b: %f\r\n",backenc.get());*/
}

static lv_res_t btn_click_action_screen(lv_obj_t * btn) //handles screen changes
{

    uint8_t id = lv_obj_get_free_num(btn); //id usefull when there are multiple buttons
    updateBattery();

    if(id == 0) //red auton button
    {
      loadDefaultObj(redScr); //loads default objects
      lv_obj_align(goBackButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 340, 170); //set the position to top mid
      lv_obj_set_parent(goBackButton, redScr);  //loads back button

      if (screenLoad[0] == 0){ //if page hasn't been previously loaded

        redAutonButtons[0] = createBtn(redScr,0,0,200,60,10, "RED 1"); //auton selection buttons
        redAutonButtons[1] = createBtn(redScr,0,0,200,60,11, "RED 2");
        redAutonButtons[2] = createBtn(redScr,0,0,200,60,12, "RED 3");
        redAutonButtons[3] = createBtn(redScr,0,0,200,60,13, "RED 4");

        for (int i = 0; i < 4; i++){ //more init of auton buttons
          if (i<2){lv_obj_align(redAutonButtons[i], NULL, LV_ALIGN_OUT_LEFT_TOP, 210+260*i, 40); }
          else if(i<4){lv_obj_align(redAutonButtons[i], NULL, LV_ALIGN_OUT_LEFT_TOP, 210+260*(i-2), 105);}
          lv_btn_set_action(redAutonButtons[i], LV_BTN_ACTION_CLICK, btn_click_auton); //set function to be called on button click
          lv_btn_set_style(redAutonButtons[i], LV_BTN_STYLE_REL, redButtonStyleREL); //set the relesed style
          lv_btn_set_style(redAutonButtons[i], LV_BTN_STYLE_PR, redButtonStylePR); //set the pressed style
        }
        screenLoad[0] = 1; //page is now loaded, no need to load it again
      }
      screenNum = 1; //current screen is now red auton
    } else if (id == 1){ //blue auton button
      if (screenNum == 0){
        loadDefaultObj(blueScr);
        lv_obj_align(goBackButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 340, 170); //set the position to top mid
        lv_obj_set_parent(goBackButton, blueScr);

        if (screenLoad[1] == 0){

          blueAutonButtons[0] = createBtn(blueScr,0,0,200,60,20, "BLUE 1");
          blueAutonButtons[1] = createBtn(blueScr,0,0,200,60,21, "BLUE 2");
          blueAutonButtons[2] = createBtn(blueScr,0,0,200,60,22, "BLUE 3");
          blueAutonButtons[3] = createBtn(blueScr,0,0,200,60,23, "BLUE 4");

          for (int i = 0; i < 4; i++){
            if (i<2){lv_obj_align(blueAutonButtons[i], NULL, LV_ALIGN_OUT_LEFT_TOP, 210+260*i, 40); }
            else if(i<4){lv_obj_align(blueAutonButtons[i], NULL, LV_ALIGN_OUT_LEFT_TOP, 210+260*(i-2), 105);}
            lv_btn_set_action(blueAutonButtons[i], LV_BTN_ACTION_CLICK, btn_click_auton); //set function to be called on button click
            lv_btn_set_style(blueAutonButtons[i], LV_BTN_STYLE_REL, blueButtonStyleREL); //set the relesed style
            lv_btn_set_style(blueAutonButtons[i], LV_BTN_STYLE_PR, blueButtonStylePR); //set the pressed style
          }
          screenLoad[1] = 1;
        }
        screenNum = 2;
      }

    } else if (id == 2){ //skills button
      if (screenNum == 0){
        loadDefaultObj(skillsScr);
        lv_obj_align(goBackButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 340, 170); //set the position to top mid
        lv_obj_set_parent(goBackButton, skillsScr);

        if (screenLoad[2] == 0){

          skillsAutonButtons[0] = createBtn(skillsScr,0,0,200,60,30, "SKILLS 1");
          skillsAutonButtons[1] = createBtn(skillsScr,0,0,200,60,31, "SKILLS 2");
          skillsAutonButtons[2] = createBtn(skillsScr,0,0,200,60,32, "SKILLS 3");
          skillsAutonButtons[3] = createBtn(skillsScr,0,0,200,60,33, "SKILLS 4");

          for (int i = 0; i < 4; i++){
            if (i<2){lv_obj_align(skillsAutonButtons[i], NULL, LV_ALIGN_OUT_LEFT_TOP, 210+260*i, 40); }
            else if(i<4){lv_obj_align(skillsAutonButtons[i], NULL, LV_ALIGN_OUT_LEFT_TOP, 210+260*(i-2), 105);}
            lv_btn_set_action(skillsAutonButtons[i], LV_BTN_ACTION_CLICK, btn_click_auton); //set function to be called on button click
            lv_btn_set_style(skillsAutonButtons[i], LV_BTN_STYLE_REL, skillsButtonStyleREL); //set the relesed style
            lv_btn_set_style(skillsAutonButtons[i], LV_BTN_STYLE_PR, skillsButtonStylePR); //set the pressed style
          }
          screenLoad[2] = 1;
        }
        screenNum = 3;
      }

    } else if (id == 3){ //debug button
      if (screenNum == 0){
        loadDefaultObj(debugScr);
        lv_obj_align(goBackButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 340, 170); //set the position to top mid
        lv_obj_set_parent(goBackButton, debugScr);
        if(screenLoad[3] == 0){


          for (int i = 0; i < 6; i++){
              debugLabel [i] =  lv_label_create(debugScr, NULL);
              lv_label_set_text(debugLabel[i], "TEST");
              lv_obj_align(debugLabel[i], NULL, LV_ALIGN_OUT_LEFT_TOP,50,40+20*i);
          }
          /*Modify the Label's text*/
          //lv_label_set_text(debugLabel, "NO AUTON SELECTED");
          //lv_obj_align(debugLabel, NULL, LV_ALIGN_CENTER,0,0);
          lv_task_create(debug_task, 100, LV_TASK_PRIO_MID, NULL);
        }
        isDebug = true;
        screenNum = 4;
      }

    } else if (id == 4){ //go back button
      loadDefaultObj(mainScr);
      isDebug = false;
      screenNum = 0;
    }
    printf("screen %i\r\n", screenNum);

    return LV_RES_OK;
}

void initialize() {/*Create a three buttons, color, side, display auton */
  leftenc.reset();
  rightenc.reset();
  pros::delay(500);
    lv_scr_load(mainScr);
    autonState = drawRectangle( 0, 0, 210, 25, LV_COLOR_GRAY,LV_COLOR_WHITE);

    goBackButton = createBtn(lv_scr_act(), 0,0,200,60, 4, "EXIT");
    goBackStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_ORANGE,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    goBackStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_ORANGE);
    lv_btn_set_action(goBackButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(goBackButton, LV_BTN_STYLE_REL, goBackStyleREL); //set the relesed style
    lv_btn_set_style(goBackButton, LV_BTN_STYLE_PR, goBackStylePR); //set the pressed style
    lv_obj_align(goBackButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 470, 500); //set the position to top mid

    redButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_RED,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    redButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_RED);
    redAutonButton = createBtn(mainScr, 0,0,200,80, 0, "RED AUTON");
    lv_btn_set_action(redAutonButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(redAutonButton, LV_BTN_STYLE_REL, redButtonStyleREL); //set the relesed style
    lv_btn_set_style(redAutonButton, LV_BTN_STYLE_PR, redButtonStylePR); //set the pressed style
    lv_obj_align(redAutonButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 470, 40); //set the position to top mid

    blueButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_BLUE,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    blueButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_BLUE);
    blueAutonButton = createBtn(mainScr, 0,0,200,80, 1, "BLUE AUTON");
    lv_btn_set_action(blueAutonButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(blueAutonButton, LV_BTN_STYLE_REL, blueButtonStyleREL); //set the relesed style
    lv_btn_set_style(blueAutonButton, LV_BTN_STYLE_PR, blueButtonStylePR); //set the pressed style
    lv_obj_align(blueAutonButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 210, 40); //set the position to top mid

    skillsButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_GREEN,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    skillsButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_GREEN);
    skillsAutonButton = createBtn(mainScr, 0,0,200,80, 2, "SKILLS");
    lv_btn_set_action(skillsAutonButton, LV_BTN_ACTION_CLICK, btn_click_action_screen); //set function to be called on button click
    lv_btn_set_style(skillsAutonButton, LV_BTN_STYLE_REL, skillsButtonStyleREL); //set the relesed style
    lv_btn_set_style(skillsAutonButton, LV_BTN_STYLE_PR, skillsButtonStylePR); //set the pressed style
    lv_obj_align(skillsAutonButton, NULL, LV_ALIGN_OUT_LEFT_TOP, 210, 140); //set the position to top mid

    debugButtonStyleREL = createBasicStyle(lv_style_pretty,LV_COLOR_MAGENTA,LV_COLOR_SILVER,2,LV_COLOR_WHITE);
    debugButtonStylePR = createBasicStyle(lv_style_pretty,LV_COLOR_SILVER,LV_COLOR_WHITE,2,LV_COLOR_MAGENTA);
    debugButton = createBtn(mainScr, 0,0,200,80, 3, "DEBUG");
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
