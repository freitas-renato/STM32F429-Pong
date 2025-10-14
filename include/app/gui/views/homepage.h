#ifndef HOMEPAGE_H
#define HOMEPAGE_H

#include <lvgl.h>
#include <stdbool.h>

//TODO: docs

void homepage_ui_create(lv_obj_t* parent);

void homepage_ui_update_ball_position(int8_t h_dir, int8_t v_dir);

int homepage_ui_get_ball_x_position(void);
int homepage_ui_get_ball_y_position(void);
void homepage_ui_ball_reset_position(void);

int homepage_ui_get_left_paddle_y_position(void);
int homepage_ui_get_left_paddle_x_position(void);

int homepage_ui_get_right_paddle_y_position(void);
int homepage_ui_get_right_paddle_x_position(void);

void left_paddle_move(int direction);
void right_paddle_move(int direction);

bool homepage_ui_is_ball_touching_paddle_left();
bool homepage_ui_is_ball_touching_paddle_right();

bool homepage_ui_is_ball_touching_top();
bool homepage_ui_is_ball_touching_bottom();

void homepage_ui_update_score(int player1, int player2);

#endif  // HOMEPAGE_H
