#include "app/gui/components/scoreboard.h"

#include <string.h>
#include <stdio.h>

static lv_group_t* m_scoreboard_group = NULL;

static lv_obj_t* score_left_label = NULL;
static lv_obj_t* score_right_label = NULL;

static int m_score_left = 0;
static int m_score_right = 0;

void scoreboard_create(lv_obj_t* parent) {
    if (parent == NULL) {
        parent = lv_screen_active();
    }

    if (m_scoreboard_group != NULL) {
        // Scoreboard already created
        return;
    }

    m_scoreboard_group = lv_group_create();
    if (m_scoreboard_group == NULL) {
        // Handle error
        return;
    }

    // Create left score label
    score_left_label = lv_label_create(parent);
    lv_label_set_text(score_left_label, "0");
    lv_obj_set_style_text_font(score_left_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(score_left_label, lv_color_white(), 0);
    lv_obj_align(score_left_label, LV_ALIGN_TOP_MID, -40, 10);
    lv_group_add_obj(m_scoreboard_group, score_left_label);

    // Create right score label
    score_right_label = lv_label_create(parent);
    lv_label_set_text(score_right_label, "0");
    lv_obj_set_style_text_font(score_right_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(score_right_label, lv_color_white(), 0);
    lv_obj_align(score_right_label, LV_ALIGN_TOP_MID, 40, 10);
    lv_group_add_obj(m_scoreboard_group, score_right_label);
}

void scoreboard_update(lv_obj_t* parent, int score_left, int score_right) {
    if (m_scoreboard_group == NULL) {
        // Scoreboard not created yet
        return;
    }

    m_score_left  = score_left;
    m_score_right = score_right;

    // lv_obj_t* score_left_label  = lv_obj_get_child(m_scoreboard_group, 0);
    // lv_obj_t* score_right_label = lv_obj_get_child(m_scoreboard_group, 1);

    if (score_left_label != NULL) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%d", m_score_left);
        lv_label_set_text(score_left_label, buf);
    }

    if (score_right_label != NULL) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%d", m_score_right);
        lv_label_set_text(score_right_label, buf);
    }
}

void scoreboard_reset(lv_obj_t* parent) {
    if (m_scoreboard_group == NULL) {
        // Scoreboard not created yet
        return;
    }

    m_score_left  = 0;
    m_score_right = 0;

    scoreboard_update(parent, m_score_left, m_score_right);
}

void scoreboard_show(lv_obj_t* parent, bool show) {
    if (m_scoreboard_group == NULL) {
        // Scoreboard not created yet
        return;
    }

    // lv_obj_t* score_left_label  = lv_obj_get_child(m_scoreboard_group, 0);
    // lv_obj_t* score_right_label = lv_obj_get_child(m_scoreboard_group, 1);

    if (score_left_label != NULL && !show) {
        if (show) {
            lv_obj_clear_flag(score_left_label, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(score_left_label, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (score_right_label != NULL && !show) {
        if (show) {
            lv_obj_clear_flag(score_right_label, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(score_right_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
}
