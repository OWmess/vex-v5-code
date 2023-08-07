#pragma once
#include "display/lvgl.h"

class PID_tuning {
    PID_tuning(){
        lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
        lv_label_set_text(label, "Hello world!");
        lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
    }
    ~PID_tuning(){
        lv_obj_clean(lv_scr_act());
    }
};