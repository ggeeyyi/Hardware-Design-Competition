/*
* Copyright 2023 NXP
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"


void setup_scr_Musicsheetpresenter(lv_ui *ui)
{
	//Write codes Musicsheetpresenter
	ui->Musicsheetpresenter = lv_obj_create(NULL);
	lv_obj_set_size(ui->Musicsheetpresenter, 480, 320);
    lv_obj_clear_flag(guider_ui.Musicsheetpresenter, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_scrollbar_mode(ui->Musicsheetpresenter, LV_SCROLLBAR_MODE_OFF);

	//Write style for Musicsheetpresenter, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Musicsheetpresenter, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Musicsheetpresenter, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Musicsheetpresenter_cont_1
	ui->Musicsheetpresenter_cont_1 = lv_obj_create(ui->Musicsheetpresenter);
	lv_obj_set_pos(ui->Musicsheetpresenter_cont_1, 0, 0);
	lv_obj_set_size(ui->Musicsheetpresenter_cont_1, 480, 59);
	lv_obj_set_scrollbar_mode(ui->Musicsheetpresenter_cont_1, LV_SCROLLBAR_MODE_OFF);

	//Write style for Musicsheetpresenter_cont_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Musicsheetpresenter_cont_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_border_opa(ui->Musicsheetpresenter_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_border_color(ui->Musicsheetpresenter_cont_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Musicsheetpresenter_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Musicsheetpresenter_cont_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Musicsheetpresenter_cont_1, lv_color_hex(0x2F3243), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Musicsheetpresenter_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Musicsheetpresenter_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Musicsheetpresenter_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Musicsheetpresenter_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Musicsheetpresenter_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Musicsheetpresenter_btn_3
	ui->Musicsheetpresenter_btn_3 = lv_btn_create(ui->Musicsheetpresenter_cont_1);
	ui->Musicsheetpresenter_btn_3_label = lv_label_create(ui->Musicsheetpresenter_btn_3);
	lv_label_set_text(ui->Musicsheetpresenter_btn_3_label, "PAUSED");
	lv_label_set_long_mode(ui->Musicsheetpresenter_btn_3_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Musicsheetpresenter_btn_3_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Musicsheetpresenter_btn_3, 0, LV_STATE_DEFAULT);
	lv_obj_set_pos(ui->Musicsheetpresenter_btn_3, 378, 12);
	lv_obj_set_size(ui->Musicsheetpresenter_btn_3, 86, 30);
	lv_obj_set_scrollbar_mode(ui->Musicsheetpresenter_btn_3, LV_SCROLLBAR_MODE_OFF);

	//Write style for Musicsheetpresenter_btn_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Musicsheetpresenter_btn_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Musicsheetpresenter_btn_3, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Musicsheetpresenter_btn_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Musicsheetpresenter_btn_3, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Musicsheetpresenter_btn_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Musicsheetpresenter_btn_3, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Musicsheetpresenter_btn_3, &lv_font_simsun_16, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Musicsheetpresenter_btn_3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Musicsheetpresenter_btn_1
	ui->Musicsheetpresenter_btn_1 = lv_btn_create(ui->Musicsheetpresenter);
	ui->Musicsheetpresenter_btn_1_label = lv_label_create(ui->Musicsheetpresenter_btn_1);
	lv_label_set_text(ui->Musicsheetpresenter_btn_1_label, "Return\n");
	lv_label_set_long_mode(ui->Musicsheetpresenter_btn_1_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->Musicsheetpresenter_btn_1_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->Musicsheetpresenter_btn_1, 0, LV_STATE_DEFAULT);
	lv_obj_set_pos(ui->Musicsheetpresenter_btn_1, 6, 10);
	lv_obj_set_size(ui->Musicsheetpresenter_btn_1, 51, 28);
	lv_obj_set_scrollbar_mode(ui->Musicsheetpresenter_btn_1, LV_SCROLLBAR_MODE_OFF);

	//Write style for Musicsheetpresenter_btn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->Musicsheetpresenter_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_color(ui->Musicsheetpresenter_btn_1, lv_color_hex(0x009ea9), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_border_width(ui->Musicsheetpresenter_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Musicsheetpresenter_btn_1, 25, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Musicsheetpresenter_btn_1, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_color(ui->Musicsheetpresenter_btn_1, lv_color_hex(0x0d4b3b), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_opa(ui->Musicsheetpresenter_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_spread(ui->Musicsheetpresenter_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_ofs_x(ui->Musicsheetpresenter_btn_1, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_ofs_y(ui->Musicsheetpresenter_btn_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Musicsheetpresenter_btn_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Musicsheetpresenter_btn_1, &lv_font_simsun_12, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Musicsheetpresenter_btn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes Musicsheetpresenter_label_1
	ui->Musicsheetpresenter_label_1 = lv_label_create(ui->Musicsheetpresenter);
	lv_label_set_text(ui->Musicsheetpresenter_label_1, "Sheet music presenter");
	lv_label_set_long_mode(ui->Musicsheetpresenter_label_1, LV_LABEL_LONG_WRAP);
	lv_obj_set_pos(ui->Musicsheetpresenter_label_1, 121, 20);
	lv_obj_set_size(ui->Musicsheetpresenter_label_1, 280, 32);
	lv_obj_set_scrollbar_mode(ui->Musicsheetpresenter_label_1, LV_SCROLLBAR_MODE_OFF);

	//Write style for Musicsheetpresenter_label_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_border_width(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_radius(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(ui->Musicsheetpresenter_label_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->Musicsheetpresenter_label_1, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_letter_space(ui->Musicsheetpresenter_label_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_line_space(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->Musicsheetpresenter_label_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_top(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_right(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_bottom(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_pad_left(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->Musicsheetpresenter_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Update current screen layout.
	lv_obj_update_layout(ui->Musicsheetpresenter);

	
	//Init events for screen.
	events_init_Musicsheetpresenter(ui);
}
