/* Himax Android Driver Sample Code for debug nodes
*
* Copyright (C) 2017 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
#ifndef H_HIMAX_DEBUG
#define H_HIMAX_DEBUG

#include "himax_platform.h"
#include "himax_common.h"

extern int hx83102_gestrue_flag;	//zhaopengfei@wind-mobi.com 20180410 add

#ifdef HX_ESD_RECOVERY
extern u8 HX83102_ESD_RESET_ACTIVATE;
extern 	int hx83102_EB_event_flag;
extern 	int hx83102_EC_event_flag;
extern 	int hx83102_ED_event_flag;
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
#define HIMAX_PROC_TOUCH_FOLDER 	"android_touch"
#define HIMAX_PROC_DEBUG_LEVEL_FILE	"debug_level"
#define HIMAX_PROC_VENDOR_FILE		"vendor"
#define HIMAX_PROC_ATTN_FILE		"attn"
#define HIMAX_PROC_INT_EN_FILE		"int_en"
#define HIMAX_PROC_LAYOUT_FILE		"layout"
#define HIMAX_PROC_CRC_TEST_FILE		"CRC_test"

static struct proc_dir_entry *himax_touch_proc_dir 			= NULL;
static struct proc_dir_entry *himax_proc_debug_level_file 	= NULL;
static struct proc_dir_entry *himax_proc_vendor_file 		= NULL;
static struct proc_dir_entry *himax_proc_attn_file 			= NULL;
static struct proc_dir_entry *himax_proc_int_en_file 		= NULL;
static struct proc_dir_entry *himax_proc_layout_file 		= NULL;
static struct proc_dir_entry *himax_proc_CRC_test_file 		= NULL;

static uint8_t HX_PROC_SEND_FLAG;

extern int hx83102_touch_proc_init(void);
extern void hx83102_touch_proc_deinit(void);
bool hx83102_getFlashDumpGoing(void);

extern int hx83102_int_en_set(struct i2c_client *client);

#ifdef HX_TP_PROC_GUEST_INFO
#define HIMAX_PROC_GUEST_INFO_FILE		"guest_info"
static struct proc_dir_entry *himax_proc_guest_info_file 	= NULL;
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)
#define HIMAX_PROC_ITO_TEST_FILE		"ITO_test"
static struct proc_dir_entry *himax_proc_ito_test_file 		= NULL;

extern void ito_set_step_status(uint8_t status);
extern uint8_t ito_get_step_status(void);
extern void ito_set_result_status(uint8_t status);
extern uint8_t ito_get_result_status(void);

#endif

#ifdef HX_TP_PROC_REGISTER
#define HIMAX_PROC_REGISTER_FILE	"register"
static struct proc_dir_entry *himax_proc_register_file = NULL;
static uint8_t byte_length = 0;
static uint8_t register_command[4];
bool hx83102_cfg_flag = false;
#endif

#ifdef HX_TP_PROC_DIAG
#define HIMAX_PROC_DIAG_FILE	"diag"
static struct proc_dir_entry *himax_proc_diag_file = NULL;
#define HIMAX_PROC_DIAG_ARR_FILE	"diag_arr"
static struct proc_dir_entry *himax_proc_diag_arrange_file = NULL;
static struct file *diag_sram_fn;
static uint8_t write_counter = 0;
static uint8_t write_max_count = 30;
#define IIR_DUMP_FILE "/sdcard/HX_IIR_Dump.txt"
#define DC_DUMP_FILE "/sdcard/HX_DC_Dump.txt"
#define BANK_DUMP_FILE "/sdcard/HX_BANK_Dump.txt"

#ifdef HX_TP_PROC_2T2R
static uint8_t x_channel_2 = 0;
static uint8_t y_channel_2 = 0;
static uint32_t *diag_mutual_2 = NULL;

int32_t *hx83102_getMutualBuffer_2(void);
uint8_t 	hx83102_getXChannel_2(void);
uint8_t 	hx83102_getYChannel_2(void);

void 	hx83102_setMutualBuffer_2(void);
void 	hx83102_setXChannel_2(uint8_t x);
void 	hx83102_setYChannel_2(uint8_t y);
#endif
static uint8_t x_channel 		= 0;
static uint8_t y_channel 		= 0;
static int32_t *diag_mutual = NULL;
static int32_t *diag_mutual_new = NULL;
static int32_t *diag_mutual_old = NULL;
static uint8_t diag_max_cnt = 0;
static uint8_t hx_state_info[2] = {0};

static int g_diag_command = 0;
static int32_t diag_self[100] = {0};
uint8_t hx83102_diag_coor[128];// = {0xFF};

int32_t *hx83102_getMutualBuffer(void);
int32_t *hx83102_getMutualNewBuffer(void);
int32_t *hx83102_getMutualOldBuffer(void);
int32_t *hx83102_getSelfBuffer(void);
uint8_t 	hx83102_getDiagCommand(void);
uint8_t 	hx83102_getXChannel(void);
uint8_t 	hx83102_getYChannel(void);

void 	hx83102_setMutualBuffer(void);
void 	hx83102_setMutualNewBuffer(void);
void 	hx83102_setMutualOldBuffer(void);
void 	hx83102_setXChannel(uint8_t x);
void 	hx83102_setYChannel(uint8_t y);
#endif

#ifdef HX_TP_PROC_DEBUG
#define HIMAX_PROC_DEBUG_FILE	"debug"
static struct proc_dir_entry *himax_proc_debug_file = NULL;
#define HIMAX_PROC_FW_DEBUG_FILE	"FW_debug"
static struct proc_dir_entry *himax_proc_fw_debug_file = NULL;
#define HIMAX_PROC_DD_DEBUG_FILE	"DD_debug"
static struct proc_dir_entry *himax_proc_dd_debug_file = NULL;

static bool	fw_update_complete = false;
static int handshaking_result = 0;
static unsigned char debug_level_cmd = 0;
static uint8_t mutual_set_flag = 0;
uint8_t hx83102_cmd_set[8];
#endif

#ifdef HX_TP_PROC_FLASH_DUMP
#define HIMAX_PROC_FLASH_DUMP_FILE	"flash_dump"
static struct proc_dir_entry *himax_proc_flash_dump_file = NULL;

static int Flash_Size = 131072;
static uint8_t *flash_buffer 				= NULL;
static uint8_t flash_command 				= 0;
static uint8_t flash_read_step 			= 0;
static uint8_t flash_progress 			= 0;
static uint8_t flash_dump_complete	= 0;
static uint8_t flash_dump_fail 			= 0;
static uint8_t sys_operation				= 0;
static bool    flash_dump_going			= false;

static uint8_t getFlashDumpComplete(void);
static uint8_t getFlashDumpFail(void);
static uint8_t getFlashDumpProgress(void);
static uint8_t getFlashReadStep(void);
static uint8_t getFlashCommand(void);
uint8_t hx83102_getSysOperation(void);

static void setFlashCommand(uint8_t command);
static void setFlashReadStep(uint8_t step);

void hx83102_setFlashBuffer(void);
static void setFlashDumpComplete(uint8_t complete);
static void setFlashDumpFail(uint8_t fail);
static void setFlashDumpProgress(uint8_t progress);
void hx83102_setSysOperation(uint8_t operation);
static void setFlashDumpGoing(bool going);

#endif

#ifdef HX_TP_PROC_SELF_TEST
#define HIMAX_PROC_SELF_TEST_FILE	"self_test"
static struct proc_dir_entry *himax_proc_self_test_file = NULL;
#endif

#ifdef HX_TP_PROC_RESET
#define HIMAX_PROC_RESET_FILE		"reset"
static struct proc_dir_entry *himax_proc_reset_file 		= NULL;
#endif

#ifdef HX_HIGH_SENSE
#define HIMAX_PROC_HSEN_FILE "HSEN"
static struct proc_dir_entry *himax_proc_HSEN_file = NULL;
#endif

#ifdef HX_TP_PROC_SENSE_ON_OFF
#define HIMAX_PROC_SENSE_ON_OFF_FILE "SenseOnOff"
static struct proc_dir_entry *himax_proc_SENSE_ON_OFF_file = NULL;
#endif

#ifdef HX_SMART_WAKEUP
#define HIMAX_PROC_SMWP_FILE "SMWP"
static struct proc_dir_entry *himax_proc_SMWP_file = NULL;
#define HIMAX_PROC_GESTURE_FILE "GESTURE"
static struct proc_dir_entry *himax_proc_GESTURE_file = NULL;
static uint8_t HX_SMWP_EN = 0;
#endif

#ifdef HX_ESD_RECOVERY
#define HIMAX_PROC_ESD_CNT_FILE "ESD_cnt"
static struct proc_dir_entry *himax_proc_ESD_cnt_file = NULL;
#endif

#endif

#endif
