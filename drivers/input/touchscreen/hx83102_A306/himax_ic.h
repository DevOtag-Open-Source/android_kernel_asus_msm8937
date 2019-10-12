/* Himax Android Driver Sample Code for HX83102 chipset
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

#include "himax_platform.h"
#include "himax_common.h"
#include <linux/slab.h>

#define HIMAX_REG_RETRY_TIMES 5

#ifdef HX_ESD_RECOVERY
extern u8 HX83102_ESD_RESET_ACTIVATE;
#endif

enum fw_image_type
{
    fw_image_32k	= 0x01,
    fw_image_48k,
    fw_image_60k,
    fw_image_64k,
    fw_image_124k,
    fw_image_128k,
};

int hx83102_hand_shaking(struct i2c_client *client);
void hx83102_set_SMWP_enable(struct i2c_client *client,uint8_t SMWP_enable, bool suspended);
void hx83102_set_HSEN_enable(struct i2c_client *client,uint8_t HSEN_enable, bool suspended);
void hx83102_usb_detect_set(struct i2c_client *client,uint8_t *cable_config);
int hx83102_determin_diag_rawdata(int diag_command);
int hx83102_determin_diag_storage(int diag_command);
void hx83102_diag_register_set(struct i2c_client *client, uint8_t diag_command);
void hx83102_flash_dump_func(struct i2c_client *client, uint8_t local_flash_command, int Flash_Size, uint8_t *flash_buffer);
int hx83102_chip_self_test(struct i2c_client *client);
void hx83102_burst_enable(struct i2c_client *client, uint8_t auto_add_4_byte);
int hx83102_register_read(struct i2c_client *client, uint8_t *read_addr, int read_length, uint8_t *read_data, bool hx83102_cfg_flag);
void hx83102_register_write(struct i2c_client *client, uint8_t *write_addr, int write_length, uint8_t *write_data, bool hx83102_cfg_flag);
bool hx83102_sense_off(struct i2c_client *client);
void hx83102_sense_on(struct i2c_client *client, uint8_t FlashMode);
int hx83102_fts_ctpm_fw_upgrade_with_sys_fs_32k(struct i2c_client *client, unsigned char *fw, int len, bool change_iref);
int hx83102_fts_ctpm_fw_upgrade_with_sys_fs_60k(struct i2c_client *client, unsigned char *fw, int len, bool change_iref);
int hx83102_fts_ctpm_fw_upgrade_with_sys_fs_64k(struct i2c_client *client, unsigned char *fw, int len, bool change_iref);
int hx83102_fts_ctpm_fw_upgrade_with_sys_fs_124k(struct i2c_client *client, unsigned char *fw, int len, bool change_iref);
int hx83102_fts_ctpm_fw_upgrade_with_sys_fs_128k(struct i2c_client *client, unsigned char *fw, int len, bool change_iref);
void hx83102_touch_information(struct i2c_client *client);
int  hx83102_read_i2c_status(struct i2c_client *client);
int  hx83102_read_ic_trigger_type(struct i2c_client *client);
void hx83102_read_FW_ver(struct i2c_client *client);
bool hx83102_ic_package_check(struct i2c_client *client);
void hx83102_power_on_init(struct i2c_client *client);
bool hx83102_read_event_stack(struct i2c_client *client, uint8_t *buf, uint8_t length);
void hx83102_get_DSRAM_data(struct i2c_client *client, uint8_t *info_data);
bool hx83102_calculateChecksum(struct i2c_client *client, bool change_iref);
bool hx83102_flash_lastdata_check(struct i2c_client *client);
uint8_t hx83102_read_DD_status(uint8_t *hx83102_cmd_set, uint8_t *tmp_data);
int hx83102_read_FW_status(uint8_t *state_addr, uint8_t *tmp_addr);
void hx83102_resume_ic_action(struct i2c_client *client);
void hx83102_suspend_ic_action(struct i2c_client *client);

//ts_work
int hx83102_cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max);
bool hx83102_diag_check_sum(struct hx_83102_report_data *hx83102_touch_data); //return checksum value
void hx83102_diag_parse_raw_data(struct hx_83102_report_data *hx83102_touch_data,int mul_num, int self_num,uint8_t diag_cmd, int32_t *mutual_data, int32_t *self_data);
