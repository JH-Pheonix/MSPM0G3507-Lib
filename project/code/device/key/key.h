#ifndef _DEVICE_KEY_H_
#define _DEVICE_KEY_H_

#include "zf_common_headfile.h"

#define KEY_DOWN_TIME 10
#define KEY_HOLD_TIME 11

#define KEY_MSG_FIFO_SIZE 20
#define KEY_NUM KEY_NONE // 定义按键数量，替代 KEY_MAX

typedef enum
{
    MKEY_NAME,
    KEY_NONE, // 无按键状态
} KEY_e;

typedef enum
{
    KEY_DOWN = 0,
    KEY_UP = 1,
    KEY_HOLD,
} KEY_STATUS_e;

typedef struct
{
    KEY_e key;
    KEY_STATUS_e status;
} KEY_MSG_t;

typedef enum
{
    KEY_MSG_EMPTY,
    KEY_MSG_NORMAL,
    KEY_MSG_FULL,
} key_msg_e;

void key_init_rewrite(KEY_e key);
KEY_STATUS_e key_get_status(KEY_e key);
KEY_STATUS_e key_check_status(KEY_e key);

uint8 key_get_msg(KEY_MSG_t *keymsg);
void key_send_msg(KEY_MSG_t keymsg);
void key_clear_msg(void); // 新增清除按键消息函数
KEY_e key_scan(void);     // 新增按键扫描函数

void key_IRQHandler();

extern KEY_MSG_t keymsg;

#endif
