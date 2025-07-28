#include "key.h"

KEY_MSG_t keymsg = {KEY_NONE, KEY_UP}; // 初始化为无按键状态
static const gpio_pin_enum KEY_PTxn[KEY_NUM] = {MKEY_LIST};

KEY_MSG_t key_msg[KEY_MSG_FIFO_SIZE];
volatile uint8 key_msg_front = 0, key_msg_rear = 0;
volatile uint8 key_msg_flag = KEY_MSG_EMPTY;

void key_init_rewrite(KEY_e key)
{
    if (key < KEY_NUM)
    {
        gpio_init(KEY_PTxn[key], GPI, 0, GPO_PUSH_PULL);
    }
    else
    {
        key = KEY_NUM;
        while (key--)
        {
            gpio_init(KEY_PTxn[key], GPI, 0, GPO_PUSH_PULL);
        }
    }
}

KEY_STATUS_e key_get_status(KEY_e key)
{
    if (gpio_get_level(KEY_PTxn[key]) == KEY_DOWN)
    {
        return KEY_DOWN;
    }
    return KEY_UP;
}

KEY_STATUS_e key_check_status(KEY_e key)
{
    if (key_get_status(key) == KEY_DOWN)
    {
        system_delay_ms(10);
        if (key_get_status(key) == KEY_DOWN)
        {
            return KEY_DOWN;
        }
    }
    return KEY_UP;
}

uint8 key_get_msg(KEY_MSG_t *keymsg)
{
    uint8 tmp;

    if (key_msg_flag == KEY_MSG_EMPTY)
    {
        return 0;
    }

    keymsg->key = key_msg[key_msg_front].key;
    keymsg->status = key_msg[key_msg_front].status;

    key_msg_front++;

    if (key_msg_front >= KEY_MSG_FIFO_SIZE)
    {
        key_msg_front = 0;
    }
    tmp = key_msg_rear;
    if (key_msg_front == tmp)
    {
        key_msg_flag = KEY_MSG_EMPTY;
    }
    else
    {
        key_msg_flag = KEY_MSG_NORMAL;
    }

    return 1;
}

void key_send_msg(KEY_MSG_t keymsg)
{
    uint8 tmp;

    if (key_msg_flag == KEY_MSG_FULL)
    {
        return;
    }
    key_msg[key_msg_rear].key = keymsg.key;
    key_msg[key_msg_rear].status = keymsg.status;

    key_msg_rear++;

    if (key_msg_rear >= KEY_MSG_FIFO_SIZE)
    {
        key_msg_rear = 0;
    }

    tmp = key_msg_rear;
    if (tmp == key_msg_front)
    {
        key_msg_flag = KEY_MSG_FULL;
    }
    else
    {
        key_msg_flag = KEY_MSG_NORMAL;
    }
}

void key_clear_msg(void)
{
    keymsg.key = KEY_NONE;
    keymsg.status = KEY_UP;
}

KEY_e key_scan(void)
{
    for (KEY_e keynum = (KEY_e)0; keynum < KEY_NUM; keynum++)
    {
        if (key_get_status(keynum) == KEY_DOWN)
        {
            system_delay_ms(10); // 消抖
            if (key_get_status(keynum) == KEY_DOWN)
            {
                return keynum;
            }
        }
    }
    return KEY_NONE;
}

void key_IRQHandler()
{
    KEY_e keynum;
    static uint8 keytime[KEY_NUM] = {0};
    KEY_MSG_t keymsg_local;
    uint8 key_pressed = 0; // 标记是否有按键被按下

    for (keynum = (KEY_e)0; keynum < KEY_NUM; keynum++)
    {
        if (key_get_status(keynum) == KEY_DOWN)
        {
            key_pressed = 1; // 有按键被按下
            keytime[keynum]++;
            if (keytime[keynum] <= KEY_DOWN_TIME)
            {
                continue;
            }
            else if (keytime[keynum] <= KEY_HOLD_TIME)
            {
                keymsg_local.key = keynum;
                keymsg_local.status = KEY_DOWN;
                key_send_msg(keymsg_local);
                keymsg = keymsg_local; // 更新全局按键状态
            }
            //  else {
            //     keymsg_local.key = keynum;
            //     keymsg_local.status = KEY_HOLD;
            //     send_key_msg(keymsg_local);
            // }
        }
        else
        {
            if (keytime[keynum] > KEY_DOWN_TIME)
            {
                keymsg_local.key = keynum;
                keymsg_local.status = KEY_UP;
                key_send_msg(keymsg_local);
                keymsg = keymsg_local; // 更新全局按键状态
            }
            keytime[keynum] = 0;
        }
    }

    // 如果没有按键被按下并且当前状态是KEY_UP，将键值设为KEY_NONE
    if (!key_pressed && keymsg.status == KEY_UP)
    {
        keymsg.key = KEY_NONE;
    }
}
