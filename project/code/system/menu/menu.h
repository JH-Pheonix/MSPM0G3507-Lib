#ifndef _SYSTEM_MENU_H_
#define _SYSTEM_MENU_H_

#include "zf_common_headfile.h"

#define ALIGN_DIST 13
#define PAGE_DISP_NUM 7  // 菜单显示最多
#define EnumNameLenth 6  // 枚举名称的长度
#define EnumNameNum 10   // 枚举数组长度

enum MenuType {  // 菜单任务类型
    Param_Uint,  // 正整数参数
    Param_Int,   // 有符号参数
    Enumerate,   // 枚举类型
    Sub_Menus,   // 子菜单
    Functions,   // 触发函数功能
    Type_Null,   // 无类型
    Param_Uint16,
};

union Item {                 // 菜单附加参数
    void (*ItemFunc)(void);  // 要运行的菜单函数
    uint8* EnumName;         // 枚举变量名称
    uint8 SubMenuNum;        // 子菜单项数
};

union MenuParam {
    uint32* UINT32;  // 无符号参数地址
    int32* INT32;    // 有符号参数地址
    uint16* UINT16;

    struct MENU_TABLE* SubMenu;  // 子菜单地址
};

struct MENU_TABLE {
    uint8* MenuName;             // 菜单项目名称
    union MenuParam MenuParams;  // 要调试的参数和子菜单 地址
    enum MenuType MenuType;      // 此菜单任务类型
    union Item ItemHook;  // 附加参数 同时运行的函数 枚举变量名称 子菜单项数
};  // 菜单执行

struct MENU_PRMT {
    uint8 ExitMark;  // 退出菜单(0-不退出，1-退出)
    uint8 Cursor;    // 光标值(当前光标位置)
    uint8 PageNo;    // 菜单页(显示开始项)
    uint8 Index;     // 菜单索引(当前选择的菜单项)
    uint8 DispNum;   // 显示项数(每页可以现在菜单项)
    uint8 MaxPage;   // 最大页数(最大有多少种显示页)
};  // 菜单参数

struct Site_t {
    uint8 x;
    uint8 y;
};

// 示例参数变量的外部声明
extern uint32 g_motor_speed;
extern uint32 g_servo_angle;
extern int32 g_sensor_calibration;

void Read_EEPROM(void);
void Write_EEPROM(void);

// void Menu_PrmtInit(MENU_PRMT* prmt, uint8 num, uint8 page);
// void Menu_Process(uint8* menuName,
//                   MENU_PRMT* prmt,
//                   MENU_TABLE* table,
//                   uint8 num);

void MainMenu_Set();
// uint8 Menu_Move();
// KEY_e KeySan(void);
// void SubNameCat(uint8* SubMenuName, uint8* TableMenuName);
// void adjustParam(Site_t site, MENU_TABLE* table, uint16 Color, uint16
// bkColor); void Menu_Display(MENU_TABLE* menuTable,
//                   uint8 pageNo,
//                   uint8 dispNum,
//                   uint8 cursor);
#endif