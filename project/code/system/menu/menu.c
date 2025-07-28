#include "menu.h"
#include "key.h"
#include "lcd.h"

//======================================================================================================================
typedef struct MENU_TABLE MENU_TABLE; // 菜单执行
typedef struct MENU_PRMT MENU_PRMT;   // 菜单参数
typedef enum MenuType MenuType;
typedef union Item Item;
typedef union MenuParam MenuParam;
typedef struct Site_t Site_t;
#define MenuNum(Table) sizeof(Table) / sizeof(Table[0])
#define Debug_Null NULL
void Menu_Null(void) {}

void Menu_Process(uint8 *menuName,
                  MENU_PRMT *prmt,
                  MENU_TABLE *table,
                  uint8 num);
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page);
void write_Flash(uint8 flashNum);
void read_Flash(uint8 flashNum);

MENU_TABLE Table_Null[] = {
    {(uint8 *)"", {.SubMenu = Table_Null}, Type_Null, {.ItemFunc = Menu_Null}},
};
//======================================================================================================================


// 示例参数变量
uint32 g_motor_speed = 100;        // 电机速度
uint32 g_servo_angle = 90;         // 舵机角度
int32 g_sensor_calibration = 0;    // 传感器校准值

uint32 *EEPROM_DATA_UINT[] = {
    &g_motor_speed,
    &g_servo_angle,
};

int32 *EEPROM_DATA_INT[] = {
    &g_sensor_calibration,
};

MENU_TABLE MainMenu_Table[] = {
    {(uint8 *)"System Info", {.SubMenu = Table_Null}, Functions, {.ItemFunc = NULL}},
    {(uint8 *)"Read EEPROM", {.SubMenu = Table_Null}, Functions, {.ItemFunc = Read_EEPROM}},
    {(uint8 *)"Write EEPROM", {.SubMenu = Table_Null}, Functions, {.ItemFunc = Write_EEPROM}},
};

/******************************************************************************
 * FunctionName   : MainMenu_Set()
 * Description    : 常规设置
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void MainMenu_Set()
{
    // ExitMenu_flag = 0;
    lcd_clear();
    MENU_PRMT MainMenu_Prmt;
    uint8 menuNum =
        sizeof(MainMenu_Table) / sizeof(MainMenu_Table[0]); // 菜单项数
    Menu_Process((uint8 *)" -=    Setting   =- ", &MainMenu_Prmt, MainMenu_Table,
                 menuNum);
    Write_EEPROM(); // 将数据写入EEPROM保存
    lcd_clear();
}
//================================================================================================================
// 菜单基本函数
// 不用管这个部分 基本不用改
/******************************************************************************
 * FunctionName   : Menu_PrmtInit()
 * Description    : 初始化菜单参数
 * EntryParameter : prmt - 菜单参数, num - 每页显示项数, page - 最大显示页数
 * ReturnValue    : None
 *******************************************************************************/
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page)
{
    prmt->ExitMark = 0;   // 清除退出菜单标志
    prmt->Cursor = 0;     // 光标清零
    prmt->PageNo = 0;     // 页清零
    prmt->Index = 0;      // 索引清零
    prmt->DispNum = num;  // 页最多显示项目数
    prmt->MaxPage = page; // 最多页数
}
/******************************************************************************
 * FunctionName   : Menu_Move()
 * Description    : 菜单移动
 * EntryParameter : prmt - 菜单参数, key - 按键值
 * ReturnValue    : 有确认返回0，否则返回1
 ******************************************************************************/
uint8 Menu_Move(MENU_PRMT *prmt, KEY_e key)
{
    uint8 rValue = 1;
    switch (key)
    {
    case KEY_U: // 向上
    {
        if (prmt->Cursor != 0) // 光标不在顶端
        {
            prmt->Cursor--; // 光标上移
        }
        else // 光标在顶端
        {
            if (prmt->PageNo != 0) // 页面没有到最小
            {
                prmt->PageNo--; // 向上翻
            }
            else
            {
                prmt->Cursor = prmt->DispNum - 1; // 光标到底
                prmt->PageNo = prmt->MaxPage - 1; // 最后页
            }
        }
        break;
    }

    case KEY_D: // 向下
    {
        if (prmt->Cursor < prmt->DispNum - 1) // 光标没有到底，移动光标
        {
            prmt->Cursor++; // 光标向下移动
        }
        else // 光标到底
        {
            if (prmt->PageNo < prmt->MaxPage - 1) // 页面没有到底，页面移动
            {
                prmt->PageNo++; // 下翻一页
            }
            else // 页面和光标都到底，返回开始页
            {
                prmt->Cursor = 0;
                prmt->PageNo = 0;
            }
        }
        break;
    }
    case KEY_B: // 确认
    {
        prmt->Index = prmt->Cursor + prmt->PageNo; // 计算执行项的索引
        rValue = 0;

        break;
    }
    case KEY_L: // 左键返回上级菜单
    {
        prmt->ExitMark = 1;

        break;
    }
    case KEY_R: // 右键跳到底部
    {
        prmt->Cursor = prmt->DispNum - 1; // 光标到底
        prmt->PageNo = prmt->MaxPage - 1; // 最后页
        break;
    }
    default:
        break;
    }
    return rValue; // 返回执行索引
}
/******************************************************************************
 * FunctionName   : KeySan()
 * Description    : 按键获取
 * EntryParameter : None
 * ReturnValue    : 按键值
 *******************************************************************************/
KEY_e KeySan(void)
{
    KEY_e key_temp = KEY_NONE;
    while (keymsg.status == KEY_UP)
    {
        // 等待按键被按下
        system_delay_ms(10); // 避免过度占用CPU
    }

    key_temp = keymsg.key; // 保存当前按键值
    key_clear_msg();       // 清除按键状态
    return key_temp;       // 返回按键值
}
/******************************************************************************
 * FunctionName   : SubNameCat()
 * Description    : 生成子菜单标题
 * EntryParameter : None
 * ReturnValue    : void
 *******************************************************************************/
void SubNameCat(uint8 *SubMenuName, uint8 *TableMenuName)
{
    const uint8 SubTitlePrefix[] = "-=";
    const uint8 SubTitleSuffix[] = "=-";
    for (uint8 i = 0; i < 20; i++)
    { // initiate the string
        SubMenuName[i] = '\0';
    }
    for (uint8 i = 0; i < strlen((char *)TableMenuName);
         i++)
    { // find the first space and cut the string
        if (TableMenuName[i] == ' ')
        {
            TableMenuName[i] = '\0';
            break;
        }
    }
    if (strlen((char *)TableMenuName) >
        14)
    { // if the name is too long, we need to cut it
        TableMenuName[14] = '\0';
        strcat((char *)SubMenuName, (char *)SubTitlePrefix);
        strcat((char *)SubMenuName, (char *)TableMenuName);
        strcat((char *)SubMenuName, (char *)SubTitleSuffix);
    }
    else
    {
        strcat((char *)SubMenuName, (char *)SubTitlePrefix);
        uint8 spaceNum = (14 - strlen((char *)TableMenuName)) / 2;
        for (uint8 i = 0; i < spaceNum; i++)
        { // fill the space
            strcat((char *)SubMenuName, " ");
        }
        strcat((char *)SubMenuName, (char *)TableMenuName);
        for (uint8 i = 0; i < 14 - spaceNum - strlen((char *)TableMenuName);
             i++)
        { // fill the space
            strcat((char *)SubMenuName, " ");
        }
        strcat((char *)SubMenuName, (char *)SubTitleSuffix);
    }
    // const uint8 SubTitle1[] = " -=";
    // const uint8 SubTitle2[] = "=- ";
    // for (uint8 i = 0;i < 20;i++){
    //     SubMenuName[i] = '\0';
    // }
    // uint8 TableBody[20];
    // uint8 NameLenth = (uint8)strlen((char*)TableMenuName);
    // for (uint8 i = 0;i < NameLenth;i ++){
    //     TableBody[i] = TableMenuName[i];
    //     if(TableMenuName[i] == ' ') {
    //         TableBody[i] = '\0';
    //         NameLenth = i;
    //     }
    // }
    // if (NameLenth > 14) NameLenth = 14;
    // strcat((char*)SubMenuName,(char*)SubTitle1);
    // for (uint8 i = 0;i < (14-NameLenth)/2         ;i ++)
    // strcat((char*)SubMenuName," "); //填充空格
    // strcat((char*)SubMenuName,(char*)TableBody);
    // for (uint8 i = 0;i < 7-NameLenth + NameLenth/2;i ++)
    // strcat((char*)SubMenuName," "); //填充空格
    // strcat((char*)SubMenuName,(char*)SubTitle2);
}
/******************************************************************************
 * FunctionName   : adjustParam()
 * Description    : 调整参数
 * EntryParameter : None
 * ReturnValue    : void
 *******************************************************************************/
void adjustParam(Site_t site, MENU_TABLE *table)
{
    KEY_e key;
    do
    {
        key = KeySan(); // 使用修改后的KeySan函数，会自动清除按键状态
        MenuParam param;
        if (table->MenuType == Param_Uint)
            param.UINT32 = table->MenuParams.UINT32;
        else
            param.INT32 = table->MenuParams.INT32;
        switch (key)
        {
        case KEY_U:
            if (table->MenuType == Param_Uint)
                (*param.UINT32)++;
            else if (table->MenuType == Param_Int)
                (*param.INT32)++;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 < EnumNameNum - 1)
                    (*param.INT32)++;
            }
            break;

        case KEY_D:
            if (table->MenuType == Param_Uint)
                (*param.UINT32)--;
            else if (table->MenuType == Param_Int)
                (*param.INT32)--;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 > 0)
                    (*param.INT32)--;
            }
            break;

        case KEY_L:
            if (table->MenuType == Param_Uint)
                (*param.UINT32) -= 10;
            else if (table->MenuType == Param_Int)
                (*param.INT32) -= 10;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 > 5)
                    (*param.INT32) -= 5;
            }
            break;

        case KEY_R:
            if (table->MenuType == Param_Uint)
                (*param.UINT32) += 10;
            else if (table->MenuType == Param_Int)
                (*param.INT32) += 10;
            else if (table->MenuType == Enumerate)
            {
                if (*param.INT32 < EnumNameNum - 5)
                    (*param.INT32) += 5;
            }
            break;

        default:
            break;
        }
        if (table->MenuType == Param_Uint)
        {
            lcd_show_uint_color(site.x, site.y, *param.UINT32, 6,
                                DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
            if (table->ItemHook.ItemFunc != Menu_Null)
                table->ItemHook.ItemFunc();
        }
        else if (table->MenuType == Param_Int)
        {
            lcd_show_int_color(site.x, site.y, *param.INT32, 6,
                               DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
            if (table->ItemHook.ItemFunc != Menu_Null)
                table->ItemHook.ItemFunc();
        }
        else if (table->MenuType == Enumerate)
        {
            lcd_show_string(site.x, site.y,
                            (const char *)(table->ItemHook.EnumName +
                                     (*param.INT32) * (EnumNameLenth + 1)));
        }
    } while (key != KEY_B);
}

/******************************************************************************
 * FunctionName   : Menu_Display()
 * Description    : 显示菜单项
 * EntryParameter : page - 显示页，dispNum - 每一页的显示项，cursor - 光标位置
 * ReturnValue    : None
 *******************************************************************************/
void Menu_Display(MENU_TABLE *menuTable,
                  uint8 pageNo,
                  uint8 dispNum,
                  uint8 cursor)
{
    uint8 i;
    Site_t site;
    for (i = 0; i < dispNum; i++)
    {
        site.x = 0;
        site.y = i + 1;
        if (cursor == i)
            /* 反白显示当前光标选中菜单项 */
            lcd_show_string_color((uint16)site.x, (uint16)site.y,
                                  (const char *)menuTable[pageNo + i].MenuName,
                                  DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
        else
            /* 正常显示其余菜单项 */
            lcd_show_string_color((uint16)site.x, (uint16)site.y,
                                  (const char *)menuTable[pageNo + i].MenuName,
                                  DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
        /* 若此菜单项有需要调的参数，则显示该参数 */
        if (menuTable[pageNo + i].MenuType == Param_Uint ||
            menuTable[pageNo + i].MenuType == Param_Int ||
            menuTable[pageNo + i].MenuType == Enumerate)
        {
            site.x = ALIGN_DIST;
            if (menuTable[pageNo + i].MenuType == Param_Uint)
            {
                uint32 num_t = (*(menuTable[pageNo + i].MenuParams.UINT32));
                lcd_show_uint(site.x, site.y, num_t, 6);
            }
            else if (menuTable[pageNo + i].MenuType == Param_Int)
            {
                int32 num_t = (*(menuTable[pageNo + i].MenuParams.INT32));
                lcd_show_int(site.x, site.y, num_t, 6);
            }
            else if (menuTable[pageNo + i].MenuType == Enumerate)
            {
                uint32 num_t = (*(menuTable[pageNo + i].MenuParams.UINT32));
                lcd_show_string(
                    site.x, site.y,
                    (const char *)(menuTable[pageNo + i].ItemHook.EnumName +
                             num_t * (EnumNameLenth + 1)));
            }
        }
    }
}

/******************************************************************************
 * FunctionName   : Menu_Process()
 * Description    : 处理菜单项
 * EntryParameter : menuName - 菜单名称，prmt - 菜单参数，table - 菜单表项, num
 *- 菜单项数 ReturnValue    : None Describe
 *: 1.进入子菜单 2.调节参数 3.调节参数并执行 4.执行函数
 ******************************************************************************/
void Menu_Process(uint8 *menuName,
                  MENU_PRMT *prmt,
                  MENU_TABLE *table,
                  uint8 num)
{
    KEY_e key;
    Site_t site;
    uint8 page; // 显示菜单需要的页数
    if (num - PAGE_DISP_NUM <= 0)
        page = 1;
    else
    {
        page = num - PAGE_DISP_NUM + 1;
        num = PAGE_DISP_NUM;
    }
    // 显示项数和页数设置
    Menu_PrmtInit(prmt, num, page);
    do
    {
        lcd_clear();
        lcd_show_string(0, 0, (const char *)menuName); // 显示菜单标题
        // 显示菜单项
        Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);
        key = KeySan(); // 获取按键

        if (Menu_Move(prmt, key) == 0) // 菜单移动 按下确认键
        {
            // 判断此菜单项有无需要调节的参数 有则进入参数调节
            // 在参数调节里看有无函数同时运行
            if (table[prmt->Index].MenuType == Param_Uint ||
                table[prmt->Index].MenuType == Param_Int ||
                table[prmt->Index].MenuType == Enumerate)
            {
                site.x = ALIGN_DIST;
                site.y = 1 + prmt->Cursor;
                if (table[prmt->Index].MenuType == Param_Uint)
                    lcd_show_uint_color(
                        site.x, site.y, *(table[prmt->Index].MenuParams.UINT32),
                        6, DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
                else if (table[prmt->Index].MenuType == Param_Int)
                    lcd_show_int_color(
                        site.x, site.y, *(table[prmt->Index].MenuParams.INT32),
                        6, DEFAULT_BACKGROUND_COLOR, DEFAULT_PEN_COLOR);
                else if (table[prmt->Index].MenuType == Enumerate)
                    lcd_show_string(
                        site.x, site.y,
                        (const char *)(table[prmt->Index].ItemHook.EnumName +
                                 (*(table[prmt->Index].MenuParams.INT32)) *
                                     EnumNameLenth));
                // 在参数调节里看有无函数同时运行  可以同时执行
                // 方便舵机调试，电机调试 这个在上面的调节参数函数里已经执行过
                adjustParam(site, &table[prmt->Index]);
            }
            // 不是参数调节的话就执行菜单函数
            else if (table[prmt->Index].MenuType == Functions)
            {
                table[prmt->Index].ItemHook.ItemFunc(); // 执行相应项
            }
            // 没有参数调节和函数执行的话 就是子菜单
            else if (table[prmt->Index].MenuType == Sub_Menus)
            {
                // 确定有子菜单
                //                if (table[prmt->Index].MenuParams.SubMenu !=
                //                Table_Null){
                lcd_clear();
                MENU_PRMT Submenu_Prmt;
                uint8 SubMenuName[20];
                SubNameCat(SubMenuName, table[prmt->Index].MenuName);
                Menu_Process(SubMenuName, &Submenu_Prmt,
                             table[prmt->Index].MenuParams.SubMenu,
                             table[prmt->Index].ItemHook.SubMenuNum);
                //                }
            }
        }
        // } while (prmt->ExitMark == 0 && ExitMenu_flag == 0);
    } while (prmt->ExitMark == 0);
    lcd_clear();
}

void write_Flash(uint8 flashNum)
{
    /* 一共有96KB 96KB分为了12页 每页可以存1024个uint32类型的数据 仅使用第0页 */
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_erase_page(0, (uint32)flashNum);

    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
        flash_union_buffer[i].uint32_type = (uint32)*EEPROM_DATA_UINT[i];
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
        flash_union_buffer[i + Flash_Save_intNum].int32_type =
            (int32)*EEPROM_DATA_INT[i];
    flash_write_page_from_buffer(0, (uint32)flashNum);

    flash_buffer_clear(); // 清除缓存
    lcd_show_string(0, 0, "WRITE IS OK!");
}

void read_Flash(uint8 flashNum)
{
    /* 每页可以存1024个uint32类型的数据 */
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_buffer_clear();                   // 清除缓存
    flash_read_page_to_buffer(0, flashNum); // 将数据从缓存区读出来
    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
    {
        uint32 temp_vaule = flash_union_buffer[i].uint32_type;
        *EEPROM_DATA_UINT[i] = temp_vaule;
    }
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
    {
        int32 temp_vaule =
            flash_union_buffer[Flash_Save_uintNum + i].int32_type;
        *EEPROM_DATA_INT[i] = temp_vaule;
    }
    flash_buffer_clear(); // 清除缓存
    lcd_show_string(0, 0, "READ IS OK!");
}

void Read_EEPROM()
{
    lcd_clear();
    lcd_show_string(0, 0, "READ EEPROM");
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_buffer_clear();             // 清除缓存
    flash_read_page_to_buffer(0, 11); // 将数据从缓存区读出来
    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
    {
        uint32 temp_vaule = flash_union_buffer[i].uint32_type;
        *EEPROM_DATA_UINT[i] = temp_vaule;
    }
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
    {
        int32 temp_vaule =
            flash_union_buffer[Flash_Save_uintNum + i].int32_type;
        *EEPROM_DATA_INT[i] = temp_vaule;
    }
    flash_buffer_clear(); // 清除缓存
    lcd_clear();
    lcd_show_string(0, 0, "READ SUCCESS");
}

void Write_EEPROM()
{
    lcd_clear();
    lcd_show_string(0, 0, "WRITE EEPROM");
    const uint16 Flash_Save_uintNum =
        sizeof(EEPROM_DATA_UINT) / sizeof(EEPROM_DATA_UINT[0]);
    const uint16 Flash_Save_intNum =
        sizeof(EEPROM_DATA_INT) / sizeof(EEPROM_DATA_INT[0]);
    flash_erase_page(0, 11);
    flash_buffer_clear(); // 清除缓存
    for (uint16 i = 0; i < Flash_Save_uintNum; i++)
        flash_union_buffer[i].uint32_type = (uint32)*EEPROM_DATA_UINT[i];
    for (uint16 i = 0; i < Flash_Save_intNum; i++)
        flash_union_buffer[i + Flash_Save_uintNum].int32_type =
            (int32)*EEPROM_DATA_INT[i];
    flash_write_page_from_buffer(0, 11);
    flash_buffer_clear(); // 清除缓存
    lcd_clear();
    lcd_show_string(0, 0, "WRITE SUCCESS");
}

// 示例函数实现
/******************************************************************************
 * FunctionName   : Motor_Test()
 * Description    : 电机测试函数
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void Motor_Test(void)
{
    // 这里可以添加电机控制代码
    // 例如：设置PWM输出，控制电机转速
    // pwm_set_duty(PWM_CH1, g_motor_speed);
}

/******************************************************************************
 * FunctionName   : Servo_Test()
 * Description    : 舵机测试函数
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void Servo_Test(void)
{
    // 这里可以添加舵机控制代码
    // 例如：设置舵机角度
    // servo_set_angle(SERVO_CH1, g_servo_angle);
}

/******************************************************************************
 * FunctionName   : PID_Debug()
 * Description    : PID调试函数
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void PID_Debug(void)
{
    // 这里可以添加PID参数更新代码
    // 例如：更新PID控制器参数
    // pid_update_params(g_pid_kp, g_pid_ki, g_pid_kd);
}

/******************************************************************************
 * FunctionName   : Sensor_Calibrate()
 * Description    : 传感器校准函数
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void Sensor_Calibrate(void)
{
    // 这里可以添加传感器校准代码
    // 例如：应用校准值
    // sensor_apply_calibration(g_sensor_calibration);
}

/******************************************************************************
 * FunctionName   : System_Info()
 * Description    : 显示系统信息
 * EntryParameter : None
 * ReturnValue    : None
 *******************************************************************************/
void System_Info(void)
{
    lcd_clear();
    lcd_show_string(0, 0, "=== System Info ===");
    lcd_show_string(0, 1, "Version: v1.0.0");
    lcd_show_string(0, 2, "MCU: MSPM0G3507");
    lcd_show_string(0, 3, "Flash: 512KB");
    lcd_show_string(0, 4, "RAM: 64KB");
    lcd_show_string(0, 5, "Press any key...");
    
    // 等待按键返回
    KeySan();
}
