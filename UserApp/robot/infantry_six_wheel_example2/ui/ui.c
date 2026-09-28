#include "ui.h"
#include "cmsis_os.h"
#include "referee_ui.h"
#include "referee.h"
#include "robot.h"


static referee_info_t *referee_recv_info;  // 接收到的裁判系统数据
RobotInstance *robotdata;
uint8_t UI_Seq;  // 包序号，供整个referee文件使用

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID() {
  // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;  // 计算客户端ID
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}


static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_drone_width_line[2]; //车辆示宽线
static Graph_Data_t UI_shoot_dir_circle[3]; //射击中心圆
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change

// 定义用于UI更新的数据结构

// 云台俯仰角表盘
static Graph_Data_t UI_pitch_ticks[20];    // 刻度线
static String_Data_t UI_pitch_labels[20];  // 刻度标签
static Graph_Data_t UI_pitch_needle;       // 指针

// 自瞄模式选择器
static Graph_Data_t UI_autoaim_bg;         // 背景圆弧
static Graph_Data_t UI_autoaim_indicator;  // 选择指示圆弧
static String_Data_t UI_autoaim_text;      // 模式文字 "A  B  C"

// 左侧功率/弹量圆弧
static Graph_Data_t UI_cap_arc;          // 电容能量圆弧
static Graph_Data_t UI_ammo_arc;         // 弹量圆弧
static String_Data_t UI_cap_text_E;      // 文字 "E"
static String_Data_t UI_cap_text_F;      // 文字 "F"
static String_Data_t UI_ammo_text_full;  // 文字 "500"
static String_Data_t UI_ammo_text_mid;   // 文字 "250"

// 摩擦轮转速指示器
static String_Data_t UI_fric_text_down;     // 文字 "3"
// 车头方向动态圆弧
static Graph_Data_t UI_yaw_arc;  // 方向指示弧

static Graph_Data_t Line_DecoMid;
static Graph_Data_t Arc_DecoUp;
static Graph_Data_t Arc_DecoDown;

static Referee_Interactive_info_t interactive_data;

// 检测UI变化的函数
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data) {
  if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode) {
    _Interactive_data->UI_Interactive_Flag.gimbal_flag = 1;
    _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
  }

  if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode) {
    _Interactive_data->UI_Interactive_Flag.shoot_flag = 1;
    _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
  }

  if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode) {
    _Interactive_data->UI_Interactive_Flag.fric_flag = 1;
    _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
  }

  if (_Interactive_data->Chassis_Power_Data.chassis_power_mx !=
      _Interactive_data->Chassis_last_Power_Data.chassis_power_mx) {
    _Interactive_data->UI_Interactive_Flag.power_flag = 1;
    _Interactive_data->Chassis_last_Power_Data.chassis_power_mx =
        _Interactive_data->Chassis_Power_Data.chassis_power_mx;
  }

  // 俯仰角变化
  if (fabsf(_Interactive_data->pitch_angle - _Interactive_data->last_pitch_angle) > 0.1f) {
    _Interactive_data->UI_Interactive_Flag.pitch_flag = 1;
    _Interactive_data->last_pitch_angle = _Interactive_data->pitch_angle;
  }

  if (_Interactive_data->autoaim_mode != _Interactive_data->last_autoaim_mode) {
    _Interactive_data->UI_Interactive_Flag.autoaim_flag = 1;
    _Interactive_data->last_autoaim_mode = _Interactive_data->autoaim_mode;
  }

  if (fabsf(_Interactive_data->cap_voltage - _Interactive_data->last_cap_voltage) > 0.1f
      ) {
    _Interactive_data->UI_Interactive_Flag.cap_flag = 1;
    _Interactive_data->last_cap_voltage = _Interactive_data->cap_voltage;

  }
  if (_Interactive_data->bullet_left_real != _Interactive_data->last_bullet_left_real) {
    _Interactive_data->UI_Interactive_Flag.ammo_flag = 1;
    _Interactive_data->last_bullet_left_real = _Interactive_data->bullet_left_real;
  }

  if (_Interactive_data->fric_speed_left != _Interactive_data->last_fric_speed_left ||
    _Interactive_data->fric_speed_right != _Interactive_data->last_fric_speed_right) {
    _Interactive_data->UI_Interactive_Flag.fric_flag = 1;
    _Interactive_data->last_fric_speed_left = _Interactive_data->fric_speed_left;
    _Interactive_data->last_fric_speed_right = _Interactive_data->fric_speed_right;
    }

  if (fabsf(_Interactive_data->chassis_relative_angle - _Interactive_data->last_chassis_relative_angle) > 0.01f) {
    _Interactive_data->UI_Interactive_Flag.yaw_flag = 1;
    _Interactive_data->last_chassis_relative_angle = _Interactive_data->chassis_relative_angle;
  }
}

// UI更新函数
static void MyUIRefresh(Referee_Interactive_info_t *interactive_data) {
  // 更新云台状态

    char *gimbal_str = "unknown";
    switch (interactive_data->heat_mode_e) {
      case NO_CONTROL:
        gimbal_str = "no";
        break;
      case REFEREE_CONTROL:
        gimbal_str = "referee      ";
        break;
      case SIMULLATE_CONTROL:
        gimbal_str = "simulate  ";
        break;
      default:

        break;
    }
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, gimbal_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    interactive_data->UI_Interactive_Flag.gimbal_flag = 0;


  // 更新射击状态

    char *bullet_str = "unknown";
    switch (interactive_data->bullet_speed_mode_e) {
      case NO_CONTROL:
        bullet_str = "no";
        break;
      case MANUAL_BULLET_SPEED:
        bullet_str = "man      ";
        break;
      case ENABLE_BULLET_SPEED:
        bullet_str = "en  ";
        break;
      default:

        break;
    }
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, bullet_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    interactive_data->UI_Interactive_Flag.shoot_flag = 0;


  // 更新摩擦轮状态
  if (interactive_data->UI_Interactive_Flag.fric_flag == 1) {
    char *friction_str = interactive_data->friction_mode == FRICTION_ON ? "on " : "off";
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, friction_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    interactive_data->UI_Interactive_Flag.fric_flag = 0;
  }


    char *cap_str = interactive_data->cap_msg.error_detect == 0? "good " : "bad";
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, cap_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);


  // 自瞄模式指示器
  if (interactive_data->UI_Interactive_Flag.autoaim_flag == 1) {
    uint32_t start_angle, end_angle;
    switch (interactive_data->autoaim_mode) {
      case 0:
        start_angle = 168;
        end_angle = 180;
        break;  // 左段
      case 1:
        start_angle = 180;
        end_angle = 192;
        break;  // 右段
      default:
        start_angle = 168;
        end_angle = 192;
        break;  // 全段
    }
    UIArcDraw(&UI_autoaim_indicator, "ac0", UI_Graph_Change, 6, UI_Color_Pink, start_angle, end_angle, 14, 960, 540,
              310, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_indicator);
    interactive_data->UI_Interactive_Flag.autoaim_flag = 0;
  }

  // 电容能量圆弧
  if (interactive_data->UI_Interactive_Flag.cap_flag == 1) {
    float volt = interactive_data->cap_voltage;
    float bar = (volt - 12.0f) / (23.0f - 12.0f) * 40.0f;
    if (bar < 1) bar = 1;
    if (bar > 40) bar = 40;
    uint32_t end_angle = 270 + (uint32_t)bar;
    uint32_t color = UI_Color_Pink;
    UIArcDraw(&UI_cap_arc, "ap0", UI_Graph_Change, 6, color, 270, end_angle, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_cap_arc);
    interactive_data->UI_Interactive_Flag.cap_flag = 0;
  }

  // 弹量圆弧
  if (interactive_data->UI_Interactive_Flag.ammo_flag == 1) {
    uint16_t bullet = interactive_data->bullet_left_real;
    if (bullet > 500) bullet = 500;
    float bar = (float)bullet / 500.0f * 40.0f;
    uint32_t start_angle = 270 - (uint32_t)bar;
    uint32_t color;
    if (bullet > 80)
      color = UI_Color_Green;
    else if (bullet > 30)
      color = UI_Color_Yellow;
    else if (bullet > 0)
      color = UI_Color_Orange;
    else
      color = UI_Color_Black;
    UIArcDraw(&UI_ammo_arc, "aa0", UI_Graph_Change, 6, color, start_angle, 270, 22, 960, 535, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_ammo_arc);
    interactive_data->UI_Interactive_Flag.ammo_flag = 0;
  }

  // 显示左侧摩擦轮转速（精确到 100）
  if (interactive_data->UI_Interactive_Flag.fric_flag == 1) {
    uint16_t left_speed = interactive_data->fric_speed_left;
    uint16_t display_speed = left_speed / 100;  // 26000 -> 260
    char fric_speed_str[16];
    sprintf(fric_speed_str, "frispeed:%d", display_speed);
    UICharDraw(&UI_fric_text_down, "fs0", UI_Graph_Change, 6, UI_Color_White, 18, 2, 1556, 850, fric_speed_str);
    UICharRefresh(&referee_recv_info->referee_id, UI_fric_text_down);
    interactive_data->UI_Interactive_Flag.fric_flag = 0;
  }

  // 车头方向圆弧
  if (interactive_data->UI_Interactive_Flag.yaw_flag == 1) {
    int yaw_deg = (int)interactive_data->chassis_relative_angle;
    if (yaw_deg < 0) yaw_deg += 360;
    uint32_t show1, show2;
    if (yaw_deg < 30) {
      show1 = yaw_deg + 30;
      show2 = 360 - (30 - yaw_deg);
    } else if (yaw_deg > 345) {
      show1 = yaw_deg - 330;
      show2 = yaw_deg - 30;
    } else {
      show1 = yaw_deg + 30;
      show2 = yaw_deg - 30;
    }
    uint32_t color;
    switch (interactive_data->robot_mode) {
      case ROBOT_CHASSIS_PROSTRATE_FOLLOW:
        color = UI_Color_Pink;
        break;
      case ROBOT_CHASSIS_PROSTRATE_ROTATE:
        color = UI_Color_Green;
        break;
      default:
        color = UI_Color_Orange;
        break;
    }
    UIArcDraw(&UI_yaw_arc, "yd0", UI_Graph_Change, 6, color, show1, show2, 23, 1556, 721, 88, 88);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_yaw_arc);
    interactive_data->UI_Interactive_Flag.yaw_flag = 0;
  }

  // 俯仰角仪表盘指针和数值
  if (interactive_data->UI_Interactive_Flag.pitch_flag == 1) {
    float pitch_deg = interactive_data->pitch_angle;
    float ui_angle = 90.0f + pitch_deg;  // 转换为UI坐标系角度

    while (ui_angle < 0) ui_angle += 360.0f;
    while (ui_angle >= 360) ui_angle -= 360.0f;

    uint32_t start_angle = (uint32_t)(ui_angle - 1);
    uint32_t end_angle = (uint32_t)(ui_angle + 1);

    if (start_angle >= 360) start_angle -= 360;
    if (end_angle >= 360) end_angle -= 360;

    UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_Change, 6, UI_Color_Pink, start_angle, end_angle, 38, 960, 540, 365,
              365);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);
    interactive_data->UI_Interactive_Flag.pitch_flag = 0;
  }
}

/**
 * @brief 初始化俯仰角仪表盘UI组件
 * @param[in] center_x 仪表盘圆心X坐标
 * @param[in] center_y 仪表盘圆心Y坐标
 * @param[in] radius 仪表盘半径
 * @return void
 */
void UIPitchGaugeInit(uint32_t center_x, uint32_t center_y, uint32_t radius) {
  const int main_ticks[] = {50, 70, 90, 110, 130};
  const char *main_labels[] = {"50", "70", "90", "110", "130"};
  const int sub_ticks[] = {60, 80, 100, 120};

  for (int i = 0; i < 5; i++) {
    float ui_angle = 90.0f - (float)main_ticks[i];
    float angle_rad = ui_angle * 3.1415926535f / 180.0f;
    uint32_t tick_length = 27;

    int32_t start_x = center_x + (int32_t)(radius * cosf(angle_rad));
    int32_t start_y = center_y - (int32_t)(radius * sinf(angle_rad));
    int32_t end_x = center_x + (int32_t)((radius - tick_length) * cosf(angle_rad));
    int32_t end_y = center_y - (int32_t)((radius - tick_length) * sinf(angle_rad));

    char graph_name[3] = {0};
    graph_name[0] = 'p';
    graph_name[1] = 't';
    graph_name[2] = '0' + i;

    UILineDraw(&UI_pitch_ticks[i], graph_name, UI_Graph_ADD, 8, UI_Color_White, 7, start_x, start_y, end_x, end_y);

    int32_t label_x, label_y;
    int32_t label_distance = radius - tick_length - 25;
    int32_t base_x = center_x + (int32_t)(label_distance * cosf(angle_rad));
    int32_t base_y = center_y - (int32_t)(label_distance * sinf(angle_rad));
    int32_t text_width = 18;
    int32_t text_height = 12;
    float cos_val = cosf(angle_rad);
    float sin_val = sinf(angle_rad);

    if (cos_val > 0.7f) {
      label_x = base_x - text_width;
    } else if (cos_val < -0.7f) {
      label_x = base_x;
    } else if (cos_val > 0.3f) {
      label_x = base_x - text_width / 2;
    } else if (cos_val < -0.3f) {
      label_x = base_x - text_width / 2;
    } else {
      label_x = base_x - text_width / 2;
    }

    if (sin_val > 0.7f) {
      label_y = base_y;
    } else if (sin_val < -0.7f) {
      label_y = base_y - text_height;
    } else if (sin_val > 0.3f) {
      label_y = base_y - text_height / 2;
    } else if (sin_val < -0.3f) {
      label_y = base_y - text_height / 2;
    } else {
      label_y = base_y - text_height / 2;
    }

    switch (main_ticks[i]) {
      case 50:
        label_x -= 2;
        label_y += 4;
        break;
      case 70:
        label_x -= 1;
        label_y += 2;
        break;
      case 90:
        break;
      case 110:
        label_x -= 1;
        label_y -= 2;
        break;
      case 130:
        label_x -= 2;
        label_y -= 4;
        break;
    }

    char label_name[3] = {0};
    label_name[0] = 'p';
    label_name[1] = 'l';
    label_name[2] = '0' + i;

    UICharDraw(&UI_pitch_labels[i], label_name, UI_Graph_ADD, 8, UI_Color_White, 12, 2, label_x, label_y,
               main_labels[i]);
  }

  for (int i = 0; i < 4; i++) {
    float ui_angle = 90.0f - (float)sub_ticks[i];
    float angle_rad = ui_angle * 3.1415926535f / 180.0f;
    uint32_t tick_length = 13;

    int32_t start_x = center_x + (int32_t)(radius * cosf(angle_rad));
    int32_t start_y = center_y - (int32_t)(radius * sinf(angle_rad));
    int32_t end_x = center_x + (int32_t)((radius - tick_length) * cosf(angle_rad));
    int32_t end_y = center_y - (int32_t)((radius - tick_length) * sinf(angle_rad));

    char graph_name[3] = {0};
    graph_name[0] = 'p';
    graph_name[1] = 's';
    graph_name[2] = '0' + i;

    static Graph_Data_t UI_sub_ticks[4];
    UILineDraw(&UI_sub_ticks[i], graph_name, UI_Graph_ADD, 8, UI_Color_White, 7, start_x, start_y, end_x, end_y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_sub_ticks[i]);
  }

  for (int i = 0; i < 5; i++) {
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_ticks[i]);
    UICharRefresh(&referee_recv_info->referee_id, UI_pitch_labels[i]);
  }
}

void MyUIInit(RobotInstance *robot) {
  referee_recv_info = robot->referee_data;

  while (referee_recv_info->GameRobotState.robot_id == 0) osDelay(100);
  DeterminRobotID();
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
   // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 2, CENTER_X - 300, CENTER_Y, CENTER_X + 300, CENTER_Y);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 50, CENTER_Y + Aim_Line_1, CENTER_X + 50, CENTER_Y + Aim_Line_1);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 80, CENTER_Y + Aim_Line_2, CENTER_X + 80, CENTER_Y + Aim_Line_2);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 120, CENTER_Y + Aim_Line_3, CENTER_X + 120, CENTER_Y + Aim_Line_3);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X - 200, CENTER_Y + Aim_Line_4, CENTER_X + 200, CENTER_Y + Aim_Line_4);
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_White, 1, CENTER_X, 300, CENTER_X, 650);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_line[5]);

    // 绘制车辆示宽线
    UILineDraw(&UI_drone_width_line[0], "sl6", UI_Graph_ADD, 7, UI_Color_Green, 2, 960 - WIDTHLINE_UP, 320, 960 - WIDTHLINE_DOWN, 0);
    UILineDraw(&UI_drone_width_line[1], "sl7", UI_Graph_ADD, 7, UI_Color_Green, 2, 960 + WIDTHLINE_DOWN, 0, 960 + WIDTHLINE_UP, 320);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_drone_width_line[0], UI_drone_width_line[1]);

    // 绘制发射中心圆
    UICircleDraw(&UI_shoot_dir_circle[0], "sc0", UI_Graph_ADD, 7, UI_Color_White, 3, 960, 540, 15);
    //绘制车头方向指示的中心圆
    UICircleDraw(&UI_shoot_dir_circle[1], "sc1", UI_Graph_ADD, 7, UI_Color_White, 3, 1556, 721, 76);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_dir_circle[0],  UI_shoot_dir_circle[1]);

    // 绘制车辆状态标志指示
    // UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 750, "chassis:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 700, "heat:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 650, "bullet:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 600, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 150, 550, "supercap:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);


    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    // UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 750, "PowerOff");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 700, "PowerOff");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 650, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 600, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 270, 550, "close ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    //绘制枪口热度，射频指示
    // UICharDraw(&UI_Shoot_sta[0], "ss5", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1500, 750, "SHOOT_HEAT:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_sta[0]);
    // UICharDraw(&UI_Shoot_sta[1], "ss6", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1500, 700, "SHOOT_RATE:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_sta[1]);

    //绘制枪口热度，射频指示，动态
    // UICharDraw(&UI_Shoot_dyn[0], "sd7", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1700, 750, "0");
    // UIFloatDraw(&UI_Shoot_dyn[0], "sd7", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 2, 1700, 750, 0);
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_dyn[0]);
    // UICharDraw(&UI_Shoot_dyn[1], "sd8", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 1700, 700, "0");
    // UIFloatDraw(&UI_Shoot_dyn[1], "sd8", UI_Graph_ADD, 8, UI_Color_White, 15, 2, 2, 1700, 700, 0);
    // UICharRefresh(&referee_recv_info->referee_id, UI_Shoot_dyn[1]);

    // 底盘功率显示，静态
    // UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // 能量条框
    // UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    // UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // 能量条初始状态
    // UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    // UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);

    UIPitchGaugeInit(960, 540, 390);

    // 自瞄模式选择器 (假设有3个模式，显示 "A  B  C")
    UIArcDraw(&UI_autoaim_bg, "ab0", UI_Graph_ADD, 5, UI_Color_White, 168, 192, 14, 960, 540, 310, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_bg);
    UIArcDraw(&UI_autoaim_indicator, "ac0", UI_Graph_ADD, 6, UI_Color_Pink, 168, 180, 14, 960, 540, 310, 240);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_autoaim_indicator);
    UICharDraw(&UI_autoaim_text, "at0", UI_Graph_ADD, 5, UI_Color_White, 18, 3, 900, 280, "A  B  C");
    UICharRefresh(&referee_recv_info->referee_id, UI_autoaim_text);

    // 左侧功率/弹量圆弧
    UIArcDraw(&UI_cap_arc, "ap0", UI_Graph_ADD, 6, UI_Color_Pink, 270, 310, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_cap_arc);
    // UIArcDraw(&UI_ammo_arc, "aa0", UI_Graph_ADD, 6, UI_Color_Main, 230, 270, 22, 960, 535, 370, 370);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_ammo_arc);
    UICharDraw(&UI_cap_text_E, "ce0", UI_Graph_ADD, 6, UI_Color_White, 17, 2, 610, 545, "E");
    UICharRefresh(&referee_recv_info->referee_id, UI_cap_text_E);
    UICharDraw(&UI_cap_text_F, "cf0", UI_Graph_ADD, 6, UI_Color_White, 17, 2, 702, 775, "F");
    UICharRefresh(&referee_recv_info->referee_id, UI_cap_text_F);
    UICharDraw(&UI_ammo_text_full, "af0", UI_Graph_ADD, 6, UI_Color_White, 12, 2, 697, 300, "500");
    UICharRefresh(&referee_recv_info->referee_id, UI_ammo_text_full);
    UICharDraw(&UI_ammo_text_mid, "am0", UI_Graph_ADD, 6, UI_Color_White, 12, 2, 630, 420, "250");
    UICharRefresh(&referee_recv_info->referee_id, UI_ammo_text_mid);

    // 摩擦轮转速指示器
  // 初始化显示左侧摩擦轮转速
  UICharDraw(&UI_fric_text_down, "fs0", UI_Graph_ADD, 6, UI_Color_White, 18, 2, 1556, 850, "frispeed:0");
  UICharRefresh(&referee_recv_info->referee_id, UI_fric_text_down);

    // 车头方向动态圆弧
    UIArcDraw(&UI_yaw_arc, "yd0", UI_Graph_ADD, 6, UI_Color_Main, 15, 345, 23, 1556, 721, 88, 88);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_yaw_arc);

    // 俯仰角仪表盘指针
    // 绘制指针初始位置（指向90°）
    UIArcDraw(&UI_pitch_needle, "pn0", UI_Graph_ADD, 6, UI_Color_Pink, 90 - 1, 90 + 1, 45, 960, 540, 365, 365);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_pitch_needle);

    UILineDraw(&Line_DecoMid, "ll0", UI_Graph_ADD, 6, UI_Color_White, 7, 577, 538, 606, 538);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Line_DecoMid);
    UIArcDraw(&Arc_DecoUp, "ll1", UI_Graph_ADD, 6, UI_Color_White, 310, 311, 22, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoUp);
    UIArcDraw(&Arc_DecoDown, "ll2", UI_Graph_ADD, 6, UI_Color_White, 229, 230, 22, 960, 535, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, Arc_DecoDown);
}

Referee_Interactive_info_t *getUI() { return &interactive_data; }

void UITask(RobotInstance *robot) {
  referee_recv_info = robot->referee_data;

  // 【1】按键强制清屏重绘（仅当按下 Ctrl 时触发）
  if (interactive_data.force_refresh_ui == 1) {
    MyUIInit(robot);  // 只有手动按下 Ctrl，才允许执行全屏 Delete 和静态线 ADD
    interactive_data.force_refresh_ui = 0;
  }

  // 【2】防丢包心跳保底（不删图层，只重发 CHANGE）
  static uint16_t slow_refresh_counter = 0;
  if (++slow_refresh_counter >= 150) {
    slow_refresh_counter = 0;
    // 仅仅把所有动态标志位置 1，逼迫 MyUIRefresh 重发一遍最新参数
    interactive_data.UI_Interactive_Flag.gimbal_flag = 1;
    interactive_data.UI_Interactive_Flag.shoot_flag = 1;
    interactive_data.UI_Interactive_Flag.fric_flag = 1;
    interactive_data.UI_Interactive_Flag.cap_flag = 1;
    interactive_data.UI_Interactive_Flag.ammo_flag = 1;
    interactive_data.UI_Interactive_Flag.fric_flag = 1;
    interactive_data.UI_Interactive_Flag.yaw_flag = 1;
    interactive_data.UI_Interactive_Flag.pitch_flag = 1;
    interactive_data.UI_Interactive_Flag.autoaim_flag = 1;
  }

  // 【3】核心修复：必须把真实数据喂给 interactive_data，UIChangeCheck 才能正常工作！
  // 基础状态
  interactive_data.robot_mode = robot->robot_mode;
  if (robot->chassis) {
    interactive_data.chassis_mode = robot->chassis->chassis_ctrl_cmd.chassis_mode;
    interactive_data.cap_voltage = robot->chassis->super_cap->cap_msg.cap_v;
  }
  if (robot->gimbal) {
    interactive_data.gimbal_mode = robot->gimbal->gimbal_ctrl_cmd.gimbal_mode;
    interactive_data.pitch_angle = robot->gimbal->gimbal_ctrl_cmd.pitch;                      // 俯仰角
    interactive_data.autoaim_mode = (interactive_data.gimbal_mode == GIMBAL_VISION) ? 1 : 0;
  }
  if (robot->shoot) {
    interactive_data.shoot_mode = robot->shoot->shoot_ctrl_cmd.shoot_mode;
    interactive_data.friction_mode = robot->shoot->shoot_ctrl_cmd.friction_mode;
    interactive_data.bullet_speed_mode_e = robot->shoot->shoot_ctrl_cmd.bullet_speed_mode;
    interactive_data.heat_mode_e = robot->shoot->shoot_ctrl_cmd.heat_mode;
    interactive_data.fric_speed_left = robot->shoot->friction_motor[0]->measure.speed_aps;
    interactive_data.fric_speed_right = robot->shoot->friction_motor[1]->measure.speed_aps;
  }

  // 动态数值（如果这里的变量名和你的底层解算名字不一致，请手动微调一下）
  interactive_data.chassis_relative_angle = robot->offset_angle;  // 底盘朝向
  if (referee_recv_info) {
    interactive_data.bullet_left_real =
        referee_recv_info->ProjectileAllowance.projectile_allowance_17mm;  // 剩余金币或弹量
  }


  // 检查是否有变化
  UIChangeCheck(&interactive_data);

  // 执行UI刷新
  MyUIRefresh(&interactive_data);
}