# 机器人状态消息
# 发布者: 状态机节点 (10Hz广播)
# 订阅者: 所有需要感知状态的节点

# 主状态枚举
# 0: UNKNOWN - 未知/初始化
# 1: IDLE - 空闲
# 2: CHASSIS - 底盘控制模式
# 3: ARM - 机械臂控制模式
# 4: MENU - 菜单选择模式
# 5: PRESET - 预设动作模式
# 6: EMERGENCY - 紧急停止
# 7: CALIBRATION - 校准模式
# 8: ERROR - 错误状态
uint8 main_state

# 子状态（用于菜单等需要更细粒度状态的场景）
# 例如菜单中的选中项索引
uint8 sub_state

# 状态名称（用于显示）
string state_name

# 可用模式列表（用于GUI显示）
string[] available_modes

# 当前菜单选项（如果在MENU模式）
uint8 menu_selection
string[] menu_items

# 系统健康状态
# 0: OK
# 1: WARNING
# 2: ERROR
# 3: CRITICAL
uint8 health_status

# 错误信息（如果有）
string error_message

# 时间戳
builtin_interfaces/Time timestamp

# 附加状态标志（位掩码）
# bit0: 电池低电量
# bit1: 通信异常
# bit2: 关节超限
# bit3: 碰撞检测
uint16 flags