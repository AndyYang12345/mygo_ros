# 组合键意图消息
# 发布者: 抽象意图节点
# 订阅者: 状态机节点

# 组合类型
# 使用字符串标识，便于扩展
# 标准组合：
# "LT_RT" - 左右扳机同时按下
# "LB_RB" - 左右肩键同时按下
# "PLUS_MINUS" - + 与 - 同时按下
# "A_B" - A+B同时按下
# "LEFT_STICK_RIGHT_STICK" - 双摇杆同时按下
# 自定义组合可自由定义
string combo_type

# 组合触发时长（秒）
# 0.0: 立即触发
# >0.0: 长按触发
float32 duration

# 时间戳
builtin_interfaces/Time timestamp

# 事件来源
string source

# 组合中包含的按键
uint8[] involved_buttons

# 附加数据（例如摇杆组合时的位置）
float32 left_joystick_x
float32 left_joystick_y
float32 right_joystick_x
float32 right_joystick_y