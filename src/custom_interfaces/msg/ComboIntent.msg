# 组合键意图消息
# 发布者: 抽象意图节点
# 订阅者: 状态机节点

# 组合类型
# 使用字符串标识，便于扩展
# 标准组合：
# "LT_RT_CONFIRM" - 左右扳机同时按下
# "LB_RB" - 左右肩键同时按下
# "PLUS_MINUS" - + 与 - 同时按下
# "A_B" - A+B同时按下
# "LEFT_STICK_RIGHT_STICK" - 双摇杆同时按下
# 自定义组合可自由定义
string combo_name

# 时间戳
builtin_interfaces/Time timestamp

# 事件来源
string source