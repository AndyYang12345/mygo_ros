# 菜单导航消息
# 发布者: 状态机节点（转发）或意图解析节点
# 订阅者: GUI界面

# 导航动作
# 0: UP - 向上
# 1: DOWN - 向下  
# 2: LEFT - 向左
# 3: RIGHT - 向右
# 4: CONFIRM - 确认
# 5: CANCEL - 取消
# 6: NEXT_PAGE - 下一页
# 7: PREV_PAGE - 上一页
uint8 action

# 连续值（用于摇杆导航）
float32 intensity  # 移动强度 0.0-1.0

# 当前选中项
uint32 current_selection

# 菜单层级
uint32 level

# 时间戳
builtin_interfaces/Time timestamp