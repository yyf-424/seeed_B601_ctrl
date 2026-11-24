"""b601_safety —— B601 机械臂的独立安全监控包。

    logic.py           纯判定逻辑（无 ROS 依赖，可单测）
    safety_monitor.py  节点 b601_safety_monitor（订阅 /joint_states，发布安全状态，急停时全臂失能）
"""
