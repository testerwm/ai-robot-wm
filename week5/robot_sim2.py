import pybullet as p
import pybullet_data as pd
import time
import math

# --- 1. 环境初始化 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # 隐藏默认GUI面板，保持画面简洁
p.resetDebugVisualizerCamera(1.8, 60, -35, [0.5, 0, 0.65])

# 加载地面和桌子
p.loadURDF("plane.urdf")
table_pos = [0.5, 0, 0]
p.loadURDF("table/table.urdf", table_pos, useFixedBase=True)

# 加载机械臂
panda_pos = [0.5, 0, 0.625]
pandaId = p.loadURDF("franka_panda/panda.urdf", panda_pos, useFixedBase=True)

# ---------------------------------------------------------------
# 工具函数：平滑移动到目标关节角度（用于衔接动作）
# ---------------------------------------------------------------
def move_to_joints(target_joints, steps=120, delay=1./120.):
    """在 steps 步内平滑插值到目标关节角度"""
    current = [p.getJointState(pandaId, i)[0] for i in range(7)]
    for step in range(steps):
        t = (step + 1) / steps
        for i in range(7):
            angle = current[i] + t * (target_joints[i] - current[i])
            p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, angle, force=500)
        p.stepSimulation()
        time.sleep(delay)

def move_to_cartesian(tx, ty, tz, steps=80, delay=1./120.):
    """在 steps 步内平滑移动到目标笛卡尔坐标（IK模式）"""
    # 获取当前末端位置
    ee_state = p.getLinkState(pandaId, 11)
    curr = ee_state[0]
    for step in range(steps):
        t = (step + 1) / steps
        ix = curr[0] + t * (tx - curr[0])
        iy = curr[1] + t * (ty - curr[1])
        iz = curr[2] + t * (tz - curr[2])
        joint_poses = p.calculateInverseKinematics(
            pandaId, 11, [ix, iy, iz],
            p.getQuaternionFromEuler([math.pi, 0, 0])
        )
        for i in range(7):
            p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joint_poses[i], force=500)
        p.stepSimulation()
        time.sleep(delay)

def add_label(text, color=[0, 0, 0]):
    """在屏幕固定位置显示提示文字"""
    return p.addUserDebugText(text, [0.05, -0.6, 1.3], color, 1.4)

def draw_trail(positions, color=[1, 0, 0]):
    """根据位置列表画轨迹线"""
    for i in range(1, len(positions)):
        p.addUserDebugLine(positions[i-1], positions[i], color, 2, lifeTime=0)

# ---------------------------------------------------------------
# 初始姿态（自然下垂，避免奇异点）
# ---------------------------------------------------------------
HOME_JOINTS = [0, -0.5, 0, -2.0, 0, 1.5, 0.8]

print("=" * 55)
print("  Panda 机械臂演示：笛卡尔画圆 + 关节空间画直线")
print("=" * 55)
print("\n[准备] 机械臂回到初始姿态...")
label_id = add_label("Initializing...", [0.3, 0.3, 0.3])
move_to_joints(HOME_JOINTS, steps=180)

# ===============================================================
# 任务一：笛卡尔空间画圆（IK控制）
# ===============================================================
print("\n[任务1] 笛卡尔空间 —— 画圆")
print("  圆心: (0.6, 0.0, 0.85)  半径: 0.15m")
p.removeUserDebugItem(label_id)
label_id = add_label("Task 1: Cartesian Circle (IK)", [0.0, 0.4, 0.9])

# 圆的参数
cx, cy, cz = 0.6, 0.0, 0.85   # 圆心（笛卡尔坐标）
radius = 0.15                   # 半径（米）
total_angle = 2 * math.pi       # 整圆
circle_steps = 300              # 步数越多越平滑

# 先平滑移动到圆的起点
start_x = cx + radius * math.cos(0)
start_y = cy + radius * math.sin(0)
move_to_cartesian(start_x, start_y, cz, steps=120)

# 逐步画圆，并记录轨迹
trail_circle = []
for step in range(circle_steps + 1):
    angle = total_angle * step / circle_steps
    tx = cx + radius * math.cos(angle)
    ty = cy + radius * math.sin(angle)
    tz = cz

    joint_poses = p.calculateInverseKinematics(
        pandaId, 11, [tx, ty, tz],
        p.getQuaternionFromEuler([math.pi, 0, 0])
    )
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joint_poses[i], force=500)

    p.stepSimulation()
    time.sleep(1. / 120.)

    # 记录真实末端位置（顺运动学结果）
    ee_pos = p.getLinkState(pandaId, 11)[0]
    trail_circle.append(list(ee_pos))

# 画出圆轨迹（红色）
draw_trail(trail_circle, color=[1, 0.1, 0.1])
print("  ✓ 圆轨迹绘制完成（红色）")

# ===============================================================
# 任务二：关节空间画直线（FK控制）
# ===============================================================
print("\n[任务2] 关节空间 —— 画直线（J1肩关节匀速扫动）")
p.removeUserDebugItem(label_id)
label_id = add_label("Task 2: Joint Space Line (FK)", [0.1, 0.7, 0.2])

# 先回到初始姿态
print("  [过渡] 回到初始姿态...")
move_to_joints(HOME_JOINTS, steps=150)

# 通过改变 J1（肩关节）产生一段弧形直线轨迹
# 起点关节角度：J1 = -0.8
# 终点关节角度：J1 = +0.8
# 其余关节固定，产生近似水平直线
LINE_START = [0, -0.8, 0, -1.8, 0, 1.2, 0.8]
LINE_END   = [0,  0.8, 0, -1.8, 0, 1.2, 0.8]
line_steps = 200

# 先移动到直线起点
move_to_joints(LINE_START, steps=150)

# 逐步扫动关节，记录末端轨迹
trail_line = []
for step in range(line_steps + 1):
    t = step / line_steps
    joints_now = [
        LINE_START[i] + t * (LINE_END[i] - LINE_START[i])
        for i in range(7)
    ]
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joints_now[i], force=500)

    p.stepSimulation()
    time.sleep(1. / 120.)

    ee_pos = p.getLinkState(pandaId, 11)[0]
    trail_line.append(list(ee_pos))

# 画出直线轨迹（绿色）
draw_trail(trail_line, color=[0.1, 0.9, 0.2])
print("  ✓ 直线轨迹绘制完成（绿色）")

# ===============================================================
# 演示完成，保持画面展示轨迹
# ===============================================================
move_to_joints(HOME_JOINTS, steps=150)
p.removeUserDebugItem(label_id)
add_label("Done! Red=Circle  Green=Line  (Close window to exit)", [0.5, 0.1, 0.1])

print("\n" + "=" * 55)
print("  演示完成！")
print("  红色轨迹 = 笛卡尔空间画圆（任务1）")
print("  绿色轨迹 = 关节空间画直线（任务2）")
print("  关闭仿真窗口退出程序。")
print("=" * 55)

# 保持窗口打开，直到用户关闭
try:
    while True:
        p.stepSimulation()
        time.sleep(1. / 60.)
except Exception:
    pass
finally:
    p.disconnect()