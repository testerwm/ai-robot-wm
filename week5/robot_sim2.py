import pybullet as p
import pybullet_data as pd
import time
import math

# --- 1. 环境初始化 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(1.8, 60, -35, [0.5, 0, 0.65])

p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)
pandaId = p.loadURDF("franka_panda/panda.urdf", [0.5, 0, 0.625], useFixedBase=True)

# ---------------------------------------------------------------
# Panda 7个关节的真实物理限位（单位：弧度）
# ---------------------------------------------------------------
LOWER  = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
UPPER  = [ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
RANGES = [u - l for u, l in zip(UPPER, LOWER)]
# 静止姿态参考值（IK 优先靠近这组角度）
REST   = [0, -0.5, 0, -2.0, 0, 1.5, 0.8]

def ik_solve(tx, ty, tz):
    current = [p.getJointState(pandaId, i)[0] for i in range(7)]
    return p.calculateInverseKinematics(
        pandaId, 11,
        [tx, ty, tz],
        p.getQuaternionFromEuler([math.pi, 0, 0]),
        lowerLimits=LOWER,       # ← 关键：加关节限位
        upperLimits=UPPER,
        jointRanges=RANGES,
        restPoses=REST,          # ← 偏好姿态（全局固定，不跟随当前）
        maxNumIterations=200,
        residualThreshold=1e-5
    )

def move_to_joints(target_joints, steps=120, delay=1./120.):
    current = [p.getJointState(pandaId, i)[0] for i in range(7)]
    for step in range(steps):
        t = (step + 1) / steps
        for i in range(7):
            angle = current[i] + t * (target_joints[i] - current[i])
            p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, angle, force=500)
        p.stepSimulation()
        time.sleep(delay)

def move_to_cartesian(tx, ty, tz, steps=100, delay=1./120.):
    curr = p.getLinkState(pandaId, 11)[0]
    for step in range(steps):
        t = (step + 1) / steps
        ix = curr[0] + t * (tx - curr[0])
        iy = curr[1] + t * (ty - curr[1])
        iz = curr[2] + t * (tz - curr[2])
        joint_poses = ik_solve(ix, iy, iz)
        for i in range(7):
            p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joint_poses[i], force=500)
        p.stepSimulation()
        time.sleep(delay)

def add_label(text, color=[0, 0, 0]):
    return p.addUserDebugText(text, [0.05, -0.6, 1.3], color, 1.4)

def draw_trail(positions, color=[1, 0, 0]):
    for i in range(1, len(positions)):
        p.addUserDebugLine(positions[i-1], positions[i], color, 2, lifeTime=0)

# ---------------------------------------------------------------
# 初始姿态
# ---------------------------------------------------------------
HOME_JOINTS = [0, -0.5, 0, -2.0, 0, 1.5, 0.8]

print("=" * 55)
print("  Panda 机械臂演示：笛卡尔画圆 + 关节空间画直线")
print("=" * 55)
label_id = add_label("Initializing...", [0.3, 0.3, 0.3])
move_to_joints(HOME_JOINTS, steps=180)

# ===============================================================
# 任务一：XY 水平平面画完整圆
# ===============================================================
print("\n[任务1] 笛卡尔空间 —— XY 水平平面画圆")
print("  圆心: (0.55, 0.0, 0.75)  半径: 0.10m")
p.removeUserDebugItem(label_id)
label_id = add_label("Task 1: Cartesian Circle (IK)", [0.0, 0.4, 0.9])

cx, cy, cz   = 0.95, 0.2, 0.75
radius       = 0.20
total_angle  = 4 * math.pi
circle_steps = 1000

move_to_cartesian(cx + radius, cy, cz, steps=120)

trail_circle = []
for step in range(circle_steps + 1):
    angle = 4 * math.pi * step / circle_steps
    tx = cx + radius * math.cos(angle)
    ty = cy + radius * math.sin(angle)
    tz = cz

    joint_poses = ik_solve(tx, ty, tz)
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joint_poses[i], force=500)
    p.stepSimulation()
    time.sleep(1. / 120.)

    ee_pos = p.getLinkState(pandaId, 11)[0]
    trail_circle.append(list(ee_pos))

draw_trail(trail_circle, color=[1, 0.1, 0.1])
print("  ✓ 圆轨迹绘制完成（红色）")

# ===============================================================
# 任务二：关节空间画直线（FK 控制）
# ===============================================================
print("\n[任务2] 关节空间 —— 画直线")
p.removeUserDebugItem(label_id)
label_id = add_label("Task 2: Joint Space Line (FK)", [0.1, 0.7, 0.2])

move_to_joints(HOME_JOINTS, steps=150)

LINE_START = [0, -0.8, 0, -1.8, 0, 1.2, 0.8]
LINE_END   = [0,  0.8, 0, -1.8, 0, 1.2, 0.8]
line_steps = 200

move_to_joints(LINE_START, steps=150)

trail_line = []
for step in range(line_steps + 1):
    t = step / line_steps
    joints_now = [LINE_START[i] + t * (LINE_END[i] - LINE_START[i]) for i in range(7)]
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joints_now[i], force=500)
    p.stepSimulation()
    time.sleep(1. / 120.)

    ee_pos = p.getLinkState(pandaId, 11)[0]
    trail_line.append(list(ee_pos))

draw_trail(trail_line, color=[0.1, 0.9, 0.2])
print("  ✓ 直线轨迹绘制完成（绿色）")

# ===============================================================
# 演示完成
# ===============================================================
move_to_joints(HOME_JOINTS, steps=150)
p.removeUserDebugItem(label_id)
add_label("Done! Red=Circle  Green=Line  (Close window to exit)", [0.5, 0.1, 0.1])

print("\n" + "=" * 55)
print("  演示完成！红色=圆  绿色=直线")
print("  关闭仿真窗口退出程序。")
print("=" * 55)

try:
    while True:
        p.stepSimulation()
        time.sleep(1. / 60.)
except Exception:
    pass
finally:
    p.disconnect()