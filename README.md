# Python Packages for xArm5 Teleoperation

## 1. Package ikfastpy_xarm5

> 功能：基于xArm5模型利用ikfastpy插件对xArm5进行逆运动学求解，用以根据机器人TCP位姿来求解关节位置（已完成，待验证）。

## 2. Package model

> 功能：使用pybullet引擎加载xArm5的urdf模型文件，并利用引擎来进行逆运动学求解。

## 3. Script slave_xarm5

> 功能：与主操作手Touch进行通讯，接收Touch的控制指令并跟踪主操作手的运动。<br>
> 启动：sh slave_xarm5.sh
