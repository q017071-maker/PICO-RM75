# RM75–PICO 遥操作

PICO 4 Ultra 双控制器经 Unity/XR、Wi-Fi UDP 控制睿尔曼 RM75 七轴机械臂。保留原6DoF右手柄位姿跟随，新增双摇杆速度输入和连续平移入口。所有实控都需要右Grip使能及终端手动确认，模式互斥。

**从另一台电脑复现请阅读 [操作手册](docs/操作手册.md)**，包括从零准备、PICO构建安装、接线、两终端操作、停机、故障排查和回退。

```bash
git clone https://github.com/q017071-maker/PICO-RM75.git
cd PICO-RM75
# 首次部署按手册安装 Unity/PICO SDK，并准备 Python 环境。
# 设备和 APK 已准备好的日常启动：
bash 启动.sh joystick-bridge
# 另一个终端：
bash 启动.sh joystick-continuous
```

连续入口需手动输入 `MOVE_RM75_CONTINUOUS`，先释放右Grip等待READY，再按住Grip拨摇杆。平移最高5cm/s、两路输入目标50Hz，取消每次Grip的25cm平移半径和30°累计关节预算；保留绝对限位5°裕量、3°单步、奇异性阈值0.01和故障停止。旋转仍保留每次Grip约10°范围，不能保证任意方向都能走到机械硬止挡。

| 模式 | 启动命令 | 范围/用途 |
|---|---|---|
| 连续摇杆平移 | `bash 启动.sh joystick-continuous` | 持续速度输入，直到可达安全边界或保护触发 |
| 旧摇杆模式 | `bash 启动.sh joystick-live` | 25cm半径、关节累计30°预算，供回退 |
| 原6DoF位姿跟随 | `bash 启动.sh live-6dof` | 右手柄位置/姿态相对跟随，原参数保留 |
| 摇杆只读预览 | `bash 启动.sh joystick-preview` | 连接机器人读状态，旧25cm范围；不发运动 |
| 无设备模拟 | `bash 启动.sh joystick-offline` | 无SDK/网络/机器人连接 |

Unity主工程在 **`project/`**，应在Unity Hub中添加这个子目录。仓库根 `Assets/`、`tests/`、`README_STABLE_VERSION.txt` 是原2026-10-28基线，保留用于历史比较；当前部署、开发和测试以 `project/` 和 `deployment/` 为准。

运行基线：Ubuntu24.04 x86_64、Python3.12、robotic-arm1.1.6、Unity2022.3.62f3、PICO Integration3.4.0。不需要ROS。Unity SDK、Python虚拟环境、APK、签名密钥及现场日志不随源码提交，应按手册在本机生成。第三方模型和样例的来源见 [说明](docs/第三方依赖.md)。

本次离线验证：302项项目测试、34项部署测试通过；1项真实UDP测试受沙箱限制跳过。连续入口已由现场用户运行并反馈可用；现场日志目标规划回放14631帧通过，并观察到奇异性HOLD。该记录不能代替新设备的方向、停止距离和运动中失联验收，详见 [版本与验证记录](docs/版本与验证记录.md)。
