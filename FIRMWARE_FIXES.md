# 固件修复记录

日期：2026-11-06
依据：`matlab/` 下的建模与仿真分析（详见 `matlab/README.md`）

原文件已备份到 `backup_20261106/`：

```
backup_20261106/main.cpp.bak
backup_20261106/Kms.c.bak
backup_20261106/Kms.h.bak
```

**回滚**：把上面三个 `.bak` 复制回原路径即可。

---

## 一、改动清单

### A. 控制参数（2 处，影响最大）

| 编号 | 位置 | 原值 | 改为 | 依据 |
|---|---|---|---|---|
| FIX-1 | `src/main.cpp:57` `vel_kp` | `-0.05f` | `+0.05f` | 速度偏快时需要**后倾减速**，原负号变成前倾加速（正反馈）。线性模型：`-0.05` 在 s=+0.21 处产生不稳定极点；仿真中轮速冲到 4.7 m/s、位移漂 32 m |
| FIX-2 | `src/main.cpp:59` `balance_kd` | `-0.02f` | `-0.01f` | `-0.02` 把最快闭环极点推到 230 rad/s，10 ms 采样撑不住。仿真出现 **14.9 Hz 极限环，θ 峰峰 1.4°，力矩 60% 时间饱和**（实物=高频嗡嗡抖+电机发烫） |

> 注意：`balance_kd` 的**负号是对的**（因为 `e = target − pitch`，等效标准 Kd=+0.02 为正阻尼），问题在幅值，不要改成正号。

### B. 实时性（1 处，最关键）

| 编号 | 位置 | 改动 |
|---|---|---|
| FIX-3 | `src/main.cpp:331` | 删除 `set_leg_position_smooth()` 末尾的 `vTaskDelay(duration_ms + 100)`；在 `initialize_robot()` 里补 `vTaskDelay(2100ms)` |

原实现在 **10 ms 控制任务里阻塞 350 ms**，期间不发新力矩、不跑 `motor_update()`，
整个平衡环冻结。仿真中**第一次触发（0.22 s）就直接跌倒**。

改为"只发指令立即返回"，舵机自己走完 250 ms。上电阶段仍需等待，因为那时控制任务
还没创建，阻塞是安全的。

> 副作用：`set_leg_position_smooth()` 不再保证返回时舵机已到位。目前所有调用点
> （BLE 的 height/forward/resetpos）本来就不依赖到位，安全。

### C. 正确性（4 处）

| 编号 | 位置 | 原代码 | 改为 |
|---|---|---|---|
| FIX-4 | `main.cpp:266` + `Kms.h` | `JointAngles joint_angles;` 未初始化 | `= {0}`，并检查 `inverse_kinematics()` 返回值 |
| FIX-5 | `main.cpp:413` | `robot_kp * leg_position_offset` | `robot_kp * position_error` |
| FIX-6 | `main.cpp:434` | 阈值 `0.5 mm`、无频率限制 | `2.0 mm` + 最小 100 ms 间隔 |
| FIX-7 | `main.cpp:367` | `current_velocity * 0.01f` 固定 dt | 用实际经过时间 `dt_int` |

- **FIX-4**：`Kms.c` 原来无解时**完全不写**输出结构体，`main.cpp` 会读到栈上垃圾值。
- **FIX-5**：原式是自己乘自己，`robot_kp > 0` 时会指数放大。当前 `robot_kp = 0` 未启用，属预防性修复。
- **FIX-6**：原判据 0.5 mm ≈ 0.25°，仿真 20 s 内触发 **1298 次（65 Hz）**，配合原阻塞式舵机
  等于让控制环长期停摆。
- **FIX-7**：当前 `robot_kp = 0`，暂不影响行为，属预防性修复。

### D. 并联五连杆机构（3 处，`lib/Km/`）

| 编号 | 位置 | 改动 |
|---|---|---|
| FIX-8 | `Kms.c` beta 归一化 | `beta ∈ [0,2π)` → 折到 `(-π, π]` |
| FIX-9 | `Kms.c` beta 选支 | 独立判据 → `select_beta()` 取 `[-π/2, π/2]` 内的解 |
| FIX-10 | `Kms.c` / `Kms.h` | `void inverse_kinematics()` → `bool`，输出先清零 |

**FIX-8 的由来**（只在并联五连杆上才会出现）：

五连杆正常工作时 `beta` 会随 `leg_x` 减小穿过 0。原归一化
`beta = (beta >= 0) ? beta : beta + 2π` 只处理了负数，漏了 `beta` 已接近 `+2π` 的情况：

| leg_x | beta 原始值 | 归一化后 | 小腿脉冲 |
|---|---|---|---|
| 52 mm | −359.55° | 0.45° | 2033 |
| **53 mm** | **+359.02°** | **359.02°** | **1444（跳 −589 μs）** |

`359.02°` 随后被 `main.cpp:280` 的 `beta_deg ∈ [-20, 80]` **截成 80°**，
`s2 = 90 − 80 = 10`，脉冲瞬间塌下去。等价于**舵机瞬间甩约 80°**。

- 正确值应为 `−0.98°` → `s2 = 90.98` → 脉冲 **2043**（平滑连续）
- 触发条件：`leg_x > 52.7 mm`，即**前倾超过 6.35°** 就会踩到
- 修复后 `beta` 在 `[-90°, 90°]` 内连续变化，脉冲不再跳变

**FIX-9**：闭链机构两个曲柄必须落在同一装配构型。实测该机构两支分别约 `190°` 和 `~0°`，
取 `[-90°, 90°]` 内的一支即可唯一确定。原代码两个曲柄各自独立选支，理论上会解出
互不相容的"假解"（扫描 8991 点，实际工作区 `x∈[20,60]` 内未触发，但右边缘 `x>64` 有 2 点）。

### E. 启用位置环（3 处，新增）

| 编号 | 位置 | 改动 |
|---|---|---|
| FIX-11 | `6050_middle.cpp:310` | 去除 `+0.6°` 硬偏置 → `PITCH_BIAS_DEG`（默认 `0.0f`） |
| FIX-12 | `main.cpp` 参数区 / `read_wheel_velocity()` | 新增 `WHEEL_RADIUS_MM`；`current_position` 积分时乘轮半径，单位由 rad 改为 **m** |
| FIX-13 | `main.cpp` → `calculate_wheel_torque()` | 位置环从"腿部偏移"改到"目标俯仰角"；`robot_kp` 默认 `2.0`（单位 deg/m），BLE 范围放宽到 0~20 |

**为什么要改接法**

原实现（`adjust_leg_position_for_balance`）把位置误差加到 `leg_position_offset` 上。
但腿部偏移只是移动质心、改变**平衡点**，并不产生持续力矩——仿真里 100 s 后仍有
0.36 m 误差且速度 0.47 m/s 收不回来。

改成加到目标俯仰角后，位置误差直接经直立环变成轮子力矩，才能真正把机器人拉回原点：

| 配置 | 100 s 后位置误差 | 末速度 | 判定 |
|---|---|---|---|
| 不开位置环 | 15.0 m | 0 | 一直漂 |
| 旧接法（腿偏移） | 0.36 m | 0.47 m/s | ✗ 振荡不收 |
| **新接法（目标俯仰角）** | **0.004 m** | **0.002 m/s** | **✓ 收敛** |

**为什么必须同时去掉 +0.6° 偏置**

`6050_middle.cpp:310` 原来写死 `pitch = sma(ky) + 0.6f`。这个偏置会直接变成位置环的
稳态误差：

```
稳态误差 = PITCH_BIAS_DEG / robot_kp
```

实测（`fig13`，k=1/2/3）稳态误差 0.599 / 0.301 / 0.201 m，与 0.6/k = 0.600 / 0.300 / 0.200
严格吻合。偏置归零后误差降到 **1.4~4.2 mm**。

**关于 `WHEEL_RADIUS_MM`**

`current_velocity` 来自 `enc_get_speed()`，单位是**轮子角速度 rad/s**，不是 m/s。
位置环要真实的米制位移，必须乘轮半径。当前填的 `35.0f` 是估测值，
**请按实测修改**——这个值直接决定位置环增益标定的准确度。

**整定结果与代价**

- `robot_kp = 2.0`：扰动 J=1.0 N·s 后峰值 7.93°（<10° 要求），收敛到 1.4 mm
- `robot_kp = 3.0`：峰值 9.93°，已贴 10° 上限，不建议再高
- **代价**：位置会来回摆约 ±3.8 m、约 60 s 才收敛（`fig14`）。
  因为速度环仍是纯 P，位置环只能借它的慢通道；想更快必须给速度环加积分

---


## 二、验证状态

| 项目 | 状态 |
|---|---|
| **完整构建** | ✅ **通过**。`pio run` → `[SUCCESS] Took 61.53 s`，`firmware.bin` 618 KB，RAM 6.7% / Flash 58.9% |
| 逐个文件编译 | ✅ `main.cpp` / `Kms.c` / `Kms_move.c` / `6050_middle.cpp` / `esp_hw_link.cpp` / `duoji_control.c` 全部 0 错误 0 警告 |
| diff 复核 | ✅ 逐条人工复核 |
| 实机验证 | ❌ 未做（需上电测试） |

构建仅剩 2 条**改动前就存在**的警告，与本轮修改无关：

```
lib/Km/Kms.c:28:13: warning: 'limit_y' defined but not used
lib/6050mpu/6050_middle.cpp:34:20: warning: 'TAG' defined but not used
```

### 构建环境（重要，换机器要重做）

`~/.platformio/penv/.espidf-5.5.3` 这个 ESP-IDF Python 环境一开始缺 3 个包，
导致构建失败。**注意要装进这个 venv，不是系统 Python / Anaconda**：

```powershell
$py = 'C:\Users\98134\.platformio\penv\.espidf-5.5.3\Scripts\python.exe'
& $py -m pip install esp-idf-kconfig idf-component-manager cryptography
```

三个包分别对应三个报错：

| 缺失包 | 报错 |
|---|---|
| `idf-component-manager` | `No module named 'idf_component_manager'` |
| `esp-idf-kconfig` | `No module named 'kconfgen.__main__'` |
| `cryptography` | `gen_crt_bundle.py` 静默失败 → `x509_crt_bundle.S` 缺失 |

> `kconfgen` 这个 PyPI 包**不是** ESP-IDF 要的，必须装 `esp-idf-kconfig`。
>
> 另外若之前多次构建失败过，`.pio/build/esp32-s3-devkitc-1` 可能残留脏配置，
> 删掉重建即可（`pio run` 会自动重新生成）。

---

## 三、实机测试建议顺序

按这个顺序逐项验证，每步出问题都先回滚再排查：

1. **上电**：应仍能正常起立（FIX-3 上电等待保留了），串口出现「舵机指令已下发」
2. **`start` 后静止观察**：若仍高频抖动，说明 FIX-2 不够，把 `balance_kd` 再往 `-0.005` 调
3. **轻推前后**：
   - 若机器人**越跑越快** → FIX-1 方向反了，把 `vel_kp` 改回 `-0.05`（这是唯一一个靠模型推出的符号，实机以实测为准）
   - 若能自我恢复 → FIX-1 正确
4. **前倾超过 6.35°**（可用 `forward:` 命令逼近）：小腿舵机应**平滑**转动，不再猛甩
5. **长时间运行**：串口不应再刷屏「更新腿部位置」（FIX-6 生效）

---

## 四、未改动但需要注意

| 位置 | 问题 | 为什么没动 |
|---|---|---|
| `main.cpp:287-290` / `306-309` | 左右舵机限幅不一致（左 `s0∈[0,130]`、右 `s0∈[0,90]`） | **可能是机械安装差异，属标定值**，盲目统一会让腿歪。建议实测后按实际行程重标 |
| `main.cpp:64` | `torque_normalization_factor = 1.0/max_torque` 恒等于 1，"归一化"是空操作 | 无害，只改注释会更乱 |
| `main.cpp:247` | 日志单位标 `m/s`，实际是 `rad/s` | 纯注释问题，不影响行为 |
| `enc_get_speed()` | 返回 rad/s，被当 m/s 用 | 会导致 `max_velocity=2.0` 实际只有 2 rad/s ≈ 0.07 m/s，但改它会同时改变速度环量纲，**必须和 FIX-1 一起在实机上重新标定**，这次先不动 |
