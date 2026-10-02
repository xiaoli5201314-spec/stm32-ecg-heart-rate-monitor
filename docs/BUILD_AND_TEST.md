# 构建与测试

本文对应 [Makefile](../firmware/Makefile)、[测试入口](../firmware/test/run_tests.c) 和 [Python 复核工具](../tools/verify_filters.py)。命令从仓库根目录执行，默认配置为 250 Hz、`ecg_real_t = double`。

已有运行证据单独记录在 [VERIFICATION.md](VERIFICATION.md)。测试阈值不是运行结果；下文没有列入该记录的命令均不能据此宣称已经通过。

## 1. 推荐环境与最小复现

使用 Linux 或 WSL 中的 GNU Make、GCC、标准 C 库和数学库。Makefile 在 Linux 自动启用 `ECG_HAVE_PTHREAD` 并链接 `-pthread`，以运行双线程 SPSC 测试。

Ubuntu 缺少工具时，可先安装：

```bash
sudo apt-get update
sudo apt-get install build-essential
```

在仓库根目录：

```bash
gcc --version
make --version
make -C firmware -B test CC=gcc
```

`CC=gcc` 明确选择编译器；`-B` 强制重编译，避免把旧目标文件当作本次验证。Makefile 默认包含：

```text
-std=c99 -O2 -g
-Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes
-Wpointer-arith -Wwrite-strings -Wundef -Wswitch-enum -Wredundant-decls
-MMD -MP -Werror
```

`-MMD -MP` 生成头文件依赖。默认 `WERROR=1`，构建成功说明该工具链在这些选项下未产生编译告警，不等于所有编译器均无告警。`WERROR=0` 可用于排查，但不能替代默认配置的验收。

Windows PowerShell 用户可先进入 WSL，再切换到映射的仓库目录：

```powershell
wsl.exe
```

```bash
# 将路径替换为自己的仓库位置；Windows 的 D: 通常映射为 /mnt/d。
cd '/mnt/d/你的仓库路径/stm32-ecg-heart-rate-monitor'
make -C firmware -B test CC=gcc
```

不要把 POSIX Makefile 的 `mkdir -p`、`uname`、`/dev/null` 等用法视为 Windows 原生工具链已验证的承诺。

## 2. 如何判断本次测试结果

测试程序输出各套件、SG 参数扫描、`MEASURED VALUES` 和最后的汇总。应同时检查：

1. 编译命令完成，Make 返回码为 0。
2. 最后显示 `RESULT: ALL TESTS PASSED`，失败套件和失败断言均为 0。
3. Linux 输出包含 `ring_buffer::spsc_threads` 和实际转移的 400000 个样本。
4. 没有 `pthread_create failed, skipping the threaded test` 等跳过提示。
5. 保存本次日期、工具链、命令和输出。断言数以本次运行日志为准。

当前 Linux 源码包含 28 个套件入口；非 Linux 未启用 pthread 时数量会减少。线程创建失败也可能跳过压力测试而不使整体失败，因此仅有退出码不足以证明并发测试已执行。

### 覆盖与验收条件

| 测试来源 | 覆盖内容 | 条件及解释 |
|---|---|---|
| [test_ring_buffer.c](../firmware/test/test_ring_buffer.c) | 参数校验、FIFO、回绕、满/空、部分写入、溢出记账、字节环、SPSC 双线程 | 双线程配置 400000 个样本；确认运行而非跳过 |
| [test_filters.c](../firmware/test/test_filters.c) | SG 系数、常数/多项式复现、噪声增益、流式/整段对齐、参数扫描 | 默认参数的平均峰值保持率至少 95%，最差单搏至少 93% |
| 同上：陷波 | 系数及频响、12 s 正弦时域测试，跳过前 4 s | 50 Hz 及 49.5、49.75、50、50.25、50.5 Hz 五个离散探针的衰减不大于 -20 dB |
| 同上：高通/标定 | 直流抑制、频响、多档合成平台拟合、中值抗毛刺、非法参数 | 合成标定增益误差小于 1%，R² 大于 0.999，非线性度小于 1% |
| [test_protocol.c](../firmware/test/test_protocol.c) | CRC 参考/查表版本、帧编解码、半包/粘包/垃圾/坏 CRC、载荷编码 | CRC 标准向量、码值往返、越界拒绝和既定恢复场景 |
| [test_heart_rate.c](../firmware/test/test_heart_rate.c) | 干净/含干扰心率、不应期、RR 异常值、HRV、SQI | 干净信号 7 档、含干扰 5 档；60 s 记录，30 s 后统计稳态误差不超过 2 BPM |
| [run_tests.c](../firmware/test/run_tests.c) | 模拟 ADC/DMA、环形缓冲、滤波、帧、UART 内存捕获及解码 | 60 s 虚拟采样；检查交付点数、丢点、序号、波形量化误差、CRC 和上行心率 |

SG 扫描使用阶数 `{2,3,4,5}`、窗长 `{5,7,9,11,13,15,17,21,25,31}`，跳过 `window <= order`，共 39 个有效组合。打印的最优组合取决于该合成场景和代价函数；测试没有断言默认 4 阶/17 点在所有场景下全局最优。

### 指标的适用范围

- `notch_worst_attenuation_49.5_50.5Hz_db` 是五个探针中的最差值，不是连续频带保证。
- `notch_passband_worst_attenuation_db` 的现有代码实际取各探针 dB 值中的最大值，即衰减最小者。它不能证明“最大通带插损小于 0.5 dB”。
- SG 流式与整段比较只适用于完成启动、避开边缘并补偿延迟后的内部区间。
- 60 s 集成测试是虚拟采样时长，不是墙钟运行时间，也不是真实电极或串口测试。
- 默认集成输入 15000 点，仅交付 `floor(15000/64)*64 = 14976` 点。末尾 24 点没有触发完整 DMA 半块回调，不计入环形缓冲丢点；“丢点为零”不能解读成全部输入都被交付。
- 当前 `sdnn_ms` 的计算缺少从 RR 采样点到毫秒的换算。周期信号的小方差测试不能验证其单位正确性。

## 3. CSV 导出与 NumPy 交叉复核

Python 工具不是标准库独立脚本：**必须安装 NumPy**。绘图另外需要 matplotlib。因此它没有加入当前仅 GCC/Make 的 CI。

2026-10-02 已在 WSL / NumPy 1.21.5 下执行 `make -C firmware dump` 和 `python3 tools/verify_filters.py`，均为 PASS；具体数值见 [验证记录](VERIFICATION.md)。这是本地执行记录，不是 GitHub Actions 的结果。以下复现命令显式指定 GCC 并强制重编译，与该次原始命令分开记录。

可选独立虚拟环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install numpy
```

生成本次 C 输出，再运行复核：

```bash
make -C firmware -B dump CC=gcc
ls -l firmware/build/coefficients.txt firmware/build/raw_signal.csv firmware/build/filtered_signal.csv
python3 tools/verify_filters.py
```

`dump` 先运行全部测试，再导出 30 s / 7500 点的系数与信号。三份文件必须来自同一次成功运行；检查修改时间和日志，尤其不能忽略 `cannot write ...`。系数文件写入失败目前不会直接作为 `dump` 的错误返回值传播。

| 文件 | 字段/作用 |
|---|---|
| `firmware/build/coefficients.txt` | C 使用的采样率、标定参数、高通/陷波参数与系数、SG 核 |
| `firmware/build/raw_signal.csv` | `sample,adc_count,electrode_uv,clean_uv,hum_uv,wander_uv` |
| `firmware/build/filtered_signal.csv` | `sample,notch_uv,filtered_uv,beat` |

`electrode_uv` 是标定换算值，默认截距为 0，尚含模拟中点对应的直流；不是纯净生物信号。`beat` 标记检测通知发生的样本，不是回溯细化后的 R 峰索引。

脚本重新计算陷波系数，以 NumPy `pinv` 求 SG 核，并逐样本复现高通、陷波和 SG 启动行为。当前成功判据包括：

- 系数最大偏差小于 `1e-9`。
- 完整链输出最大偏差小于 `1e-4 uV`。
- 脚本所定义的 SG 平均峰值保持率至少 95%。

全链路对干净参考的峰高保持率、含干扰峰高保持率、50 Hz 抑制量和心率推算值是分析输出，不是该脚本 PASS 的独立验收条件。SG 单级分析使用对齐的中心核；全链路保持率不能拿 SG 单级结果替代。峰高采用抛物线拟合并减去 R 峰前 48..120 ms 的 PQ 段中位基线，心率则按 CSV 中 `beat` 通知索引的中位间期推算。

**缺失文件的陷阱：** 工具缺少系数或 CSV 时会使用自己的默认参数/合成数据。这样的 `PASS` 不证明 C 与 Python 一致；必须确认输出明确加载三份 C 导出文件且显示完整链与 C 的对比。脚本不自动验证文件的新旧或同源性，其信号质量分析还使用固定 72 BPM 假设。

当前 Python 换算使用 `counts * uv_per_lsb`，没有减去导出的标定截距；只与默认零截距导出相符。打印的 `calibration` 偏差也只是同一表达式相减，不是对 C 标定列的独立校验。不能据此宣称非零偏置标定已被交叉验证。

可选绘图：

```bash
python3 -m pip install matplotlib
python3 tools/verify_filters.py --plot
```

## 4. 其他 Make 目标

| 仓库根目录命令 | 产物/行为 | 边界 |
|---|---|---|
| `make -C firmware demo CC=gcc` | 10 s 合成信号的逐搏数据与 ASCII 波形 | 演示模式不运行完整单元测试，不是 GUI |
| `make -C firmware lib CC=gcc` | `firmware/build/libecg.a` | 仅六个 `src/*.c` 对象，不包含 HAL；不代表完整板级固件 |
| `make -C firmware size CC=gcc` | 主机对象文件尺寸 | 不能替代目标 MCU 的链接 map、栈深度和运行计时 |
| `make -C firmware arm` | 对六个源文件做 Cortex-M3 编译检查 | 输出写入 `/dev/null`，没有固件镜像 |

ARM 检查先确认工具存在：

```bash
command -v arm-none-eabi-gcc
make -C firmware arm
```

仅在前一条命令成功后运行后一条。Makefile 的缺工具提示虽然包含 `skipping`，后续独立 recipe 仍可能尝试编译而失败，不能认为缺少工具会可靠成功跳过。该目标不含启动代码、链接脚本、STM32 HAL 端口或板级 `main`，也未使用主机测试同一套完整告警参数。

[platformio.ini](../firmware/platformio.ini) 是配置入口，不是已经完成构建/烧录的证据。不要以其中的目标板环境声明代替实际板级实现。

## 5. CI 的边界

[ci.yml](../.github/workflows/ci.yml) 在 `push`、`pull_request` 和 `workflow_dispatch` 触发，使用 `ubuntu-22.04`、`actions/checkout@v4` 和只读 `contents` 权限。作业输出 GCC/Make 版本并运行：

```bash
make -C firmware -B test CC=gcc
```

它不烧录硬件、不运行 `dump`/NumPy、不发布任何产物，也不证明临床有效性。**NumPy 交叉复核目前仅有上述本地执行证据**；当前 CI 保持 C 构建与测试，不把脚本回退模式的 PASS 当作 C 导出验证。runner 系统版本已固定，但镜像中的 GCC/Make 可能更新，应以作业日志为准。配置存在不等于远程 CI 已通过；实际远程执行状态见 [验证记录](VERIFICATION.md)。
