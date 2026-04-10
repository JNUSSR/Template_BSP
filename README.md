# JNUSSR Template BSP

暨南大学 SSR 战队 ROBOCON BSP 模板工程。

本项目基于 robowalker_train 框架思路，融合 STM32CubeMX + CMake + FreeRTOS，面向 STM32F427 平台，作为队内通用板级支持包与任务开发基础。

## 特性

- MCU: STM32F427 系列
- RTOS: FreeRTOS（Middlewares/Third_Party/FreeRTOS）
- HAL: STM32F4xx HAL Driver
- 构建系统: CMake + Ninja（兼容 GCC Arm Embedded 工具链）
- 工程兼容: 同时保留 Keil MDK 工程目录，便于迁移与调试
- 用户代码层: User 目录按算法、驱动、模块、任务分层组织

## 目录说明

- Core: CubeMX 生成的核心启动与外设代码
- Drivers: CMSIS 与 HAL 驱动
- Middlewares: FreeRTOS 等中间件
- User: 战队业务层代码（算法/驱动/模块/任务）
- cmake: 工具链与 CubeMX CMake 适配脚本
- MDK-ARM: Keil 工程与构建产物（历史兼容）

## 构建说明（CMake）

1. 安装依赖
- CMake >= 3.22
- Ninja
- gcc-arm-none-eabi 工具链

2. 配置与构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

默认目标名为 `test_feedback`，链接脚本为 `STM32F427XX_FLASH.ld`。

## 开发建议

- 新业务代码优先放在 User 目录并按现有分层维护
- 尽量避免直接改动 CubeMX 自动生成代码区块
- 通过 CMake 构建进行日常编译校验，Keil 用于下载与在线调试

## 来源与用途

- 框架参考: https://github.com/yssickjgd/robowalker_train.git
- 用途: JNUSSR ROBOCON 竞赛 BSP 基础模板

## 许可证

如无额外声明，按仓库后续约定执行。
