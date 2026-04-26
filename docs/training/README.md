# RoboCon 固件培训手册

这组文档是给新加入战队的同学看的，目标是帮助你尽快看懂这个 `STM32F427XX + FreeRTOS + CMake` 模板工程。

建议按下面顺序阅读：

1. [01-工程总览与阅读顺序.md](F:/Code/ROBOCON/2026ROBOCON/Task/BSP+FreeRTOS/Template_Cmake/docs/training/01-%E5%B7%A5%E7%A8%8B%E6%80%BB%E8%A7%88%E4%B8%8E%E9%98%85%E8%AF%BB%E9%A1%BA%E5%BA%8F.md)
2. [02-分层、命名与目录结构.md](F:/Code/ROBOCON/2026ROBOCON/Task/BSP+FreeRTOS/Template_Cmake/docs/training/02-%E5%88%86%E5%B1%82%E3%80%81%E5%91%BD%E5%90%8D%E4%B8%8E%E7%9B%AE%E5%BD%95%E7%BB%93%E6%9E%84.md)
3. [03-驱动、回调与任务启动.md](F:/Code/ROBOCON/2026ROBOCON/Task/BSP+FreeRTOS/Template_Cmake/docs/training/03-%E9%A9%B1%E5%8A%A8%E3%80%81%E5%9B%9E%E8%B0%83%E4%B8%8E%E4%BB%BB%E5%8A%A1%E5%90%AF%E5%8A%A8.md)
4. [04-电机控制链路与示例任务.md](F:/Code/ROBOCON/2026ROBOCON/Task/BSP+FreeRTOS/Template_Cmake/docs/training/04-%E7%94%B5%E6%9C%BA%E6%8E%A7%E5%88%B6%E9%93%BE%E8%B7%AF%E4%B8%8E%E7%A4%BA%E4%BE%8B%E4%BB%BB%E5%8A%A1.md)
5. [05-底盘模块与开发工作流.md](F:/Code/ROBOCON/2026ROBOCON/Task/BSP+FreeRTOS/Template_Cmake/docs/training/05-%E5%BA%95%E7%9B%98%E6%A8%A1%E5%9D%97%E4%B8%8E%E5%BC%80%E5%8F%91%E5%B7%A5%E4%BD%9C%E6%B5%81.md)

补充说明：

- **这套模板主要用于 RoboCon 比赛固件开发。**
- `Protocol/` 目录当前主要放 **MAVLink 自动生成文件**，具体消息定义和生成方式请看战队飞书里的 MAVLink 文档。
- `Motor_DJI` 和 `Motor_DM` 下面的任务目前是 **示例任务**，用于演示接入和调用方式。
- 当前根目录的 [模板工程使用指南.md](F:/Code/ROBOCON/2026ROBOCON/Task/BSP+FreeRTOS/Template_Cmake/模板工程使用指南.md) 仍然保留，适合整体查阅；这组文档更偏向培训教材。
