# 2026-11-06 历史文件清理

文档重写补充：在下述108文件清理基础上，又移除了11份过时设计、预测规划、V1/DRAFT协议、AI流程和旧排障文档。重写README中英文、当前系统、接线表、AUTO说明、V2构建/对接与文档导航。第二份完整文档备份位于本机仓库同级`AerialRoboArm_backups/docs_rewrite_20261106/before_docs_rewrite.zip`，含重写前36个文件；旧资料仍可通过Git历史追溯。以下统计和验证记录属于首轮代码清理，第二轮仅修改文档。

清理前版本：`9611966b200b8b2f4a1ce15aa4f8b24d1705122e`。本次不改变Git历史或当前控制参数，不烧录、不触发实物动作。

## 删除范围

- 4份`jlink*.cfg`和2份`tools/flash_jlink*.ps1`，当前下载入口采用DAPLink。
- 已排除编译的`app_testbench`、`app_threads`，以及只剩注释的旧控制台测试源码和头文件；同步移除唯一残留头文件引用及CMake排除项。
- `User/archive`内旧ST3215驱动与相应失效测试。
- `Doc/archive`中的重复HAL/CMSIS驱动镜像、旧接口草案、旧方案记录及过时资产。
- 旧BLDC/AS5600系统设计、双线程架构、个人工作陈述、demo_v6测试台和过时CubeMX检查表。
- 根目录旧`COMMIT_MESSAGE.txt`。

合计移除108个受Git跟踪的文件。删除前已在仓库外创建ZIP及SHA-256清单，所有被修改的已有文件也已备份。实际备份位置记录在本机`outputs/legacy_cleanup_plan.json`，备份位于仓库同级`AerialRoboArm_backups/legacy_cleanup_.../`。旧文件也可从上述Git版本查回。

## 更新与保留

更新README中英文入口、Doc索引，并新增`design/CURRENT_SYSTEM.md`；保留原始来源、作者及第三方许可证。后续演进设计文档标明计划/历史性质，V2文档补充后续状态说明，避免把早期“尚未烧录”当作当前结论。

保留现用HAL/CMSIS/FreeRTOS、SEGGER RTT、DAPLink配置、HX8/ELRS/HC13/JY60/JY61P及测距路径。SEGGER RTT仍为DAPLink/OpenOCD提供日志与采集；I2C1用于VL53L1X。未按关键词删除这两项依赖。

保留FOC_DEMO2工程名、CubeMX/IDE文件及固件产物名称，避免破坏现有脚本。保留研究照片、实验原始记录、独立传感器工程、可用的JY60/诊断/采集选项和默认关闭的三帧夹爪选项。未清除本地IDE缓存和历史编译目录。

## 验证

- 正式控制以全新`cmake-build-maintenance-formal`配置并编译成功：RAM15048 B、FLASH54968 B。
- `tools/build_hc13_host.ps1`成功：RAM13776 B、FLASH35388 B。
- `tools/test_hc13_host.ps1`原生C协议测试通过。
- 上述内存占用与清理前记录一致。现有链接器RWX段警告仍存在，本次不处理链接策略。
- 当前源码、脚本和文档不再引用被删除的J-Link入口或旧测试台文件；新增文档本地链接检查通过。
- 编译和协议测试不能替代实机验证；本次未修改运动、故障、标定或速度设置。
