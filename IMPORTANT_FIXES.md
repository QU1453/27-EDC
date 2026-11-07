# 重要问题修复记录

> 格式：时间 / 项目 / 现象 / 原因 / 改动 / 回滚。只记踩坑大、会复发的问题。

---

## 2026-10-24 · car_dipan · EIDE 看不到 BSP/Algorithm/App（第 3 次复发）

**现象**
EIDE 左侧工程树只有 `Application`、`Drivers`、`Hardware`；`bsp_motor.c` 不参与编译，`main.c` 报 `bsp_motor.h: No such file`。手改 `eide.yml` 后短暂生效，一操作 EIDE 又变回原样。

**原因**
1. EIDE 工程树完全由 `MDK-ARM/.eide/eide.yml` 的 `virtualFolder` 决定，不会自动扫描目录里的 `.c`——不登记就不显示、不编译。
2. `eide.yml` 会被两件事覆盖：① EIDE 面板里「重新导入 Keil 工程」，它按 `MDK-ARM/car_dipan.uvprojx` 重新生成，而 uvprojx 里只有 5 个 `<Group>`（`Application/MDK-ARM`、`Application/User/Core`、`Drivers/STM32G4xx_HAL_Driver`、`Drivers/CMSIS`、`Hardware`），唯一一处 `<IncludePath>` 也只有 `../Core/Inc;../Hardware`；② EIDE 进程内持有旧内存副本，重新加载/保存时把磁盘上的改动写回去。

**改动**（经用户授权后实施）
- `MDK-ARM/car_dipan.uvprojx`：`<Groups>` 末尾新增 `BSP`、`Algorithm`、`App` 三个 `<Group>`（共 5 个 `.c`）；唯一那处 `<IncludePath>` 改为 `../Core/Inc;../App;../BSP;../Algorithm;../Hardware;...`。只加分组与头文件路径，未改任何编译选项；`[xml]` 解析校验通过，共 8 组。
- `MDK-ARM/.eide/eide.yml`：`virtualFolder` 追加同样三组；`incList` 追加 `../App`、`../BSP`、`../Algorithm`。已读回确认落盘。

**怎么不再犯**
- uvprojx 已含三组，现在点「重新导入 Keil 工程」不会再丢分组（仍建议导入后扫一眼树）。
- 改完 yml 必须在 EIDE 里重载工程，并确认树里出现三组；再改一个 `.c` 保存触发构建，确认真的参与编译。
- 新增 `.c` 要同时登记两个地方：`car_dipan.uvprojx` 的 `<Group>` 和 `eide.yml` 的 `virtualFolder`。只登记 yml，重导入就丢。
- 若再出现「树里没有某组」，先查 uvprojx 有没有，再查 yml。

**回滚**
删掉 yml 里这三组和三条 `incList` 即回到原状。
