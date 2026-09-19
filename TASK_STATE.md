# 长任务状态记录 (TASK_STATE.md)

## 任务目标
彻底修复上位机 foc-studio 中【电机参数辨识】和【零点校准】被 25ms 空闲定时器错误提前截断，导致彩虹进度条闪退收起、尚未转动就误报完成的问题。

## 验收标准
1. [x] 参数辨识（`ident` / `ident full`）执行期间，彩虹进度条平稳推进，直到单片机真正打印 `ident DONE:`、`ident FAIL:`、`err:` 或总超时才结束；（证据：capture-serial.mjs 测试用例验证通过，main.js 针对 ident 定制 checkEnd 并禁用 25ms 提前空闲截断）
2. [x] 零点校准（`calib` / `calib full`）执行期间，彩虹进度条平稳推进，直到单片机真正完成校准（单片机打印 `calib DONE`、`calib FAIL` 或状态切回 IDLE/FAULT）才结束；（证据：foc_calib.c 增加完成与失败打印，main.js 针对 calib 定制 checkEnd 并测试通过）
3. [x] 若辨识或校准失败（打印 FAIL / err），上位机进度条显示失败（红标），不再盲目报完成；（证据：wizard.js 中增加对 res.includes("FAIL") / res.includes("err:") 的异常分支判定，调用 prog.fail）
4. [x] 常规短查询命令（`status`、`conf read`、`vbus`、`limit`、`vel`、`pos` 等）依然保持毫秒级快速响应（25ms 空闲截断正常生效，无任何卡顿倒退）；（证据：capture-serial.mjs 中 status 命令在 <80ms 内快速完成）
5. [x] 单片机固件编译 0 Error 0 Warning；（证据：UV4 编译输出 0 Error(s), 0 Warning(s)，且成功烧录）
6. [x] 上位机测试套件 100% 通过。（证据：npm test 全部 19 个测试套件通过，0 failed）

## 进度记录
- **已验证**：
  - 根因完全确认：`sendCapture` 内部对所有命令一律采用 25ms 空闲定时器截断，单片机启动回显第一行后进入物理准备期（无字符），导致上位机在第 25ms 即误判完成并调用 `prog.done()`。
  - 单片机端已完善：`foc_calib.c` 成功时打印 `M0 calib DONE offset=...`，失败时打印 `M0 calib FAIL fault=...`，与 `foc_ident.c` 的 `ident DONE:` / `ident FAIL:` 规范对齐；
  - 上位机 `sendCapture` 已重构：识别 `ident`、`calib` 等长异步任务，禁用 25ms 空闲截断，仅凭明确结束标记（`DONE`/`FAIL`/`err`）或总超时结束；
  - 上位机向导已完善：`_runMotorIdent` 与 `_runMotorCalib` 支持长超时（12s），进度条真实伴随电机运转，辨识失败时红标提示，成功时回填参数；
  - 全部自动化测试与固件编译烧录均已通过。
