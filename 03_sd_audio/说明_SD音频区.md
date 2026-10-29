# SD 卡 · 音频区说明

## 卡上布局（绝对地址，不要改）

| 段 | 起始绝对 LBA | 扇区数 | 内容 |
|---|---|---|---|
| BMP | 由文件系统分配 | — | 12 张图，按文件名 01→12 |
| **alarm** | **131072** | **471** | 应急音（16 kHz / 单声道 / 16 bit PCM 小端） |
| **ring** | **131543** | **249** | 普通音 |
| 合计音频区 | 131072 | **720** | = `audio_sd.bin` 的完整内容（368 640 字节） |

`audio_sd.bin` 就是这个 720 扇区的连续镜像：

```
SHA256  05A5995EC59CE13ED049E32B1233B7C50D23912E9814FB011326C6EA113C6BE4
大小    368640 字节 = 720 × 512
```

## 对应的 RTL 参数（`src\audio_sd_stream.v`）

```verilog
parameter ALARM_BASE_SECTOR = 32'd131072,
parameter ALARM_SECTORS     = 32'd471,
parameter RING_BASE_SECTOR  = 32'd131543,
parameter RING_SECTORS      = 32'd249
```

**换音频文件就必须同时改这四个数**，并且保证 `ALARM_BASE + ALARM_SECTORS == RING_BASE`、
两者总和不越过 BMP 的区域。

## 怎么写进去

```powershell
# 管理员权限运行（写的是卷句柄 \\.\X:，需要管理员）
powershell -ExecutionPolicy Bypass -File .\write_audio.ps1 -Drive E
```

注意（脚本里已经处理好，改脚本时别破坏）：

1. `\\.\E:` 是**卷句柄**，偏移要从**分区起点**算：
   `$OFFSET = ($TARGET_SECTOR - $partStart) * $SECTOR`
2. 写完会**自动清零音频区之后的 1328 个扇区**（131792…133119），
   把老版本残留的录音清掉——不清的话，地址一旦算错就会放出旧音频
3. 脚本会核对 `$EXPECT_SHA256`，不一致就报错退出

## 换图 / 换音频之后

- **图片**：整套重拷（`格式化 → 全套拷 12 张`），并把 `top_audio_sd.v` 里的
  `.SCAN_TARGET_COUNT (4'd12)` 改成实际张数
- **音频**：只动音频区的话，图片不用重拷
- **卡在电脑上动过之后，板子必须断电重上电**（按 KEY1 复位没用）
