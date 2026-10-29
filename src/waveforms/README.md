# ENS060XCD 波形阶段字典

ENS060XCD 使用共享阶段字典减少固件体积。每个阶段仍包含完整的 64 字节驱动矩阵，16 位索引只合并完全相同的数据。阶段数量、顺序、重复次数、时长、模式和温区不变；索引在构造 LUT 时查询一次，不逐像素解压，不在启动时展开到 PSRAM。

## 格式与兼容

`EpdWaveformPhases.phase_indices` 为空时，`luts` 仍表示原连续阶段数组，旧波形的聚合初始化保持有效。非空时，`luts` 指向共享字典，`phase_indices[frame]` 选择对应的 64 字节阶段。全部 LUT 构造路径通过 `epd_waveform_lut()` 取址。

当前 ENS060XCD 包含 54 张实际存储表、3573 个存储阶段，合并为 1635 个唯一阶段与 7146 字节索引。模式及温区的已有整表共享继续保留。理论阶段资源减少 116886 字节，最终收益以固件链接结果为准。

## 重新生成

从原 WBF 对应的 JSON 生成时，继续校验生成器固定的 WBF SHA256：

```sh
python scripts/generate_ens060xcd.py waveform.json src/waveforms/epdiy_ENS060XCD.h --wbf waveform.wbf
clang-format --style=file -i src/waveforms/epdiy_ENS060XCD.h
```

默认输出字典；添加 `--raw` 可导出原连续格式。没有原 WBF 时，仅允许从已有的原始连续头文件无损迁移，不接受已转换的字典头：

```sh
python scripts/generate_ens060xcd.py original.h src/waveforms/epdiy_ENS060XCD.h --from-header
clang-format --style=file -i src/waveforms/epdiy_ENS060XCD.h
```

迁移检查表长度、字节范围、描述符阶段数量与 16 位字典容量。不要手工修改字典或索引。

## 验证

宿主硬件项目根目录提供以下检查。独立参考必须使用转换前保存的原始头文件：

```sh
python test/waveform_generator_test.py
node test/run_waveform_dictionary_tests.mjs --reference-header build/waveform-size-comparison/reference.h
python test/run_epdiy_thread_safety_tests.py
```

对照覆盖 8 个模式、14 个温区、6469 个逻辑阶段的原始字节、阶段时长和元数据，并比较 1K、64K、S3 及固定起始灰度的 LUT 输出，启用地址和未定义行为检查。

主机测试不能判断人眼可见的残影、闪烁或灰阶观感，仍需在实机上观察。固件不包含设备波形自测入口或测量日志。
