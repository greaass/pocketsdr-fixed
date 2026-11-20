# 上游基线与修改记录

## 已确认的来源

- 上游仓库：https://github.com/tomojitakasu/PocketSDR
- 本地版本名称：0.21-fixed。
- 本地接收机源码当前基线：`eda8301`。
- 基线提交说明：`Update the documents for v0.21: add the v0.21 history to README, reflect the carrier NCO phase initialization (GitHub issue #97) in the algorithm descriptions, update sdr_corr_std()/sdr_corr_std2() in the API reference, and regenerate the PDFs.`
- 本地源码包含未提交修改；基线提交号不代表修改版源码的完整身份。
- 本仓库初始化日期：2026-11-20。

## 首次源码提交时补齐

1. 保存上游完整提交号、修改版源码文件清单及 SHA-256。
2. 分别说明接收机、导航解码、定位和构建方面的修改。
3. 保留相关第三方源码的原许可证，注明各自来源和修改。
4. 给出构建依赖、配置示例及已执行的验证入口。
5. 核对源码与论文实验版本的对应关系，再设置发布标签。

本文件只记录来源，不将源码目录中后续改动自动视为已用于论文实验的版本。
