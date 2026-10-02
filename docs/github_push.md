# GitHub 代码推送与部署入口

本项目根目录对应 `HEADQIANG/Push_Wiper` 的 `main` 分支。
采集代码以普通文件纳入仓库，不是 Git 子模块。现有采集目录中的独立 `.git`
仅供本地使用，不会上传，也不要向其上游 `DISCOVER-Robotics` 推送本项目修改。

## 推送当前代码

在项目根目录执行：

```bash
git status --short
git add -u
git add README.md docs run_com.md Push-Wiper-Diffusion-Policy
```

如果采集目录保留独立 `.git`，逐文件暂存采集代码，避免误建子模块：

```bash
git -C AIRBOT-Data-Collection ls-files --cached --others --exclude-standard -z \
  | while IFS= read -r -d '' path; do
      if [ -f "AIRBOT-Data-Collection/$path" ] || [ -L "AIRBOT-Data-Collection/$path" ]; then
        git add -- "AIRBOT-Data-Collection/$path"
      fi
    done
```

普通克隆不包含内层 `.git`，此时直接 `git add AIRBOT-Data-Collection` 即可。
检查差异，确认没有密钥、虚拟环境、模型权重或原始采集数据，再提交：

```bash
git diff --cached --stat
git diff --cached --check
git commit -m "Update Push-Wiper collection and policy force control"
git push origin main
```

已有纳入版本管理的 47 个训练样本继续保留；其他本地采集数据、缓存、虚拟环境和
训练输出不上传。`checkpoints/best.ckpt` 不在 Git 中，运行模型推理前需自行训练
或准备权重，不能只凭克隆代码直接运行模型或真机。

## 运行步骤

- 4090 训练：在根目录执行 `bash Push-Wiper-Diffusion-Policy/scripts/train_4090.sh`；
  安装、恢复与推理参数见 `Push-Wiper-Diffusion-Policy/docs/usage.md`。
- AIRBOT 环境安装：见 `AIRBOT-Data-Collection/README.md` 和该工程的 `docs/setup/`。
- 力位混合控制：见 `AIRBOT-Data-Collection/docs/setup/force_hybrid_control.md`。
- 在线模型与力控：见 `AIRBOT-Data-Collection/docs/setup/push_wiper_policy_force.md`；
  先准备权重，执行配置验证和离线检查，再按文档连接硬件。

本次发布不改变现有启动命令；文档中的本机绝对路径应替换为实际克隆路径。
