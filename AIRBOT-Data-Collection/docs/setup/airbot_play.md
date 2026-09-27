# AIRBOT Play/PTK/TOK 安装指南

## Python SDK

该部分与`AIRBOT Play`官方手册中的操作一致，若尚未完成，可参考下方链接点亮，但需关注如下版本要求：

- **Python** : >= 3.10
- **AIRBOT Play Python SDK** : >= 5.1.4
- **AIRBOT Play Pro Python SDK** : >= 5.1.4

安装说明： [AIRBOT Play 驱动程序 和 Python SDK](http://docs.qiuzhi.tech/0.2.7/airbot-play/quick-start/software-setup.html)

对于`AIRBOT TOK`,额外安装：`pip install requests`用于底盘数据采集。

## 机械臂绑定

为了避免混淆示教臂和执行臂，需要按设备实际使用的角色绑定`CAN`名称。

### 设备连接

首先将全部机械臂的数据线从电脑上拔除，然后按如下顺序连接，分如下几种情况：

- 单臂控制单臂：示教臂、执行臂
- 双臂控制双臂：左侧示教臂、左侧执行臂、右侧示教臂、右侧执行臂的顺序依次连接
- 单臂拖动示教：单臂无先后顺序
- 双臂拖动示教：左侧臂、右侧臂

然后可选择使用自动或手动绑定。

### 自动绑定

除拖动示教外，下述绑定操作目前已在后续[AIRBOT Play/PTK/TOK 数据采集配置](../configure/airbot_play.md)部分默认自动进行，因此这里可直接跳过绑定步骤。

### 手动绑定

连接好后，然后可执行如下命令进行绑定（终端在`data-collection`目录），同样分两种情况：

- 单臂控制单臂
  ```bash
  sudo bash ./airbot_ie/scripts/bind_can_udev.sh --target can_lead can_follow
  ```
- 双臂控制双臂
  ```bash
  sudo bash ./airbot_ie/scripts/bind_can_udev.sh --target can_left_lead can_left can_right_lead can_right
  ```
- 单臂拖动示教（通常可以不绑定直接使用can0）
  ```bash
  sudo bash ./airbot_ie/scripts/bind_can_udev.sh --target can_follow
  ```
- 双臂拖动示教
  ```bash
  sudo bash ./airbot_ie/scripts/bind_can_udev.sh --target can_left can_right
  ```

命令执行结束后，再次将全部机械臂的数据线从电脑上拔除，然后重新连接，顺序无要求。连接好后，执行如下命令检查是否成功绑定：

```bash
ip l |grep can_
```

如果输出中包含绑定命令中`--target`后指定的全部名称，则绑定成功，例如双臂控双臂情况下应能看到：
`can_left_lead`, `can_left`, `can_right_lead`, `can_right`。

### 重新绑定

由于各种原因需要重新对设备进行绑定时，需首先执行如下命令清除之前的绑定配置：

```bash
sudo bind_airbot_device rm
```

然后再重新按顺序连接设备后再次执行绑定命令即可。

## 底盘连接

对于`AIRBOT TOK`，如需底盘功能，需通过WIFI或网线连接电脑，具体请参考相关文档（推荐使用有线连接，无线方式延迟高且不稳定）。
