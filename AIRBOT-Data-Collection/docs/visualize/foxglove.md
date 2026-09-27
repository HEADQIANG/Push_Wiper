# Foxglove 可视化使用说明

网页链接：https://app.foxglove.dev/361de8be/dashboard。

## 支持类型

支持查看各种MCAP格式的数据：

- 消息数据：支持`FlatBuffers`、`ROS`等多种Schema的消息数据查看
- 附件数据：不支持直接在线查看，但可以下载后本地查看
- 元信息数据：支持查看MCAP文件中的元信息

## 话题信息

查看帧率和帧数：

<p align="center">
    <img src="../assets/visualize/image.png">
</p>

## 可视化深度图

<p align="center">
    <img src="../assets/visualize/image-1.png" width="900">
    <img src="../assets/visualize/image-2.png" width="900">
</p>

## 可视化彩色视频

### 方法一：网页端下载视频

由于视频默认保存在`attachment`中，无法直接可视化，需要下载后本地查看。可以通过[recordings](https://app.foxglove.dev/361de8be/recordings/list)上传数据：

<p align="center">
    <img src="../assets/visualize/image-3.png" width="900">
</p>

然后下载视频查看：
<p align="center">
    <img src="../assets/visualize/image-4.png" width="900">
</p>


### 方法二：使用 mcap 命令行工具导出视频

<a id="mcap_cli"></a>
#### 安装 MCAP CLI

可以基于`mcap`命令行工具将视频导出为本地文件。首先执行如下命令安装`mcap`命令行工具：

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
brew install mcap
```

#### 导出视频文件

然后执行如下命令导出视频文件：

```bash
mcap get attachment <name>.mcap -n "<attachment_name>" -o <output_name>.mp4
```
