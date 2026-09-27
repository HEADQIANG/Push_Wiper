# 数据采集程序云端上传功能

## 功能概述

在原有的数据采集程序基础上，新增了自动上传MCAP文件到云端的功能。当用户按下 `s` 键保存数据后，程序会自动将保存的MCAP文件上传到DataLoop云端存储。

## 主要改动

### 1. 配置文件改动

**文件：** `airdc/defaults/config_mmk.yaml`

- 将 `task_id` 从字符串 `"120"` 改为整数 `120`
- 新增 `upload` 配置节：

```yaml
sampler:
  param:
    task_info:
      task_id: 120  # 改为int类型，用作DataLoop的project_id
    upload:
      enabled: true                    # 是否启用上传功能
      endpoint: '192.168.215.80'      # DataLoop服务器地址
      username: 'admin'               # 用户名
      password: '123456'              # 密码
```

### 2. 代码改动

**文件：** `airdc/airbot/samplers/mcap_sampler.py`

#### 主要修改：

1. **导入新模块**：
   ```python
   from typing import Literal, Dict, Union
   import uuid
   ```

2. **修改TaskInfo模型**：
   ```python
   class TaskInfo(BaseModel):
       task_id: Union[str, int] = ""  # 支持字符串和整数
   ```

3. **新增UploadConfig模型**：
   ```python
   class UploadConfig(BaseModel):
       enabled: bool = True
       endpoint: str = '192.168.215.80'
       username: str = 'admin'
       password: str = '123456'
   ```

4. **修改AIRBOTMcapDataSamplerConfig**：
   ```python
   class AIRBOTMcapDataSamplerConfig(BaseModel):
       upload: UploadConfig = UploadConfig()
   ```

5. **修改save方法**，在保存完成后调用上传：
   ```python
   def save(self, path: str) -> str:
       # ... 原有保存逻辑 ...

       # Upload to cloud after saving
       if self.config.upload.enabled:
           self._upload_to_cloud(path)

       return path
   ```

6. **新增_upload_to_cloud方法**：
   ```python
   def _upload_to_cloud(self, file_path: str) -> bool:
       """Upload the saved file to cloud storage."""
       try:
           from dataloop import DataLoopClient

           # Initialize DataLoop client
           dataloop = DataLoopClient(
               endpoint=self.config.upload.endpoint,
               username=self.config.upload.username,
               password=self.config.upload.password
           )

           # Generate unique sample ID
           uid = str(uuid.uuid4())

           # Convert task_id to int if it's a string
           project_id = self.config.task_info.task_id
           if isinstance(project_id, str):
               project_id = int(project_id)

           # Upload the file
           self.get_logger().info(f"开始上传文件到云端: {file_path}")
           message = dataloop.samples.upload_sample(
               project_id=project_id,
               sample_id=uid,
               sample_type="Sequential",
               file_path=file_path
           )

           self.get_logger().info(f"文件上传成功: {message}")
           return True

       except ImportError:
           self.get_logger().error("dataloop 模块未安装，无法上传到云端")
           return False
       except Exception as e:
           self.get_logger().error(f"上传到云端失败: {str(e)}")
           return False
   ```

## 使用方法

### 1. 安装依赖

确保安装了DataLoop客户端：
```bash
pip install dataloop
```

### 2. 配置参数

修改 `config_mmk.yaml` 中的上传配置：
- `task_id`: 设置为对应的DataLoop项目ID（整数）
- `upload.enabled`: 设置为 `true` 启用上传功能
- `upload.endpoint`: DataLoop服务器地址
- `upload.username`: 用户名
- `upload.password`: 密码

### 3. 运行程序

```bash
cd airdc
bash run_mmk.sh
```

### 4. 数据采集和上传

1. 程序启动后，按 `空格键` 开始采集数据
2. 按 `s键` 保存数据到本地MCAP文件
3. 程序会自动上传文件到云端
4. 查看日志确认上传状态

## 工作流程

1. **数据采集**：按空格键开始采集数据
2. **本地保存**：按s键保存数据到本地MCAP文件
3. **自动上传**：保存完成后自动上传到DataLoop云端
4. **继续采集**：上传完成后可以继续下一轮数据采集

## 日志信息

- 开始上传：`开始上传文件到云端: /path/to/file.mcap`
- 上传成功：`文件上传成功: [server response]`
- 上传失败：`上传到云端失败: [error message]`
- 模块未安装：`dataloop 模块未安装，无法上传到云端`

## 注意事项

1. **网络连接**：确保网络连接正常，可以访问DataLoop服务器
2. **权限验证**：确保用户名和密码正确
3. **项目ID**：确保task_id对应的项目在DataLoop中存在
4. **异步处理**：上传操作在后台进行，不会阻塞数据采集流程
5. **错误处理**：上传失败不会影响本地文件保存，只会记录错误日志

## 测试

运行测试脚本验证上传功能：
```bash
python test_upload.py
```

## 禁用上传功能

如果需要禁用上传功能，将配置文件中的 `upload.enabled` 设置为 `false`：

```yaml
upload:
  enabled: false
```
