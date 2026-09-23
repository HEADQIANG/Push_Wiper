# 随仓库提供的 Push-Wiper 训练数据

本目录包含正式导出的47个NPZ和原始manifest，训练39段/13任务、验证8段/3任务。
样本逐字节保留；原始MCAP、采集设备配置及PNG预览不属于此训练发布包。

数据指纹：`bc84492e63bafd3fb29ef8039d8cde7985e3f3686fc3f182bea8b57a7700918b`。

NPZ中的模型字段：`mask`为480×640二值图，污渍0/干净1；
`capture_reference_pose`为7维固定拍摄位姿；`actions`为16×3的绝对基座x、y和展开的delta_yaw；
`action_definition_version=2`。其他保留字段不会作为策略输入。

一段对应一个完整推动样本，按manifest保留原有任务划分，不将相邻段拼接成滑动窗口。
训练与使用方法见仓库根目录README及训练工程docs。
