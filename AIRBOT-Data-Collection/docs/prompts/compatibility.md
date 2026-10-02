# Modules

- 请严格遵循 airdc/docs/prompts/prepare.md 中的要求！请将其中所有要求简单复述一遍以强化记忆。
- 实现完成后，请在 airdc/tests/modules/<模块类别小写复数> 目录下编写相应的配置文件，然后将路径传入 airdc/tests/modules/test_<模块类别小写复数>.py 脚本进行测试，确保测试通过。

## Sampler

请参考 airdc/docs/develop/modules.md中Sampler一节，分析将 /home/ghz/Work/lerobot/src/lerobot/scripts/lerobot_record.py 中的数据记录功能改写为DataSampler的子类是否可行，如果不可行，请说明需要在当前框架中增加哪些功能以支持该需求。如果可行，请依次完成如下步骤：
- 在 airdc/docs/manual/modules/samplers 目录下编写一个说明文档，其中：
  - 介绍lerobot的数据格式
  - 介绍lerobot原版的数据采集程序中对各类数据的采集逻辑（如高维图像数据、低维关节数据等）
  - 介绍计划实现的sampler的各个方法的实现逻辑
- 在 airdc/airdc/common/samplers 目录下创建 lerobot_sampler.py 文件并进行实现，并确保通过测试。并注意检查执行save保存后是否在预期位置生成了数据文件，以及执行remove操作后是否数据目录下相关文件删除干净。

一些提示：
- 示例payload可以在测试文件中找到
