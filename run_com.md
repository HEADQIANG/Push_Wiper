# 1. 启动机械臂
```bash
sudo env -u LD_LIBRARY_PATH /bin/bash /usr/local/bin/airbot_server   -i can_follow -p 50051
```

# 2.力控复现npz轨迹
```bash
scripts/start_force_hybrid.sh run
```

# 3. 拖拽机械臂实时查看位姿曲线
```bash
cd /home/wp/yuelk_project/Push_Wiper/AIRBOT-Data-Collection
source install/activate_airdc.sh

scripts/start_force_hybrid.sh drag-preview --preview-hz 30
```