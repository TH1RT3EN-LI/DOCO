# AprilTag PNG 文件放置说明

## 文件命名规范

推荐使用 AprilTag 36h11 族（兼容 PX4/ROS2 apriltag_ros 包）：

    tag36_11_00000.png   → ID  0
    tag36_11_00001.png   → ID  1
    tag36_11_00002.png   → ID  2
    ...

## 下载方式

```bash
# 下载 36h11 全族 PNG（含 587 个 tag）
git clone https://github.com/AprilRobotics/apriltag-imgs.git /tmp/apriltag-imgs
cp /tmp/apriltag-imgs/tag36_11/*.png  <本目录>/
```

或只下载需要的几个，例如：
```bash
wget -O tag36_11_00000.png \
  https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36_11/tag36_11_00000.png
```

## 注意事项

- PNG 必须是正方形（黑色标记 + 白色边框），建议分辨率 ≥ 128×128
- 在 SDF 中引用路径为: model://apriltag/media/<文件名>.png
- 如果使用其他 tag 族（tag25h9、tag16h5 等），保证 ros2 detection 节点配置一致
