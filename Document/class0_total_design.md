## SLAM系统思路

高博最后的大作业中，给出了这样的实现思路：

1. 你可以先实现一个 Frame-by-Frame 的里程计,即仅估计当前图像与上一个图像之间的运动, 然后把它们组成完整的相机轨迹。这样的工作方式应该是有效的,但误差会很快累积,导致轨迹发散。
2. 接下来,从轨迹中提取关键帧,对关键帧进行 bundle adjustment。由于 BA 过程重投影误差必定是下降的,可以有效的抑制误差累积。同时,你需要一个机制来管理所有地图点和关键帧。
3. 最后,请测试你的方案在 kitti等数据集上的表现, 例如轨迹精度, 运行时间等等。





搞了半天，我发现我在做大作业。