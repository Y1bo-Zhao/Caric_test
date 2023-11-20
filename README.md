# Caric_test
## ZYB
Two nodes were written for the gcs and two explorers\\
Node can run but can't fly well

    catkin build
    source devel/setup.bash
    roslaunch caric_baseline_test run.launch

  
##吕明阳：
写了个point cloud listener的文件，里面发了三个topic，一个是/target_point_cloud，是目标点。
一个是/position_point_cloud，是路径点，等同于目标点在法向量方向上位移三米得出的结果。
一个是/normal_point...呃...具体名字我好像忘了，不重要，这个就是个可视化的向量，起点是target point，终点是position point。
三米这个参数可以改，我的文件里能用的就只有一个cpp file，pointcloud2catcher,别用其他的。
具体看下面的函数，我的注释率百分之百，没注释的都不重要！
