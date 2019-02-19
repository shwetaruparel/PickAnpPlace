# Robotic Arm : Pick and Place Project

[//]: # (Image References)
[image_0]: ./images/test_dataset.jpg
[image_1]: ./images/Recorded_testdata.jpg
[image_2]: ./images/obs_map.jpg
[image_3]: ./images/diff_img.jpg
[image_4]: ./images/sim_conf.jpg
[image_5]: ./images/auto_run.jpg
----
### Kinematic Analysis
#### Explored Forward Kinematics demo and evaluated the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and to derive its DH parameters.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0



