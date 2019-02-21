# Robotic Arm : Pick and Place Project

[//]: # (Image References)

[image_0]: ./images/test_dataset.jpg
[image_1]: ./images/jointslinks.jpg
[image_2]: ./images/obs_map.jpg
[image_3]: ./images/diff_img.jpg
[image_4]: ./images/sim_conf.jpg
[image_5]: ./images/auto_run.jpg
--
### Kinematic Analysis
#### Explored Forward Kinematics demo and evaluated the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and to derive its DH parameters.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0      | 0     | 0.75 | q1
1->2 | - pi/2 | 0.35  | 0    | q2-pi/2
2->3 | 0      | 1.25  | 0    | q3
3->4 | -pi/2  | -0.054| 1.5  | q4
4->5 | pi/2   | 0     | 0    | q5
5->6 | -pi/2  | 0     | 0    | q6
6->EE| 0      | 0     | 0.303| 0



