1. 第二列的hsv有问题
## Test
0111：
    9-11：绿色一层第一次move_to_center光环境变化->ok
    14-16：ok
    18-20：ok

# Ques
2. 抓上方是走的回程的拐点，不是抓偏
3. 三层回来会最后一程可能碰到一层货架，以离货架最远的为准。
速度：
    100
        1. go back pre_point
        2. go home2
    80:
        1.shift_camera_1-1,shift_camera_robot04
        2.utils.shift_arm(my_pose,80.0) depth
4. 目标普遍接近右爪，单独用相对位移由于机械臂和货架在相反方向不好调整，建议给右边松爪
5. 一层的摆动对光线影响较大，需要比较宽的范围
6. 方块紧挨铝边，反射出同样的颜色。(还没出过问题)

# Modify:
0111_bake:ok


# Target
周末之前四个颜色，通过column4和column2的测试。


# need to solve：Ques3,4,6

