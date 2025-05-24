# Lessons Learned

- Gazebo Sim Ackermann Steering library documentation: [AckermannSteering Class Reference](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html)

## Relevant Topics

- Odom topic:
    - Description: Topic on which the system will publish odometry messages.
    - Default name: /model/${model_name}/odometry
    - Type: gz.msgs.Odometry (equivalent to nav_msgs/msg/Odometry)
    - XML parameter to edit topic name: `<odom_topic>`
- Transformation topic:
    - Description: Topic on which the system will publish the transform from `frame_id` to `child_frame_id`.
    - Default name: /model/${model_name}/tf
    - Type: gz.msgs.Pose_V (equivalent to geometry_msgs/msg/PoseArray or tf2_msgs/msg/TFMessage)
    - XML parameter to edit topic name: `<tf_topic>`
- Command topic:
    - Description: Topic that the system will subscribe to in order to receive control messages.
    - Default name: /model/${model_name}/cmd_vel (if not steering only)
    - Type: gz.msgs.Twist (equivalent to geometry_msgs/msg/Twist Or geometry_msgs/msg/TwistStamped)
    - XML parameter to edit topic name: `<topic>`
- Steering angle topic:
    - Description: Topic that the system will subscribe to in order to receive control messages, if steering only is chosen.
    - Default name: /model/${model_name}/steer_angle
    - Type: gz.msgs.Double (equivalent to std_msgs/msg/Float64)
    - XML parameter to edit topic name: `<topic>`

## Parameters

- `<frame_id>`: The basis frame id for transformation (via corresponding topic) and odometry data (also via corresponding topic).
Default: ${model_name}/odom
- `<child_frame_id>`: Target frame id for transformation and odometry.
Default: ${model_name}/${link_name}
