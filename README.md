# ROMEA Cmd Mux

This stack provides a generic command mutiplexer implementation based on ideas found in [topic_tools mux](https://github.com/ros-tooling/topic_tools), [twist_mux](https://github.com/ros-teleop/twist_mux) or [cmd_vel_mux](https://github.com/kobuki-base/cmd_vel_mux) nodes.  It takes N input topics and outputs the messages from a single one according their priority and their acitivity. Finally, thanks to rclcpp's GenericSubscriber and GenericPublisher any kind of messages can be mutiplexed.   For more detailed information, please refer to the README files of each individual package.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://github.com/Romea/romea-ros2-cmd-mux/blob/main/romea_cmd_mux_public.repos
5. vcs import src < romea_cmd_mux.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to this project, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

This project is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Cmd Mux stack was developed by **Jean Laneurit** in the context of various research projects carried out at INRAE.

## **Contact**

If you have any questions or comments about Romea Cmd Mux stack, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 