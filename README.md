# twc-swipe-across-the-dishes-moveit-example
- This project provides MoveIt example to for [twc-swipe-across-the-dishes](https://github.com/HJS-HJS/twc_swipe_across_the_dishes) module.
- Use MoveIt to move m1013 doosan manipulator.
- Use MoveIt to check the path is valid.

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="./figure/mode_0.gif" width="320">
        <br><b>Mode 0: Initial State</b>
      </td>
    </tr>
  </table>
</div>

# environment
- ROS Noetic
- MoveIt!

# install
- Doosan m1013 driver
    - [installation link](https://github.com/doosan-robotics/doosan-robot)
- twc-swipe-across-the-dishes module
    - [installation link](https://github.com/HJS-HJS/twc_swipe_across_the_dishes) module

# Run example code
- launch MoveIt! example code
    - launch in virtual mode
        ```bash
        roslaunch twc_swipe_across_the_dishes_moveit_example real.launch mode:=virtual
        ```
    - if you connect to real robot
        ```bash
        roslaunch twc_swipe_across_the_dishes_moveit_example real.launch
        ```
- launch swipe server module
    ```bash
    twc roslaunch swipe_across_the_dishes server.launch
    ```
- launch swipe example code
    ```bash
    twc roslaunch swipe_across_the_dishes server.launch
    ```
