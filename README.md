# twc-swipe-across-the-dishes-moveit-example
- This project provides MoveIt example to for [twc-swipe-across-the-dishes](https://github.com/HJS-HJS/twc_swipe_across_the_dishes) module.
- Use MoveIt to move m1013 doosan manipulator.
- Use MoveIt to check the path is valid.

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="./figure/1.moveit.png" width="320">
        <br><b>MoveIt! example code</b>
      </td>
      <td align="center">
        <img src="./figure/2.path_generated.png" width="320">
        <br><b>Generated swipe path with example pickle data</b>
      </td>
      <td align="center">
        <img src="./figure/3.move_along_path.png" width="320">
        <br><b>Move along the generated path in MoveIt</b>
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
    - [installation link](https://github.com/HJS-HJS/twc_swipe_across_the_dishes)
    - This code must be installed for example to work.

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
