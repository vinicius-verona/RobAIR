# RobAIR

## Source code for patrolling robot - IMAG

> NOTE: Explanation of each node and how to execute them is provided in the **`Report.pdf`** file.
> A [**`scprit`**](./patrol_robot_development/scripts/) directory is provided where we have examples of how to execute the nodes.

Below you can find an example of how the scripts work the only difference is that for the GIF below, it only prints a message with the command, it does not execute the command as by the time of writting this file, I had no access to the robot.

> PS-1: All scripts will open multiple terminal windows (not tabs) and place them in different position within the screen and different workspaces.
> PS-2: Some packages are required to execute the script, in case your ubuntu does not have it, it will install both **`xdotool`** and **`wmctrl`**

**Executing the patrol script as an example** - None of the script execute the RVIZ and load the map. That must be done manually or editting the script (just follow the pattern and everything should work just fine).

![Execution of patrol script](./Example/execution-example.gif)
