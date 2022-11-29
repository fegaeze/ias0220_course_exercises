### Important Notes
- The launch file has been set to build the robot structure from the xacro files directly. Entry file is robot.xacro.
- To debug your URDF file, do the following
```
- cd into your robot folder
- run this command to generate a urdf file: `rosrun xacro xacro robot.xacro > robot.urdf` (Note: xacro should be installed)
- use `check_urdf robot.urdf` to make sure your URDF can be successfully parsed (Optional)
- use `urdf_to_graphiz robot.urdf` to view your robot structure in pdf format (Optional)
- check that your urdf can be converted to sdf using this command: `gz sdf --print robot.urdf > robot.sdf`. Note: Gazebo uses sdf, so it if good to verify (Optional)
```