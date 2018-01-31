**Valve Task - How to guide**

**Set xbot config**
```
set_xbot_config /home/super/advr-superbuild/configs/ADVR_shared/user_example/walkman_valve_task.yaml

```


**Launch**
```
roslaunch walkman_gazebo walkman_valve_task.launch
```

```
NRTDeployer
```

```
taskset -c 0 XBotGUI
```


**GUI**
1) HomingExample->Start->Stop
2) ValveTask -> Start
3) Pub valve pose(for right hand):

```
./pubValvePoseWrtWorldOdom.py
```

Pub valve pose in Gazebo:

```
./advr-superbuild/external/ValveTask/python/pubValvePoseWrtWorldOdom.py
```

Pub valve pose in Gazebo:

```
./advr-superbuild/external/ValveTask/python/pubValvePoseWrtWorldOdom.py
```

Pub valve pose (for right hand):
```
rostopic pub /valve_pose geometry_msgs/PoseStamped "
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  pose:
    position:
      x: 0.8
      y: -0.4
      z: 1.40
    orientation:
      x: 0.0
      y: -0.7071070192004544
      z: 0.0
      w: 0.7071070192004544"
```
       
4) press on button _success_ or _fail_ to make finite state machine transits to next state.


**Note**
1) Stop any other plugins before starting this plugin.
2) By default, the robot use its right hand to do the job, so put you robot in a comfortable position for the operation!

