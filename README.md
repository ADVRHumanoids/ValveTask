**Valve Task**

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
XBotGUI
```


**GUI**
1) HomingExample->Start->Stop
2) ValveTask -> Start
3) Pub valve pose(for right hand):

```
./pubValvePoseWrtWorldOdom.py
```
           
4) press on button _success_ or _fail_ to make finite state machine transits to next state.