# robot cleaner

 gazebo simulation of robot cleaner 

## How to setup

You can install the project by entering the command below into the terminal:
```bash
git clone https://github.com/wjd123ap/gazebo_simulation.git
cd ./gazebo_simulation
catkin_make
```
## How to execute 
```bash
roslaunch cleaner_simulation simulation_run.launch
```
Turn on the new terminal and give the next command to apply torque to the wheels.
```bash
rostopic pub /wheelvel_control cleaner_simulation/WheelVel "left: 3.0
right: 3.0" 
```



토픽값으로 바퀴 토크가 바뀜.

## precautions

프로젝트를 실행하기 전에 다음 사항을 확인하고 수정해 주세요:

`/src/cleaner_simulation/worlds/simulation_env.world` 파일에서 다음 부분을 수정해야 합니다:

```xml
<mesh><uri>"{my_absolute_path}.stl"</uri></mesh>
```
이를 다음과 같이 변경하세요:
```xml
<mesh><uri>"{your_absolute_path}.stl"</uri></mesh>
```
`~/.bashrc`에
```
source ~/robot_ws/devel/setup.bash
```
추가하세요.

gazebo 가 error 발생하면
```
killall -9 gzserver gzclient
```

