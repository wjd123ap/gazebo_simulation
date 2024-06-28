# robot cleaner

 gazebo simulation of robot cleaner 

## How to setup

You can install the project by entering the command below into the terminal:
```bash
mkdir robot_ws
cd ./robot_ws
git clone https://github.com/wjd123ap/robot_cleaner.git
catkin_make
```
## 주의사항

프로젝트를 실행하기 전에 다음 사항을 확인하고 수정해 주세요:

`/src/cleaner_simulation/worlds/simulation_env.world` 파일에서 다음 부분을 수정해야 합니다:

```xml
<mesh><uri>"{my_absolute_path}.stl"</uri></mesh>
```
이를 다음과 같이 변경하세요:
```xml
<mesh><uri>"{your_absolute_path}.stl"</uri></mesh>
```
