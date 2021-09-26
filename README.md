# kcctcore

## auto navigation
```
#2d slam
roslaunch kcctcore navigation.launch sim:=true map_2d:=true
#simulationを行うなら下記のコマンドを別ターミナルで実行
roslaunch kcctsim sim.launch

#3d slam
roslaunch kcctcore navigation.launch sim:=true
#別ターミナルで一緒に実行
roslaunch kcctcore mcl_3dl.launch
```
* sim : gazevo simulation (bool)
* map_2d : 2dslam (bool)

![rosgraph](https://user-images.githubusercontent.com/38370926/134777815-eceb9985-c178-4d70-a401-cdee52b49079.png)
