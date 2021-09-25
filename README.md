# kcctcore

## auto navigation
```
#2d slam
roslaunch kcctcore navigation.launch sim:=true map_2d:=true

#3d slam
roslaunch kcctcore navigation.launch sim:=true
#別ターミナルで一緒に実行
roslaunch kcctcore mcl_3dl.launch
```
* sim : gazevo simulation (bool)
* map_2d : 2dslam (bool)