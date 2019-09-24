# ardrone  
## 実行方法
ドライバ起動
```
roslaunch ardrone_autonomy ardrone.launch
```
キーボード操作
```
roslaunch ardrone_operator ardrone_keyboard_operation.launch
```
マーカー追従制御
```
rosrun ardrone_operator adrone_follow_controller
```

## 参考サイト
- [移動する物体を追跡する](https://cvtech.cc/tracking/)
- [カメラキャリブレーションと3次元再構成](http://opencv.jp/opencv-2svn/cpp/camera_calibration_and_3d_reconstruction.html#cv-solvepnp)
- [OpenCV Mat と Eigen の相互変換](http://dronevisionml.blogspot.com/2015/07/opencv-mat-eigen.html)