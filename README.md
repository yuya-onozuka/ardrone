# ardrone  
## ardrone_autonomy
公式のardrone driver  
ROS Wiki：http://wiki.ros.org/ardrone_autonomy  
Documents:https://ardrone-autonomy.readthedocs.io/en/latest/  
### ドライバ起動
```
$ roslaunch ardrone_autonomy ardrone.lanuch
```

## tum_ardrone
ROS Wiki:https://wiki.ros.org/tum_ardrone
```
$ roslaunch tum_ardrone ardrone_driver.launch
$ roslaunch tum_ardrone tum_ardrone.launch
```

## ardrone_operator
* キーボード操作
* ArUco Markerへの追従
### ドライバ起動
```
$ roslaunch ardrone_autonomy ardrone.launch
```
### キーボード操作
```
$ roslaunch ardrone_operator ardrone_keyboard_operation.launch
```
### rosbag保存方法
- ardrone.launchを実行する必要なし。
filenameはアルファベットから始める。
```
$ roslaunch ardrone_operator ardrone_record.launch bagfile_name:=filename
```
### rosbag実行方法
- ardrone.launchは実行しない。
```
$ roslaunch ardrone_operator ardrone_play.launch bagfile_name:=filename
```
### マーカー追従制御
- ドライバー、キーボード操作、追従制御プログラムが立ち上がる。
- OpenCVのArUco Markerライブラリを使っているため、ArUco Markerが必要。
- [Online ArUco markers generator](https://www.google.com/search?q=aruco+create+marker&oq=aruco+create+&aqs=chrome.1.69i57j0l7.5733j0j7&sourceid=chrome&ie=UTF-8)
```
$ roslaunch ardrone_operator adrone_operator.launch
```
### 制御方法
1. ArUco Markerを検出し、マーカーまでの距離を導出
2. ArUco Markerの原点をカメラ画像に投影
3. ArUco Markerの原点がカメラ画像の中心にくるようにドローンの上下移動、ヨー回転を制御
4. ArUco Markerまでの距離を一定に保つようにドローンの前後移動を制御

### 参考サイト
- [ardrone_autonomy](https://ardrone-autonomy.readthedocs.io/en/latest/)  
ardrone_autonomyパッケージのドキュメント。
- [移動する物体を追跡する](https://cvtech.cc/tracking/)  
カメラを用いた物体追跡。
- [カメラキャリブレーションと3次元再構成](http://opencv.jp/opencv-2svn/cpp/camera_calibration_and_3d_reconstruction.html#cv-solvepnp)  
OpenCVによる画像認識を用いた座標変換。
- [ROS講座66 カメラのキャリブレーションを行う](https://qiita.com/srs/items/416aa78f2c679ddb7c52)  
ROSによるカメラのキャリブレーション。
- [OpenCV Mat と Eigen の相互変換](http://dronevisionml.blogspot.com/2015/07/opencv-mat-eigen.html)  
- [Detection of ArUco Markers](https://docs.opencv.org/3.2.0/d5/dae/tutorial_aruco_detection.html)  
ArUco Markerライブラリの使い方。
- [OpenCVarucoマーカ](https://seesaawiki.jp/asama-yaya/d/OpenCVaruco%A5%DE%A1%BC%A5%AB)  
ArUco Markerライブラリの使い方。
- [ArUco マーカーの検出](https://qiita.com/mkisono/items/cfdb9b74e41fae2f59d0)  
ArUco Markerライブラリの使い方。
- [Online ArUco markers generator](https://www.google.com/search?q=aruco+create+marker&oq=aruco+create+&aqs=chrome.1.69i57j0l7.5733j0j7&sourceid=chrome&ie=UTF-8)  
ArUco Markerを作る。
- [ROSを始めよう　その３](https://qiita.com/hagi-suke/items/9158a2770db65ea3d4d0)  
カメラを切り替えるためのclientを実装するときに参考にした。
- [Getting both camera images at the same time](https://forum.developer.parrot.com/t/getting-both-camera-images-at-the-same-time/676)  
ardroneで両方のカメラ画像を同時に取得するのは無理らしい。
- [Finger Detection and Tracking using OpenCV and Python](https://github.com/amarlearning/Finger-Detection-and-Tracking)  
- [The 20BN-jester Dataset V1](https://20bn.com/datasets/jester/v1)  
ハンドジェスチャーのデータセット