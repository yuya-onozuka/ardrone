# ardrone  

## Demo
### ArUco Marker follow control
ArUco Markerにドローンを追従させる制御。
1. 準備
- OpenCVのArUco Markerライブラリを使っているため、ArUco Markerを用意。
- [Online ArUco markers generator](https://www.google.com/search?q=aruco+create+marker&oq=aruco+create+&aqs=chrome.1.69i57j0l7.5733j0j7&sourceid=chrome&ie=UTF-8)
- ardrone_follow_controller_param.yamlでパラメータを設定（生成したArUco Markerのサイズを指定する必要がある）。

2. デモ用のlaunchファイルを起動
- ardrone driver、キーボード操作ノード、マーカー追従制御ノードを起動
```
$ roslaunch ardrone_operator ardrone_demo_follow.launch
```

### Auto Pilot
visual SLAMにより自己位置を認識できるようにし、その情報をもとに位置制御。
1. デモ用のlaunchファイルを起動
```
$ roslaunch ardrone_operator ardrone_demo_auto_pilot.launch
```
2. txtファイルの選択
* GUIのLoad Fileからtxtファイルを選択。
3. auto pilotの開始
GUI上の「Reset」→「Clear and Sebd」の順にボタンをクリックすると離陸し、PTAM(visual SLAM)のinitailizeを始める。initializeが成功すると自己位置を認識できるようになり、選択したtxtファイルのgotoコマンドの位置に動く。  
goto x y z yaw → initializeした位置、姿勢からの相対位置、姿勢を指定。

## Pacakges
### ardrone_autonomy
公式のardrone driver。  
ROS Wiki： http://wiki.ros.org/ardrone_autonomy  
Documents: https://ardrone-autonomy.readthedocs.io/en/latest/  

#### ドライバ起動
```
$ roslaunch ardrone_autonomy ardrone.lanuch
```

### tum_ardrone
visual SLAM(PTAM)で作ったマップ情報をもとに制御。  
ROS Wiki: https://wiki.ros.org/tum_ardrone  
ジェスチャーに応じた指令値を送るために少しプログラムを修正した。
```
$ roslaunch tum_ardrone ardrone_driver.launch
$ roslaunch tum_ardrone tum_ardrone.launch
```
ROS Wiki、gitのREADMEを参考にしてGUIを操作。

### ardrone_operator
* キーボード操作
* ArUco Markerへの追従
#### ドライバ起動
```
$ roslaunch ardrone_autonomy ardrone.launch
```
#### キーボード操作
```
$ roslaunch ardrone_autonomy ardrone.launch
$ roslaunch ardrone_operator ardrone_keyboard_operation.launch
```

#### マーカー追従制御
```
$ roslaunch ardrone_autonomy ardrone.launch
$ roslaunch ardrone_operator ardrone_follow_controller.launch
```

制御方法
1. ArUco Markerを検出し、マーカーまでの距離を導出
2. ArUco Markerの原点をカメラ画像に投影
3. ArUco Markerの原点がカメラ画像の中心にくるようにドローンの上下移動、ヨー回転を制御
4. ArUco Markerまでの距離を一定に保つようにドローンの前後移動を制御

#### モーション認識
```
$ roslaunch ardrone_operator receive_motion.launch
```

#### rosbag保存方法
- ardrone.launchを実行する必要なし。
filenameはアルファベットから始める。
```
$ roslaunch ardrone_operator ardrone_record.launch bagfile_name:=filename
```
#### rosbag実行方法
- ardrone.launchは実行しない。
```
$ roslaunch ardrone_operator ardrone_play.launch bagfile_name:=filename
```

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
- [厳密な重心の求め方](https://cvtech.cc/centroid/)  
赤色のマーカーのmotionを認識するために、重心の軌道を使った。

## Prerequisites
* Ubuntu16.04 
* ROS kinetic (ardrone_autonomy, tum_ardroneがkineticまでしか対応していない)
* OpenCV 3.3.0
