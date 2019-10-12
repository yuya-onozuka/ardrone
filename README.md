# ardrone  
## 実行方法
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
- ドライバー、キーボード操作、追従制御プログラムを立ち上げる。
```
$ roslaunch ardrone_operator adrone_operator.launch
```

## 参考サイト
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