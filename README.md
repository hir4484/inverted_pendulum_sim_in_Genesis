車輪型(2輪)倒立振子の製作ファイル(5)<br>
<br>
物理プラットフォーム "Genesis"の環境内で、倒立振子ロボットを動かしてみよう！<br>
Try moving an inverted pendulum robot in the environment of the physics platform "Genesis"<br>
にて解説で使った、シミュレーションファイルのリポジトリです。<br>
<br>
製作内容は、下記youtubeをご参照下さい。<br>
Please refer to the YouTube link below for details of the production.<br>
https://www.youtube.com/watch?v=l0ogELA2urQ<br>
![sample_pic1.png](./sample_pic1.png)<br>
<br>
ファイルの内容について：<br>
inv-pend_robo_PD_control.py<br>
　倒立振子の model file を、genesis の環境で倒立制御(PD制御)するための実行ファイルです。<br>
pendulum_robot_renew (folder)<br>
　このフォルダーのまま、下記記載の様に通り該当する階層に copy して使用してください。<br>
<br>
Genesis<br>
 └ genesis<br>
    └ assets<br>
       └ urdf<br>
          └ pendulum_robot_renew  # this URDF dir<br>
             ├ meshes  <br>
		     │  ├ xx1.dae   # my .dae files<br>
             │  └ xx2.dae<br>
             └ Robot.urdf   # my .urdf file<br>
<br>
Copyright (c) 2025/Aug/14, hir (hir4484@gmail.dom). Available under the MIT License. For more information, see LICENSE.
