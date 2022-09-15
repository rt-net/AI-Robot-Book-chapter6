# チャレンジ問題
書籍「ROS 2とPythonで作って学ぶAIロボット入門」の第六章チャレンジ問題とミニプロジェクト6.2のプログラムを作成してみました。
## challenge6_134.py
challenge6_134.pyはチャレンジ問題6.1、6.3、6.4のプログラムをcommander1.pyに追加したものです。
## challenge6_2.py
challenge6_2.pyはチャレンジ問題6.2のプログラムをcommander2.pyに追加したものです。
## challenge6_5.py
challenge6_5.pyチャレンジ問題6.5のプログラムをcommander5.pyに追加したものです。
## challenge6_6_miniproject6_2.py
challenge6_6_minipro6_2.pyチャレンジ問題6.6とミニプロジェクト6.2のプログラムをcommander6.pyに追加したものです。

# 実行手順
```
#CRANE+ V2　起動
$ ros2 launch crane_plus_commander crane_plus_control_rsp.launch.py

---以下は各プログラムの実行コマンド---
#challenge6.1,3,4
$ ros2 run crane_plus_commander challenge6_134

#challenge6.2
$ ros2 run crane_plus_commander challenge6_2

#challenge6.5
$ ros2 run crane_plus_commander challenge6_5

#challenge6.6 miniproject6.2
$ ros2 run crane_plus_commander challenge6_6_minipro6_2```
