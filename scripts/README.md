# scripts

## ROS ノードスクリプト
|file name|node name|pub or sub|topic name|topic type|comment
|:---:|---|---|---|---|---
|get_food.py|get_food_node|sub|/food_coordinate|Pose|1. グリッパーを開く<br>2. `Enter` キー入力を待つ<br>3. グリッパーを閉じる<br>4. 机の上を上から俯瞰する<br>5. 目標の `Pose` をサブスクライブしたら、今の姿勢のまま、エンドエフェクタを x 軸方向に合わせてから y 軸方向に合わせる<br>6. 合わせたら z 軸方向を合わせる<br>7. z 軸方向を戻す<br>8. 一回の実行でノードを終了し、終了コード `0` を返す
|go_mouth.py|go_mouth_node|sub|/mediapipe_difference|Pose|1. 机の外側に向く<br>2. 目標の `Pose` をサブスクライブしたら、今の姿勢のまま、エンドエフェクタを y 軸方向に合わせてから z 軸方向に合わせる<br>3. 合わせたら x 軸方向を合わせる<br>4. x 軸方向を戻す<br>5. 一回の実行でノードを終了し、終了コード `0` を返す
|publish_mediapipe.py|publish_mediapipe|pub|/mediapipe_difference|Pose|1. MediaPipe を用いて顔を検出<br>2. 口の中心座標を取得<br>3. 目標の位置と姿勢を計算し、パブリッシュする
|publish_mediapipe.py|publish_mediapipe|pub|/mediapipe_difference|Pose|1. MediaPipe を用いて顔を検出<br>2. 口の中心座標を取得<br>3. 目標の位置と姿勢を計算し、パブリッシュする

### ライブラリ
|file name|import|feature
|:---:|---|---
|thrust.py|sys<br>math<br>moveit_commander<br>geometry_msgs.msg/Pose|1. 姿勢を維持したまま任意軸を移動するクラス `liner_movement`<br>2. エイムしてからグリッパーで突く動作ができるクラス `arm_thrust_node` は、インスタンス化する際に下向きに突くか、横向きに突くかを決めることが可能

## お悩み相談室

> サーチポジションが低い！

サーチポジションは `thrust.py` で定義されている。

`arm_thrust_node` クラスの `__init__` コンストラクタ内で、`string == "down"` の時が下向きとなっていて、`default_pose` に `[x.xxxx, 0, z.zzzz]` が代入されているため、`z.zzzz` を大きくすれば上に行く。

ただし、`x.xxxx` の値次第で、最大の `z.zzzz` は変化することに注意！

> `get_food.py` や `go_mouth.py` がサブスクライブできてるのか知りたい！

`my_stan/scripts/buckup/` に `give_food.py` や `give_mouth_py` がある。

`give_food.py` は、`/food_coordinate` トピックにテキトーな値をパブリッシュするもの。

`give_mouth.py` は、`/mediapipe_difference` トピックにテキトーな値をパブリッシュするもの。

これらを `rosrun` することで、`get_food.py` や `go_mouth.py` をテストすることが可能。



