# SAO

Shimada, Ashihara and Osawa Project

## Usage

### examples/pong_example.py

1. Component をインスタンス化
Random: ランダムに動く学習器（？）
Const : いずれか1つの行動のみをする学習器（？）

2. Environment をインスタンス化
ならず、Environmentが訓練させるComponentを渡す
学習状況を可視化する場合はrender=Trueを引数に。デフォルトはFalse

3. Environment を介して実行・訓練
env.execute()で、1エピソード実行


### examples/pong_planner_example.py

手続きは、0.1と同様

複数のComponentをまとめたCircuitを利用する例
Circuitをインスタンス化する場合には3種類の方法がある。

1. デフォルトコンストラクタを呼び出す: 引数：Circuitに持たせるすべてのComponent

2. Circuit.create()を呼び出し: 引数：Circuitのハイパーパラメータ

3. Circuit.load()を呼び出し: 引数：保存名



## Environment

## Component

## Circuit

## TODO:

1. PropLearnerを賢くする
2. 標準的な学習機を、現在のNohに合わせて記述する
3. Noh-Gazebo連携
4. プレゼンを作る


