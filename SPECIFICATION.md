# micro-ROSファームウェア自動生成ツール 仕様書
- micro-ROSファームウェア自動生成ツールの仕様書
- [README.md](./README.md)の「環境構築」の完了が前提条件である

# ツールの使用方法
## 使用方法
1. 設定ファイル「uros_config.yml」に各種設定を記述
1. `micro-ROS_ASP3/uros_spike-rt_gen`で下記コマンドを実行してmicro-ROSパッケージ，及びカスタムメッセージ定義パッケージを生成
    -  `uros_spike-rt_gen/gen`に下記パッケージが生成される
        - micro-ROSパッケージ：spike_rt_uros
        - カスタムメッセージ定義パッケージ：spike_ros_msg
    - 実行コマンド
        ```
        python3 uros_gen.py
        ```
1. Prime HubをDFUモードにする
    - PCにUSBケーブルを接続する
    - Prime HubのBluetoothボタンを押しながらPCとの接続ケーブルをPrime Hubに挿す
    - Bluetoothボタンが「ピンク色に点灯→虹色に点滅」になるまでBluetoothボタンを押し続ける
1. micro-ROSファームウェアをビルド
    - spike_ros_msgを`micro-ROS_ASP3/external/primehub/firmware/mcu_ws`にコピー
    - micro-ROSライブラリをビルド
        ```
        cd ~/asp_uros_ws/micro-ROS_ASP3/external
        make build_firmware
        ```
1. micro-ROSパッケージをビルド，及びPrime Hubへの書き込み
    - ビルド・書き込みの前に**Raspberry Piでmicro-ROS Agentを起動**する
    - spike_rt_urosを`micro-ROS_ASP3/spike-rt`にコピー
    - 下記コマンドでビルド
        ```
        cd ~/asp_uros_ws/micro-ROS_ASP3/spike-rt
        make deploy-dfu
        ```
    - **並列ビルドが原因ででエラーになる事がある**
        - その場合は2回ビルドを行う
        ```
        cd ~/asp_uros_ws/micro-ROS_ASP3/spike-rt
        make asp.bin
        make deploy-dfu
        ```
    - Prime Hubにスマイルマークが表示されたら成功
1. ROS2アプリの開発
    - spike_ros_msgをROS2ワークスペースの`src`ディレクトリにコピーする
        ```
        cp ~/asp_uros_ws/micro-ROS_ASP3/uros_spike-rt_gen/gen/spike_ros_msg ~/ros2_ws/src
        ```
    - ROS2アプリを開発する

## オプション
- 自動生成時に`$ python3 uros_gen.py [option]`でオプションが利用可能

| コマンド | オプション内容 |
| :---:  | :---: |
| -c | 自走生成後に設定内容を表示する |
| -l | micro-ROSファームウェアのビルドを自動で行う</br> (上記手順の4番を自動で行う)|
| -u | micro-ROSパッケージのビルド・書き込みを自動で行う</br> (上記手順の6番を自動で行う)|
| -lu | -lと-uの処理をどちらも行う</br> (上記手順の4番と5番を自動で行う) |

# 設定ファイル（uros_config.yml）の記述方法
## デバイス情報の記述に関する注意
- 設定ファイルには各ポートに接続するデバイスの情報とHub内蔵モジュールに関数情報を記載する．
- 設定が記述されていない場合は**デフォルト値**が設定される
- デバイスを接続しないは何も記述しない
- インデントは必ず**半角スペース2つ**で行う
- 設定内容は順不同で構いません

## motorの設定内容

| 項目(Key) | 設定内容 | 設定値(Value) | デフォルト |
| :---:  | :---: | :---: | :---: |
| device | デバイス名 | motor | motor（必須） |
| qos | 速度司令，ブレーキ・リセット司令</br>（アプリ→uROS）のQoS | best-effort または reliable | best-effort |
| wise | モータの回転方向 | clock または counter-clock | clock |
| run_mode | モータの駆動方法 | set-power または set-speed | set-power |

## color sensorの設定内容

| 項目(Key) | 設定内容 | 設定値(Value) | デフォルト |
| :---:  | :---: | :---: | :---: |
| device | デバイス名 | color-sensor | color-sensor（必須） |
| qos | カラーセンサモード</br>（アプリ→uROS）の送信QoS | best-effort または reliable | best-effort |
| enable_lights | カラーセンサ内蔵ライト</br>を有効にするか | True または False | False |
| light_qos | ライト指令値のQoS | True または False | False |

## ultrasonic sensorの設定内容

| 項目(Key) | 設定内容 | 設定値(Value) | デフォルト |
| :---:  | :---: | :---: | :---: |
| device | デバイス名 | ultrasonic-sensor | ultrasonic-sensor（必須） |
| qos | 距離センサモード</br>（アプリ→uROS）の送信QoS | best-effort または reliable | best-effort |
| enable_lights | 距離センサ内蔵ライト</br>を有効にするか | True または False | False |
| light_qos | ライト指令値のQoS | True または False | False |

## force sensorの設定内容

| 項目(Key) | 設定内容 | 設定値(Value) | デフォルト |
| :---:  | :---: | :---: | :---: |
| device | デバイス名 | force-sensor | force-sensor（必須） |

## Hub内蔵モジュールの設定内容

| 項目(Key) | 設定内容 | 設定値(Value) | デフォルト |
| :---:  | :---: | :---: | :---: |
| hub_program_cycle | uROS送信データの送信周期(ms)</br>（トピック「spike_status」の送信周期） | 1以上の整数 | 100 |
| enable_imu | IMUを有効にするか | True または False | False |
| imu_mode | IMUのモード | temperature または gyro | temperature |
| enable_battery_management | Hubバッテリー情報の送信を有効にするか | True または False | False |
| enable_button | Hub内蔵ボタンを有効にするか | True または False | False |
| enable_speaker | Hub内蔵スピーカを有効にするか | True または False | False |
| enable_speaker | Hub内蔵スピーカの音量 | 0 ~ 100の整数 | 50 |
| opening | Hub起動時のメッセージ表示を有効にするか | True または False | False |

## uros_config.yml記述例
- 以下の条件のときの設定ファイルの記述例
    - PortA : 距離センサ
    - PortB : カラーセンサ
    - PortC : フォースセンサ
    - PortD : モータ
    - PortE : 接続デバイスなし
```
PortA:
  device: ultrasonic-sensor
  qos: best-effort
  enable_lights: true
  light_qos: best-effort

PortB:
  device: color-sensor
  qos: best-effort
  enable_lights: True
  light_qos: best-effort

PortC:
  device: force-sensor

PortD:
  device: motor
  qos: best-effort
  wise: clock
  run_mode: set-speed

hub:
  hub_program_cycle: 100
  enable_imu: true
  imu_mode: gyro
  enable_battery_management: true
  enable_button: True
  enable_speaker: true
  speaker_volume: 30
  opening: False 
```