# micro-ROSファームウェア自動生成ツール 仕様書
- micro-ROSファームウェア自動生成ツールの仕様書
- [README.md](./README.md)の「環境構築」の完了が前提条件である

# ツールの使用方法
## 本ツールによる自動生成からROS2アプリ開発までの流れ
1. 設定ファイル「uros_config.yml」に各種設定を記述
1. `micro-ROS_ASP3/uros_spike-rt_gen`で下記コマンドを実行してmicro-ROSパッケージ，及びカスタムメッセージ定義パッケージを生成
    -  `uros_spike-rt_gen/gen`にフォルダが生成される
    - `uros_spike-rt_gen/gen/[パッケージ名]`
      - このフォルダの中にmicro-ROSパッケージとメッセージ定義パッケージが生成される
      - メッセージ定義パッケージは「[micor-ROSパッケージ名]_msg」で生成される
    - 実行コマンド
        ```
        python3 uros_gen.py [options]
        ```
1. micro-ROSファームウェアをビルド
    - **使用するメッセージ定義パッケージに変更が加えられた場合のみ必要**
    - 生成したメッセージ定義パッケージを`micro-ROS_ASP3/external/primehub/firmware/mcu_ws`にコピー
      - メッセージ定義パッケージ名のデフォルトはspike_rt_uros_msg
    - `micro-ROS_ASP3/micro_ros_asp/micro_ros_asp.mk`に下記を追加
      - ```INCLUDES += -I$(MIROROS_ASP3_TOP_DIR)/$(MICROROS_INC)/[メッセージ定義パッケージ名]```
    - micro-ROSライブラリをビルド
        ```
        cd ~/asp_uros_ws/micro-ROS_ASP3/external
        make build_firmware
        ```
1. Prime HubをDFUモードにする
    - PCにUSBケーブルを接続する
    - Prime HubのBluetoothボタンを押しながらPCとの接続ケーブルをPrime Hubに挿す
    - Bluetoothボタンが「ピンク色に点灯→虹色に点滅」になるまでBluetoothボタンを押し続ける
1. micro-ROSパッケージをビルド，及びPrime Hubへの書き込み
    - ビルド・書き込みの前に**micro-ROS Agentを起動**する
    - 生成したmicro-ROSパッケージを`micro-ROS_ASP3/spike-rt`にコピー
      - micro-ROSパッケージ名のデフォルトはspike_rt_uros
    - 下記コマンドでビルド
        ```
        cd ~/asp_uros_ws/micro-ROS_ASP3/spike-rt/[micro-ROSパッケージ名]
        make deploy-dfu
        ```
    - **並列ビルドが原因ででエラーになる事がある**
        - その場合は2回ビルドを行う
        ```
        cd ~/asp_uros_ws/micro-ROS_ASP3/spike-rt/[micro-ROSパッケージ名]
        make asp.bin
        make deploy-dfu
        ```
    - Hunの電源投入後，Prime Hubにスマイルマークが表示されたら成功
1. ROS2アプリの開発準備
    - 生成したメッセージ定義パッケージをROS2ワークスペースの`src`ディレクトリにコピーする
1. ROS2アプリを開発する

## オプション
- 前述の使用手順を自動化するオプション等を用意
- 自動生成時に`$ python3 uros_gen.py [option]`でオプションが利用可能

| コマンド | オプション内容 |
| :---:  | :---: |
| -h | ヘルプコマンド</br>オプションの使用方法を表示|
| -setup | コンフィギュレーションファイル（uros_config.yml）テンプレートの生成|
| -n [name] | micro-ROSパッケージとメッセージ定義パッケージに名前をつける</br>(uROSパッケージが[name], msg_defパッケージが[name]_msg) </br>複数種類のアプリを開発する場合はそれぞれに固有の名前が必要</br>指定無しの場合はデフォルト名を使用|
| -c | 自動生成後に設定内容を表示する |
| -l | micro-ROSファームウェアのビルドを自動で行う</br> (上記手順の3番を自動で行う)</br>同一名のパッケージが存在する場合は既存のものを削除する|
| -u | micro-ROSパッケージのビルド・書き込みを自動で行う</br> (上記手順の5番を自動で行う)</br>同一名のパッケージが存在する場合は既存のものを削除する|
| -lu | -lと-uの処理をどちらも行う</br> (上記手順の3番と5番を自動で行う) |
| -mc | msg_defパッケージを生成後，ros2_wsにコピー </br> (上記手順の6番を自動で行う)</br>同一名のパッケージが存在する場合は既存のものを削除する|
| -py [name] | ros2_wsにROS2のPythonパッケージを生成</br>同一名のパッケージが存在する場合は既存のものを削除する |
| -cpp [name] | ros2_wsにROS2のC++パッケージを生成</br>同一名のパッケージが存在する場合は既存のものを削除する |
| -f | 本ツールがファイル削除する時に確認メッセージを表示しない |

# 設定ファイル（uros_config.yml）の記述方法
## デバイス情報の記述に関する注意
- 設定ファイルには各ポートに接続するデバイスの情報とHub内蔵モジュールに関数情報を記載する．
- 設定が記述されていない場合は**デフォルト値**が設定される
- デバイスを接続しないポートには何も記述しない
- インデントは必ず**半角スペース2つ**で行う
- 設定内容は順不同で可
- [sample](./sample/)を参考にする

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

# トピックの扱い方
- ToDo
- [sample](./sample/)を参考にする



# 注意
## 送信QoS
- ROSは送信QoSとしてbest-effort通信またはreliable通信を利用できる．
  - reliable通信：ACKを返す
  - bsst-effort通信：ACKを返さない

- 周期送信するメッセージにreliable通信を使用するとメッセージをドロップすることがある
  - 短い周期で周期送信するメッセージにreliable通信を使用するとACK処理が負荷となり，データをドロップする
  - 特にmicro-ROSが送信側のメッセージにreliable通信を使用するとドロップしやすい
    - micro-ROSはシングルタスクで稼働している
    - ACK待ちの優先度はサブスクライバの実行優先度よりも高い
      - ACK待ちの間はサブスクライバが実行されない
    - ACK待ちの間に複数のデータを受信した場合は受信データをドロップする

# 付録
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