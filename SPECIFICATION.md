# micro-ROSファームウェア自動生成ツール 仕様書
- micro-ROSファームウェア自動生成ツールの仕様書
- [README.md](./README.md)の「環境構築」の完了が前提条件である

# 目次
- [ツールの使用方法](#ツールの使用方法)
- [設定ファイル（uros_config.yml）の記述方法](#設定ファイルuros_configymlの記述方法)
- [トピックの扱い方](#トピックの扱い方)
- [センサモード](#センサモード)
- [カラーコード](#カラーコード)
- [注意](#注意)
- [付録](#付録)
  - [uros_configyml記述例](#uros_configyml記述例)
  - [ROS 2でトピックのQoSを指定する方法（Python）](#ros-2でトピックのqosを指定する方法python)

<br>
<br>
<br>

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
        python3 generate_uros.py [options]
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
      make
      make deploy-dfu
      ```
      - **(注意) asp.binの作成は``make``→``make asp.bin``で行う**
        ```sh
        make  # 必要
        make asp.bin    # または make deploy-dfu
        ```
    - Hunの電源投入後，Prime Hubにスマイルマークが表示されたら成功
1. ROS2アプリの開発準備
    - 生成したメッセージ定義パッケージをROS2ワークスペースの`src`ディレクトリにコピーする
1. ROS2アプリを開発する

## オプション
- 前述の使用手順を自動化するオプション等を用意
- 自動生成時に`$ python3 generate_uros.py [option]`でオプションが利用可能

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

<br>
<br>

# 設定ファイル（uros_config.yml）の記述方法
## デバイス情報の記述に関する注意
- 設定ファイルには各ポートに接続するデバイスの情報とHub内蔵モジュールに関数情報を記載する．
- 設定が記述されていない場合は**デフォルト値**が設定される
- デバイスを接続しないポートには何も記述しない
- インデントは必ず**半角スペース2つ**で行う
- 設定内容は順不同で可
- [sample](./sample/)を参考にする
- 設定ファイルの記述例は後述の[付録](#uros_configyml記述例)にも示している

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

<br>
<br>

# トピックの扱い方
(※)[sample](./sample/)を参考にすると良い

- PUPデバイス（センサモータなど）に対する情報を格納するトピックのトピック名は，格納するデータの内容とデバイスの接続ポートから名付けられる
  - (例)ポートAに接続したモータへの速度指令値を格納したトピックのトピック名→**motorA_speed**
  - 以降は **\[PORT\]** には接続ポートを表す**大文字**のアルファベットが入るものとする
    - 上記の例では，motor[PORT]_speedであり，[PORT]=Aとなる

- カスタムメッセージ型を使用したトピックの変数名も同様に，格納するデータの内容とデバイスの接続ポートから付与される
  - (例)トピック「spike_status」に含まれる，ポートAに接続したモータのエンコーダ値を格納する変数の名前→**motor_a_count**
    - 以降は **\[port\]** には接続ポートを表す**小文字**のアルファベットが入るものとする
      - 上記の例では，motor_[port]_countとなり，[port]=aとなる

- 各デバイスに対するトピックの名前や，含まれる変数の内容等の関係を以下に示す
## ROS2→uROS（ユーザがパブリッシュ）のトピック
### モータ
| トピック名 | カスタムメッセージ型名 | 変数名 |変数型| 変数に格納されるデータの内容 |
| ---  | --- | --- | --- | --- |
|motor[PORT]_speed|Int16(標準メッセージ型)|data|int16|[PORT]に接続されたモータに対する回転速度指令値|
|motor[PORT]_stop_reset|MotorStopReset|motor_stop_hold|int8|[PORT]に接続されたモータに対する停止指令値<br>0→操作無し，1→stop，2→hold|
|(同上)|(同上)|motor_reset|bool|[PORT]に接続されたモータに対するエンコーダ値リセット指令<br>True→エンコーダ値リセット|

### カラーセンサ
| トピック名 | カスタムメッセージ型名 | 変数名 |変数型| 変数に格納されるデータの内容 |
| ---  | --- | --- | --- | --- |
|color_sensor[PORT]_mode|Int8(標準メッセージ型)|data|int8|[PORT]に接続されたカラーセンサに対するモード（後述）指令値|
|color_sensor[PORT]_light|ColorLightMessage|light1|int8|[PORT]に接続されたカラーセンサの内蔵ライト1に対する輝度指令値(0~100)<br>ライトを有効化した時のみ使用|
|(同上)|(同上)|light2|int8|[PORT]に接続されたカラーセンサの内蔵ライト2に対する輝度指令値(0~100)<br>ライトを有効化した時のみ使用|
|(同上)|(同上)|light3|int8|[PORT]に接続されたカラーセンサの内蔵ライト3に対する輝度指令値(0~100)<br>ライトを有効化した時のみ使用|

### 距離センサ
| トピック名 | カスタムメッセージ型名 | 変数名 |変数型| 変数に格納されるデータの内容 |
| ---  | --- | --- | --- | --- |
|ultrasonic[PORT]_mode|Int8(標準メッセージ型)|data|int8|[PORT]に接続されたカラーセンサに対するモード（後述）指令値|
|ultrasonic_sensor[PORT]_light|UltrasonicLightMessage|light1|int8|[PORT]に接続された距離センサの内蔵ライト1に対する輝度指令値(0~100)<br>ライトを有効化した時のみ使用|
|(同上)|(同上)|light2|int8|[PORT]に接続された距離センサの内蔵ライト2に対する輝度指令値(0~100)<br>ライトを有効化した時のみ使用|
|(同上)|(同上)|light3|int8|[PORT]に接続された距離センサの内蔵ライト3に対する輝度指令値(0~100)<br>ライトを有効化した時のみ使用|
|(同上)|(同上)|light4|int8|[PORT]に接続された距離センサの内蔵ライト4に対する輝度指令値(0~100)<br>ライトを有効化した時のみ使用|

### スピーカ
| トピック名 | カスタムメッセージ型名 | 変数名 |変数型| 変数に格納されるデータの内容 |
| ---  | --- | --- | --- | --- |
|speaker_tone|SpeakerMessage|tone|int16|スピーカの周波数の指令値|
|(同上)|(同上)|duration|int16|音を鳴らす時間(ms)|

### IMU
| トピック名 | メッセージ型名 | 変数名 |変数型| 変数に格納されるデータの内容 |
| ---  | --- | --- | --- | --- |
|imu_init|Bool(標準メッセージ型)|data|bool|IMUの初期化指令(Trueで初期化)|


## uROS→ROS2(ユーザがサブスクライブ)のトピック
### spike_status
- このトピックにはHubに接続されたセンサのセンサ値等が格納される
- 従って，デバイスの接続状況の変化に応じて格納される変数も変化する
- 送信周期は設定ファイルで指定可能
- QoSは**best-effort**

| トピック名 | カスタムメッセージ型名 |
| ---  | --- |
|spike_status|SpikePrimeMessage|

| 対象デバイス | 変数名 | 変数型 | 変数に格納されるデータの内容 |
| --- | --- | --- | --- |
|モータ|motor_[port]_count|int32|[port]に接続されたモータのエンコーダ値を格納する|
|カラーセンサ|color_[port]_mode_id|int8|[port]に接続されたカラーセンサの現在のモードを格納|
|カラーセンサ|color_[port]_sensor_value_1|int16|[port]に接続されたカラーセンサのセンサ値1を格納|
|カラーセンサ|color_[port]_sensor_value_2|int16|[port]に接続されたカラーセンサのセンサ値2を格納|
|カラーセンサ|color_[port]_sensor_value_3|int16|[port]に接続されたカラーセンサのセンサ値3を格納|
|距離センサ|ultrasonic_[port]_mode_id|int16|[port]に接続された距離センサのセンサモード|
|距離センサ|ultrasonic_[port]_mode_id|int16|[port]に接続された距離センサのセンサ値を格納|
|フォースセンサ|force_[port]_sensor_pressed|bool|[port]に接続されたフォースセンサが押されているかを格納|
|フォースセンサ|force_[port]_sensor_force|bool|[port]に接続されたフォースセンサが押されている力(N)を格納|
|IMU|temperature|int8|IMUで取得した温度を格納<br>IMUモードがtemperatureの時に使用|
|IMU|x_angular_velocity|float32|X軸方向の各加速度を格納<br>IMUモードがgyroの時に使用|
|IMU|y_angular_velocity|float32|Y軸方向の各加速度を格納<br>IMUモードがgyroの時に使用|
|IMU|z_angular_velocity|float32|Z軸方向の各加速度を格納<br>IMUモードがgyroの時に使用|

### spike_power_status
- このトピックはbattery_managementを有効化した場合に使用される
- 送信周期は10秒
- QoSは**best-effort**

| トピック名 | カスタムメッセージ型名 |
| ---  | --- |
|spike_power_status|SpikePowerStatusMessage|

| 変数名 | 変数型 | 変数に格納されるデータの内容 |
| --- | --- | --- |
|voltage|int16|Hubバッテリーの電圧|
|current|int16|Hubバッテリーの電流|

### spike_button_status
- このトピックはHub内蔵ボタンの押下情報を格納
- 押下状態が変化した場合に送信
- QoSは**best-effort**
- メッセージ型は標準メッセージ型

| トピック名 | メッセージ型名 |
| ---  | --- |
|spike_button_status|Int8|

| 変数名 | 変数型 | 変数に格納されるデータの内容 |
| --- | --- | --- |
|data|int8|Hub内蔵ボタンの押下情報|

<br>
<br>

# センサモード
- カラーセンサセンサと距離センサはモードを切り替えると取得値の種類が変化する
    - カラーセンサ：全4モード
    - 距離センサ：全2モード
- センサモードの切り替えには若干の時間がかかるため，切り替わりが完了したことをユーザ側で確認する必要がある
    - トピック「spike_status」に含まれる下記の変数が，それぞれ現在のセンサモードを格納している
        - カラーセンサ：color_[port]_mode_id
        - 距離センサ：ultrasonic_[port]_mode_id

- 各モードの内容とカスタムメッセージ型の変数に格納される情報の詳細を以下に示す
    - カラーセンサ

    |モード|取得データ|メッセージ型とデータ内容の関係|
    |---|---|---|
    |0|無し（初期状態）|---|
    |1|ambient値|send_color_value_1=ambient値<br>send_color_value_2=0<br>send_color_value_3=0|
    |2|カラーコード（後述）|send_color_value_1=カラーコード<br>send_color_value_2=0<br>send_color_value_3=0|
    |3|reflection値|send_color_value_1=reflection値<br>send_color_value_2=0<br>send_color_value_3=0|
    |4|RGB値|send_color_value_1=R<br>send_color_value_2=G<br>send_color_value_3=B|
    |5|HSV値|send_color_value_1=H<br>send_color_value_2=S<br>send_color_value_3=V|

    - 距離センサ

    |モード|取得データ|メッセージ型とデータ内容の関係|
    |---|---|---|
    |0|無し（初期状態）|---|    
    |1|距離|ultrasonic_sensor=距離|    
    |2|presemce値|ultrasonic_sensor=presemce値|

<br>
<br>

# カラーコード
- カラーコードの対応表を以下に示す

|color code|color|
|---|---|
|0|NONE|
|1|RED|
|2|YELLOW|
|3|GREEN|
|4|BLUE|
|5|WHILE|
|6|BRACK|
|-2|err|

<br>
<br>

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

<br>
<br>

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

## ROS 2でトピックのQoSを指定する方法（Python）
- QoSの指定はパブリッシャやサブスクライバの初期化関数で行う
### best-effort
- QoSProfileをインポートする
  ```
  from rclpy.qos import QoSProfile
  ```
- 下記の例のようにreliabilityを指定し，パブリッシャ・サブスクライバを初期化
  ```
  def __init__(self):
    super().__init__("[node_name]")

    qos_profile = QoSProfile(depth=10, reliability=2)

    # パブリッシャ
    create_publisher([message_type], "[topic_name]", qos_profile)
    # サブスクライバ
    create_subscription([message_type], "[topic_name]", [コールバック], qos_profile)
  ```

### reliable
- 下記の例のようにパブリッシャ・サブスクライバを初期化
  ```
  # パブリッシャ
  create_publisher([message_type], "[topic_name]", 10)
  # サブスクライバ
  create_subscription([message_type], "[topic_name]", [コールバック], 10)
  ```