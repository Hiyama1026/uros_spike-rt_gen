# micro-ROSファームウェア　自動生成ツール

# 本ツールについて
LEGO® Education SPIKE™ Prime向けmicro-ROSファームウェアの自動生成ツール．</Br>
Prime Hub向けソフトウェアプラットホームである[spike-rt](https://github.com/spike-rt/spike-rt)と，ASP3カーネル向けmicro-ROSミドルウェアである[micro-ROS_ASP3](https://github.com/exshonda/micro-ROS_ASP3)を使用する．

# 特徴
- Hubの各ポートに接続するデバイスの情報等をコンフィギュレーションファイル(Yaml)を記入することでmicro-ROSファームウェアを自動生成
    - micro-ROSパッケージとカスタムメッセージ型定義パッケージを生成
- 本ツールの利用により，ユーザはmicro-ROSの通信相手であるROS2アプリ等の開発に専念できる

# 構成
- SPIKR Prime HubとRaspberry Piを使用する
    - 両者はシリアル(UART)により接続する
- Prime Hub上でmicro-ROSプログラムを稼働しRaspberry Pi上でmicro-ROS Agentを稼働する
    - Prime Hub向けのmicro-ROSプログラムを本ツールが自動生成する
    - 生成されたmicro-ROSプログラムはPrime Hubに接続されたセンサの値の取得・publishや，アクチュエータへの指令値のsubscribe・出力などを行う
    - ユーザはmicro-ROSプログラムと通信を行うROS2アプリを開発する
        - ROS2アプリはmicro-ROS Agentを実行するRaspberry Pi上や外部の汎用PCで開発

# 動作確認済みバージョン
- spike-rt (GitHubコミット番号) : [f6724115b0ef8c8367a760eaec2840089e6b4e55](https://github.com/spike-rt/spike-rt/tree/f6724115b0ef8c8367a760eaec2840089e6b4e55)
- micro-ROS_ASP3 (GitHubコミット番号) : [3a306729a797d0f4976daab50c5698acffe38a12](https://github.com/exshonda/micro-ROS_ASP3/tree/3a306729a797d0f4976daab50c5698acffe38a12)
- Python : 3.10.12

# 動作環境
- ターゲット
    - ハードウェア：Prime hub (STM32F413)
    - OS：SPIKE-RT
- ホストPC（micro-ROSパッケージのビルド・ROS2アプリ開発に使用）
    - OS : Ubuntu20.04 LTSまたはUbuntu22.04 LTS
        <!--
        - 理論上はRaspberry Pi（Raspberry Pi OS 64bit）でも可能\
        - ただしmicro-ROSライブラリのビルド等に処理能力を要する為推奨はしない\
        - Raspberry Piでのmicro-ROSビルドの動作確認は未実施\
        - Raspberry PiでのROS2アプリ開発は可能であることを確認済み\
        -->
- Raspberry Pi (micro-ROS Agentの実行に使用)
    - ハードウェア：Raspberry Pi (Raspberry Pi 4のみ動作確認済み)
    - OS：Raspberry Pi OS (64bit)

- その他
    - Prime HubとRaspberry PiはUARTにより接続する
        - 詳細は後述の「[環境構築](#環境構築)」を参考

# 本ツールの使用方法
- 本ドキュメントの「[環境構築](#環境構築)」に従い環境を構築する
- 仕様書「[SPECIFICATION.md](./SPECIFICATION.md)」に従いツールを使用する

# 環境構築
## spike-rtとmicro-ROS_ASP3をインストール
- micro-ROSファームウェアのビルドを行うPC(Linux)上でワークスペースを作成
    ```
    mkdir ~/asp_uros_ws
    ```
- spike-rtとmicro-ROS_ASP3をインストール
    - クローン
    ```
    cd ~/asp_uros_ws 
    git clone https://github.com/spike-rt/spike-rt.git
    git clone https://github.com/exshonda/micro-ROS_ASP3.git
    ```
    - 動作確認済みバージョンへのコミット移動
    ```
    cd spike-rt
    git checkout f6724115b0ef8c8367a760eaec2840089e6b4e55
    cd ../micro-ROS_ASP3
    git checkout 3a306729a797d0f4976daab50c5698acffe38a12
    ```
- spike-rtとmicro-ROS_ASP3をセットアップ
    - それぞれを下記を参考にセットアップ
        - [spike-rt](https://github.com/spike-rt/spike-rt)
        - [micro-ROS_ASP3](https://github.com/exshonda/micro-ROS_ASP3)
- micro-ROS_ASP3でPrime Hub向けの設定を行う
    - [micro-ROS_ASP3/spike-rt/README.md](https://github.com/exshonda/micro-ROS_ASP3/blob/master/spike-rt/README.md)を参考にspike-rt対応のセットアップを行う
    - `micro-ROS_ASP3/Makefile.config`のターゲットボードをPrime Hubを選択する
    - `micro-ROS_ASP3/micro_ros_asp/micro_ros_asp.mk`のINCLUDESに下記を追加
        ```
        INCLUDES += -I$(MIROROS_ASP3_TOP_DIR)/$(MICROROS_INC)/spike_ros_msg
        ```

## 本ツールのインストール
- micro-ROS_ASP3の直下にインストール
    ```
    cd ~/asp_uros_ws/micro-ROS_ASP3
    git clone git@github.com:Hiyama1026/uros_spike-rt_gen.git
    ```

## rasberryPi側の環境構築
### rasberryPi OS (**64bit**)をインストール
- [インストラー](https://www.raspberrypi.com/software/)をインストール
- インストラーからrasberryPi OS(64bit)をインストール
    - ROSを動かすために64bit版をインストールする

### GPIOの接続を有効にする
1. 下記のコマンドで設定ファイルを開く
    ```bash
    sudo nano /boot/config.txt
    ```

1. config.txtの最後に下記を追加
    ```bash
    dtoverlay=uart5
    ```
    
1. リブートする
    ```bash
    sudo reboot
    ```

1. シリアル通信のケーブルをrasberryPiに接続する
    - 参考：[RasPike](https://github.com/ETrobocon/RasPike/wiki/connect_raspi_spike)

### エージェントのビルドと実行（方法1，2のどちらでも可）
#### エージェントのビルドと実行:方法1
1. 参考

- 下記の記事を参考にで`Micro-XRCE-DDS-Agent`をビルドする<BR>
<https://qiita.com/lutecia16v/items/5760551dd3a7a0d3e7d3>

1. `Micro-XRCE-DDS-Agent`のコードをクローン

    ``` bash
    cd ~
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    ```

1. ビルド

    ```bash
    cd Micro-XRCE-DDS-Agent
    mkdir build && cd build
    cmake -DTHIRDPARTY=ON ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
    ```

1. 実行

- `verbose_level`を6に設定して、メッセージの受信を表示するようにする
    - 2つ目のエージェントの実行コマンドは`sudo`が必要な場合がある
    - device はSPIKEと接続されているポート名(/dev/ttyXX)を指定
    ```bash
    source /opt/ros/humble/setup.bash
    MicroXRCEAgent serial --dev [device] -v 6
    ```
    - (例)シリアルの接続方法が[RasPike](https://github.com/ETrobocon/RasPike/wiki/connect_raspi_spike)と同じ場合
        ```
        source /opt/ros/humble/setup.bash
        MicroXRCEAgent serial --dev /dev/ttyAMA1 -v 6
        ```

#### エージェントのビルドと実行:方法2
1. エージェントのビルド
    ```bash
    cd uros_ws    
    ros2 run micro_ros_setup create_agent_ws.sh
    colcon build
    source install/local_setup.bash
    ```

1. エージェントの実行
    - device はSPIKEと接続されているポート名(/dev/ttyXX)を指定
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
    ```    

## ROS2のインストール（ROS2アプリの開発に必要）
### ROS2アプリをUbuntuPCで開発する場合（推奨）
- 公式サイトを参考にしてROS2パッケージをインストール
    - [ここ](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)など
- 以下は[上記サイト](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)を参考にしたUbuntu PCにROS2 Humbleをインストールする手順
1. ロケールを設定
    ```
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```

1. ROS2 aptのインストール
    ```
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    ```

1. GPGキーの設定
    ```
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```

1. リポジトリをソースリストに追加
    ```
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

1. aptのアップデート
    ```
    sudo apt update
    ```

1. ツールのインストール
    ```
    sudo apt install ros-humble-desktop
    ```
    ```
    sudo apt install ros-humble-ros-base
    ```
    ```
    sudo apt install ros-dev-tools
    ```

1. セットアップの実行（起動時毎に必要なためbashの設定ファイルに記載しておくと良い）
    ```
    # Replace ".bash" with your shell if you're not using bash
    # Possible values are: setup.bash, setup.sh, setup.zsh
    source /opt/ros/humble/setup.bash
    ```

1. テストの実行
    ```
    ros2 run demo_nodes_cpp talker
    ```
    - 下記のように表示されれば成功
        ```
        [INFO] [1700732992.424835597] [talker]: Publishing: 'Hello World: 1'
        [INFO] [1700732993.424739244] [talker]: Publishing: 'Hello World: 2'
        [INFO] [1700732994.424762644] [talker]: Publishing: 'Hello World: 3'
        [INFO] [1700732995.424767342] [talker]: Publishing: 'Hello World: 4'
        ...
        ```

1. ROS2のワークスペースを作成
    ```
    mkdir ~/ros2_ws
    cd ~/ros2_ws
    mkdir src
    colcon build
    . install/setup.bash
    ```
    - 以降ROS2パッケージは`ros2_ws/src`に作成する
    - パッケージを作成したらビルドとセットアップを行う
        ```
        cd ~/ros2_ws
        colcon build
        . install/setup.bash
        ```

### ROS2アプリをその他OSの汎用PC（Windows，RHEL，macOS）で開発する場合
- [ここ](https://docs.ros.org/en/humble/Installation.html)などのROS2公式サイトを参考にインストールする

### ROS2アプリをRaspberry Pi上で開発する場合
- 後述の付録「[ROS2のRaspberry Piへのインストール方法](#ros2のraspberry-piへのインストール方法)」を参照
<!--
1. **Cパッケージのビルド時にエラーになる場合の解決方法**
    - Cパッケージ(raspike_uros_msg)のビルドでエラーになる事がある
    - その場合は`/opt/ros/humble/opt`にある`libcurl_vendor`フォルダを削除する
        ```bash
        cd /opt/ros/humble/opt
        sudo rm -rf libcurl_vendor
        cd ~/ros2_ws
        colcon build
        ```
-->
## Prime HubとRaspberry Piを接続
- 前述の手順でRaspberry Piに接続したシリアルケーブルをPrime Hubの**ポートF**に接続する


## 付録
### ROS2のRaspberry Piへのインストール方法
1. アップデート
    ```bash
    sudo apt update
    sudo apt -y upgrade 
    ```

1. ROS2パッケージをインストールする
    ```bash
    wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bullseye/ros-humble-desktop-0.3.1_arm64.deb
    sudo apt install -y ./ros-humble-desktop-0.3.1_arm64.deb
    source /opt/ros/humble/setup.bash
    ```

1. ビルドツールのインストール
    ```bash
    sudo pip install vcstool colcon-common-extensions
    ```

1. ROS環境の自動読み込み設定
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

1. 動作確認
    ```bash
    ros2 launch demo_nodes_cpp talker_listener.launch.py
    ```
    - 下記のように出力されれば成功
        ```
        [INFO] [launch]: All log files can be found below /home/hiyama/.ros/log/2023-07-04-14-52-42-067659-raspi-1582
        [INFO] [launch]: Default logging verbosity is set to INFO
        [INFO] [talker-1]: process started with pid [1583]
        [INFO] [listener-2]: process started with pid [1585]
        [talker-1] [INFO] [1688449964.564641073] [talker]: Publishing: 'Hello World: 1'
        [listener-2] [INFO] [1688449964.565626925] [listener]: I heard: [Hello World: 1]
        [talker-1] [INFO] [1688449965.564647258] [talker]: Publishing: 'Hello World: 2'
        [listener-2] [INFO] [1688449965.565234628] [listener]: I heard: [Hello World: 2]
        ...
        ```
1. ROS2用のワークスペースを作成する
    ```bash
    mkdir ~/ros2_ws
    cd ~/ros2_ws
    mkdir src
    ```

1. ROS2パッケージをビルドする
    ```bash
    cd ~/ros2_ws
    colcon build
    . install/setup.bash
    ```