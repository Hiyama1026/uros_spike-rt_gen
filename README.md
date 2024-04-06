# micro-ROSファームウェア　自動生成ツール

# 本ツールについて
- LEGO® Education SPIKE™ PrimeのHub上で実行可能なmicro-ROSファームウェアの自動生成ツール．
- SPIKE Prime Hub向けのソフトウェアプラットホームである[spike-rt](https://github.com/spike-rt/spike-rt)と，ASP3カーネル向けmicro-ROSミドルウェアである[micro-ROS_ASP3](https://github.com/exshonda/micro-ROS_ASP3)を使用する．

# 特徴
- Hubの各ポートに接続するデバイスの情報等をコンフィギュレーションファイル(Yaml)を記入することでmicro-ROSファームウェアを自動生成
    - micro-ROSパッケージとカスタムメッセージ型定義パッケージを生成
- 本ツールの利用により，ユーザはmicro-ROSの通信相手であるROS2アプリの開発に専念できる
- ROSのトピック通信を使用する

# 構成
- SPIKE Prime Hub(Hub)と, Linux環境(Ubuntu)を使用する
    - Linuxはmicro-ROS Agant(Agant)の実行やROS2アプリの開発，及び本ツールを実行してmicro-ROSファームウェアを生成するのに使用
- Agentの実行やROS2アプリの開発・実行はRaspberry Piで行うことも可能（動作確認済み）
- Agentを実行するPCとHubはUARTで接続する

# 動作確認済みバージョン
- spike-rt (GitHubコミット番号) : [f6724115b0ef8c8367a760eaec2840089e6b4e55](https://github.com/spike-rt/spike-rt/tree/f6724115b0ef8c8367a760eaec2840089e6b4e55)
- micro-ROS_ASP3 (GitHubコミット番号) : [3a306729a797d0f4976daab50c5698acffe38a12](https://github.com/exshonda/micro-ROS_ASP3/tree/3a306729a797d0f4976daab50c5698acffe38a12)
- Python : 3.10以降

# 動作環境
- ターゲット
    - ハードウェア：SPIKE Prime hub (STM32F413)
    - OS：SPIKE-RT
- ホストPC（本ツールの実行やmicro-ROS Agentの実行，ROS2アプリ開発に使用）
    - OS : Ubuntu20.04 LTSまたはUbuntu22.04 LTS
- Raspberry Pi (オプション，micro-ROS Agentの実行やROS2アプリ開発に使用)
    - ハードウェア：Raspberry Pi (Raspberry Pi 4のみ動作確認済み)
    - OS：Raspberry Pi OS (64bit)
        - **2023-12-05リリース版**を使用することを推奨
        - アーカイブ入手先例：[https://downloads.raspberrypi.com/raspios_armhf/images/?_gl=1*g4pkln*_ga*MTY4NzY2Mzg1NS4xNzA5MDI4NTI3*_ga_22FD70LWDS*MTcwOTEwMTA1MS4yLjEuMTcwOTEwMTUyNS4wLjAuMA..](https://downloads.raspberrypi.com/raspios_armhf/images/?_gl=1*g4pkln*_ga*MTY4NzY2Mzg1NS4xNzA5MDI4NTI3*_ga_22FD70LWDS*MTcwOTEwMTA1MS4yLjEuMTcwOTEwMTUyNS4wLjAuMA..)

- その他
    - HubとAgentを実行するPCはUARTで接続する
        - 詳細は後述の「[環境構築](#環境構築)」を参考

# 本ツールの使用方法
- 本ドキュメントの「[環境構築](#環境構築)」に従い環境を構築する
- 仕様書「[SPECIFICATION.md](./SPECIFICATION.md)」に従いツールを使用する

<br>
<br>

# 環境構築
## Ubuntu PC側の環境構築
(注)Raspberry Piの環境構築の方法とは異なる

### ROS2のインストール
- [公式サイト](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)等のサイトや後述の付録を参考にしてインストールする
    - ディストーションはHumbleのみ動作確認済み
- もしくは付録の[ROS2 HumbleのUbuntuPCへのインストール方法](#ros2-humbleのubuntupcへのインストール方法)を参考にしてインストール

### spike-rtとmicro-ROS_ASP3をインストール
- micro-ROSファームウェアのビルドを行うPC(Linux)上でワークスペースを作成
    ```
    mkdir ~/asp_uros_ws
    ```
- ワークスペース上にspike-rtとmicro-ROS_ASP3をインストール
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
        - spike-rt：[https://github.com/spike-rt/spike-rt](https://github.com/spike-rt/spike-rt)
        - micro-ROS_ASP3：[https://github.com/exshonda/micro-ROS_ASP3](https://github.com/exshonda/micro-ROS_ASP3)
- micro-ROS_ASP3でPrime Hub向けの設定を行う
    - [micro-ROS_ASP3/spike-rt/README.md](https://github.com/exshonda/micro-ROS_ASP3/blob/master/spike-rt/README.md)を参考にspike-rt対応のセットアップを行う
    - `micro-ROS_ASP3/Makefile.config`のターゲットボードをPrime Hubを選択する
    - ToDo オブジェクト数の静的指定

### ROS2ワークスペースを作成する
- micro-ROS_ASP3と同じ階層に作成する
- 名前は`ros2_ws`とする
```
cd ~/asp_uros_ws
mkdir ros2_ws
cd ros2_ws ; mkdir src
colcon build
. install/setup.bash
```

### 本ツールのインストール
- micro-ROS_ASP3の直下にインストール
    ```
    cd ~/asp_uros_ws/micro-ROS_ASP3
    git clone git@github.com:Hiyama1026/uros_spike-rt_gen.git
    ```

### エージェントのビルドと実行（micro-ROS AgentをUbuntu上で実行する場合のみ必要）
0. 参考
    - 下記の記事を参考に`Micro-XRCE-DDS-Agent`をビルドする
        - <https://qiita.com/lutecia16v/items/5760551dd3a7a0d3e7d3>

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
    - **CMakeのバージョンが古いと言われた場合は新しくする**
        - [CMake公式](https://cmake.org/download/)の[Older Releases](https://cmake.org/files/)からcmake-3.27.2-linux-aarch64.tar.gzをダウンロード
        - 下記コマンドで更新([参考サイト](https://qiita.com/koki2022/items/481c1b03445567263a97))
            - ``$ tar -zxvf cmake-3.27.2-linux-aarch64.tar.gz``
            - ``$ cd cmake-3.27.2-linux-aarch64/``
            - ``$ cd cmake-3.27.2-linux-aarch64/``
            - ``$ sudo ln -s /opt/cmake-3.27.2-linux-aarch64/bin/* /usr/bin``
        - .bashrcに以下を追加してロード ($ source ~/.bashrc)
            ```
            export PATH=/opt/cmake-3.27.2-linux-aarch64/bin:$PATH
            export CMAKE_PREFIX_PATH=/opt/cmake-3.27.2-linux-aarch64:$CMAKE_PREFIX_PATH
            ```
        - <補足>
            - ``$ sudo apt remove cmake``等をするとROSが壊れる
            - 壊れた場合は``$ sudo apt install -y ./ros-humble-desktop-0.3.1_arm64.deb``を再度実行

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


### SPIKE Prime HubとUbuntuPCを接続する（micro-ROS AgentをUbuntu上で実行する場合のみ必要）
- UbuntuPC上でAgentを実行する場合のみ必要
    - Agentを実行するPCとHubを接続する
- PCとHubはUARTにより接続する
    - Hub側は接続ケーブルを**ポートF**に接続する
    - PC側はUSB等で接続する
</br>
<img src="img/ubuntu_hub_connect.jpg" width="50%">
</br>

## Raspberry Pi側の環境構築(オプション)
- 以下の場合にのみ実施する
    - micro-ROS AgentをRaspberry Pi上で実行する
    - ROS 2アプリをRaspberry Pi上で開発・実行する
### Raspberry Pi OS (**64bit**)をインストール
- [インストラー](https://www.raspberrypi.com/software/)をインストール
- インストラーからRaspberry Pi OS(64bit)をインストール
    - ROSを動かすために64bit版をインストールする
    - 動作確認済みバージョンは**2023-12-05リリース版**

### ROS 2をのRaspberry Piにインストール
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

### GPIOの接続を有効にする（GPIOピンを使用してRaspberry PiとSPIKEをシリアル接続する場合に必要）
- micro-ROS AgentをRaspberry Pi上で実行する場合のみ必要
- GPIOピンを使用せずにUSB等でシリアル接続する場合は不要

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


### エージェントのビルドと実行
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


## HubとRaspberry Piを接続
- シリアル通信のケーブル（UART）をRaspberry Piに接続する
    - GPIOピンを使用した参考：[https://github.com/ETrobocon/RasPike/wiki/connect_raspi_spike](https://github.com/ETrobocon/RasPike/wiki/connect_raspi_spike)
- Raspberry Piに接続したシリアルケーブルをPrime Hubの**ポートF**に接続する

# 免責
- 本ソフトウェアの利用により直接的または間接的に生じたいかなる損害に関しても，その責任を負わない．

<Br>
<Br>
<Br>

# 付録
### ROS2 HumbleのUbuntuPCへのインストール方法
- [公式サイト](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)を参考にした手順

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


### ROS2アプリをその他OSの汎用PC（Windows，RHEL，macOS）へのインストール方法
- [ここ](https://docs.ros.org/en/humble/Installation.html)などのROS2公式サイトを参考にインストールする
- 動作未確認

