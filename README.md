# 車両型移動ロボットの経路追従シミュレーション

## 1. **課題の概要**
本課題では、4年生の皆さんに**車両型移動ロボットが動力学環境で経路追従する様子を確認してもらう**ことを目標とします。

皆さんが課題9で取り組んだ**運動学に基づく制御入力の計算**、また、課題10で取り組んだ**OpenGLによる制御手法の有効性の確認**では、あくまで摩擦や重力、慣性力などの力学的影響を考慮しない理想環境での内容でした。
そのため力学的影響を考慮した**動力学環境で動作させる**ところまではカバーされていません。

本課題では、ROS(Robot Operating System)を用いることで、物理エンジン(動力学環境)の中にいるロボットに計算した制御入力を与え、実際にロボットが動く様子を確認してもらいます。

---

## 2. **本課題で行うこと**
この課題を通じて、以下のことを達成してもらいます：

- **Dockerを活用したROS2の環境構築**
- **Gazebo Simを利用した物理シミュレーション**
- **運動学に基づく制御入力計算（課題9）の動力学環境での適用・検証**
- **シミュレーションの実行 & 可視化**

---

## 3. **必要な環境**
本課題では Docker を使用して環境を構築します。

- **必須ソフトウェア**
    - [Docker](https://www.docker.com/)
    - [Git](https://git-scm.com/)
    - [VcXsrv](https://sourceforge.net/projects/vcxsrv/)（Windows の場合）
    - [XQuartz](https://www.xquartz.org/)（Mac の場合）

- **開発環境**
    - **OS:** Ubuntu 24.04（Dockerコンテナ内）
    - **ROS2ディストリビューション:** Jazzy
    - **物理エンジン:** Gazebo Sim

---

## 4. **課題の進め方**
このプロジェクトを進めるためには、以下の手順を実行します：

### **環境構築（初回のみ）**
1. **Dockerのインストール**  
    - [公式サイト](https://www.docker.com/)からDockerをインストール
    - インストール後、バージョン確認：
        ```bash
        docker --version
        ```

2. **XLaunchのインストール（Windows の場合）**  
    - [VcXsrv](https://sourceforge.net/projects/vcxsrv/) をインストール
    - [WSL上にXサーバをインストールしてGUIを実現する（VcXsrv編）](https://atmarkit.itmedia.co.jp/ait/articles/1812/06/news040.html)に従ってセットアップを行う


3. **リポジトリのクローン**
- **Windows/macOS/Linux（ホスト環境）**
    ```bash
    git clone https://github.com/ysk5201/ROS2_car_like_mobile_robot.git
    ```
    ```bash
    cd ROS2_car_like_mobile_robot
    ```
---

### **Dockerコンテナの立ち上げ**

4. **Dockerイメージのビルド（初回のみ）**
    ```bash
    docker-compose build
    ```

5. **Dockerコンテナの起動**
    ```bash
    docker-compose up -d
    ```

6. **コンテナに入る**
    ```bash
    docker exec -it ros2_test bash
    ```

---

7. **ROS2環境のセットアップ（コンテナ内）**
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
    ```bash
    source install/setup.bash
    ```

---

### **制御入力の計算 & シミュレーション実行**

8. **制御入力の導出**

    `car_like_mobile_robot.cpp`内の`calcControlInput(double t)`関数を編集し、**ロボットが目標経路に沿って走行するような制御入力を設計** します。
    
    具体的には状態変数（x, y, theta, phi）から制御入力（u1, u2）を導出できたらOK

9. **シミュレーション実行**
    - [WSL上にXサーバをインストールしてGUIを実現する（VcXsrv編）](https://atmarkit.itmedia.co.jp/ait/articles/1812/06/news040.html)に従ってXLaunchを立ち上げておく
   
    - 以下コマンドでgazebo sim, 現在位置取得, 制御入力計算ノードを立ち上げ
    
        ```bash
        ros2 launch car_like_mobile_robot_bringup car_like_mobile_robot_sim.launch.py
        ```
   
    - Gazeboが起動し、ロボットが現れたら、**ターミナルで `s` を入力し、`Enter` を押すことでシミュレーション開始**

---

### **考察方法**

10. **目視による考察**
    - Gazebo内で車両型移動ロボットが目標の動作を達成しているか確認 

11. **データを利用した考察**
    - `ros2_ws/src/logs/*.csv` に記録されたデータを利用し、**状態変数・制御入力のグラフを作成**

---

## 5. **さいごに**
- バグの修正や機能追加の提案があれば、Pull Requestを送ってください。
- 質問や議論があれば、Issueを立ててください。
- ぜひお好きなオリジナルロボットの動力学的解析に活用してください