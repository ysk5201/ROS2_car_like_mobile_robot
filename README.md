# 車両型移動ロボットの経路追従シミュレーション

## 1. **この課題について**
本課題では、4年生の皆さんに **車両型移動ロボットが動力学環境で経路追従する様子を確認してもらう** ことを目標とします。

皆さんが課題9で取り組んだ **運動学に基づく制御入力の計算** 、また、課題10で取り組んだ **OpenGLによる制御手法の有効性の確認** では、あくまで摩擦や重力などの力学的影響を考慮しない理想環境での内容でした。そのため力学的影響を考慮した **動力学環境で動作させる** ところまではカバーされていません。

本課題では、ROS(Robot Operating System)を用いることで、物理エンジン(動力学環境)の中にいるロボットに計算した制御入力を与え、実際にロボットが動く様子を確認してもらいます。

---

## 2. **なぜやるのか？**
本課題の目的は以下の通りです：

- **運動学で計算した結果が、動力学環境でどのように作用するのかを体験する**  
- **シミュレーション環境で、導出した制御入力を実際に適用し、期待した動作を得られるか検証する**  
- **ROS2やGazeboを活用し、シミュレーション環境でのロボット制御を学ぶ**

---

## 3. **何ができるのか？**
このプロジェクトを通じて、以下のことが可能になります：

- **運動学に基づく制御入力を、実際に動力学環境で適用してロボットを制御する**  
- **指定された関数に制御入力計算のコードを記述し、経路追従シミュレーションを実行する**  
- **Docker環境でROS2シミュレーションを実行し、ロボットの挙動を可視化する**  

---

## 4. **どうやるのか？**
このプロジェクトを進めるためには、以下の手順を実行します：

### **🛠 環境構築**
1. **Dockerのインストール**  
    - [公式サイト](https://www.docker.com/)からDockerをインストール
    - `docker --version` でインストール確認

2. **XLaunchのインストール（Windowsユーザーのみ）**  
    - [VcXsrv](https://sourceforge.net/projects/vcxsrv/) をインストール
    - [WSL上にXサーバをインストールしてGUIを実現する（VcXsrv編）](https://atmarkit.itmedia.co.jp/ait/articles/1812/06/news040.html)に従ってセットアップを行う

3. **リポジトリのクローン**

    ```bash
    git clone https://github.com/ysk5201/ROS2_car_like_mobile_robot.git
    cd car_like_mobile_robot
---

4. **Docker build**

    ```bash
    docker-compose build
---

5. **Docker up**

    ```bash
    docker-compose up
---

6. **立ち上がっているDockerコンテナに入る**

    ```bash
    docker exec -it ros2_test bash
---

7. **おまじない**

    ```bash
    source insall/setup.bash
    source /opt/ros/jazzy/setup.bash
---

8. **制御入力の導出**
void CarLikeMobileRobot::calcControlInput(double t)関数を自由に記述

9. **実行**
    - XLaunchを立ち上げておく
   
    ```bash
    ros2 launch car_like_mobile_robot_bringup bringup
---

## 5. **さいごに**
- 修正すべき点があればどんどんPull Requestを送ってください