import math


def bezier_curve(control_points, num_points):
    """Bézier曲線を生成する"""
    def bernstein_poly(i, n, t):
        """バーンスタイン多項式"""
        return math.comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

    n = len(control_points) - 1
    curve = []
    prev_x, prev_y = None, None  # 直前の点を保存するための変数

    for t in [i / (num_points - 1) for i in range(num_points)]:
        x, y = 0.0, 0.0
        for i, (px, py) in enumerate(control_points):
            b = bernstein_poly(i, n, t)
            x += b * px
            y += b * py

        if prev_x is None and prev_y is None:  # 最初の点の場合
            theta = math.atan2(control_points[1][1] - control_points[0][1],
                                control_points[1][0] - control_points[0][0])
        else:  # 2点目以降の点の場合
            theta = math.atan2(y - prev_y, x - prev_x)

        curve.append((x, y, theta))
        prev_x, prev_y = x, y  # 現在の点を次のループのために保存

    return curve

def offset_position(x, y, th, offset):
    """指定したオフセットで座標を調整"""
    x_offset = x + offset * math.cos(th)
    y_offset = y + offset * math.sin(th)
    return x_offset, y_offset

MODEL_TEMPLATE = """
    <model name='unit_box_{index}'>
      <link name='box_link'>
        <collision name='box_collision'>
          <geometry>
            <box>
              <size>0.000005 0.000005 0.000005</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='box_visual'>
          <geometry>
            <box>
              <size>{length} {width} {height}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.300000012 0.300000012 0.300000012 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
            <specular>1 1 1 1</specular>
          </material>
        </visual>
        <pose>{x} {y} 0 0 0 {th}</pose>
        <enable_wind>false</enable_wind>
      </link>
      <static>false</static>
      <self_collide>false</self_collide>
    </model>
"""

WORLD_TEMPLATE = """<sdf version='1.10'>
  <world name='empty'>
    <physics name='1ms' type='ignored'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>
    <gravity>0 0 -9.8000000000000007</gravity>
    <magnetic_field>5.5644999999999998e-06 2.2875799999999999e-05 -4.2388400000000002e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <scene>
      <ambient>0.400000006 0.400000006 0.400000006 1</ambient>
      <background>0.699999988 0.699999988 0.699999988 1</background>
      <shadows>true</shadows>
    </scene>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.800000012 0.800000012 0.800000012 1</ambient>
            <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
            <specular>0.800000012 0.800000012 0.800000012 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 0 0</pose>
      <self_collide>false</self_collide>
    </model>
    {models}
    <light name='sun' type='directional'>
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.10000000000000001 -0.90000000000000002</direction>
      <diffuse>0.800000012 0.800000012 0.800000012 1</diffuse>
      <specular>0.200000003 0.200000003 0.200000003 1</specular>
      <attenuation>
        <range>1000</range>
        <linear>0.01</linear>
        <constant>0.90000000000000002</constant>
        <quadratic>0.001</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
  </world>
</sdf>
"""

def generate_models(coordinates, length, width, height):
    """座標リストからモデルを生成"""
    models = []
    for index, (x, y, th) in enumerate(coordinates):
        if index == 0:
            offset = 0.25 * length
            x_offset, y_offset = offset_position(x, y, th, offset)
            obj_length = 0.5 * length
        elif index == len(coordinates) - 1:
            offset = -0.25 * length
            x_offset, y_offset = offset_position(x, y, th, offset)
            obj_length = 0.5 * length
        else:
            x_offset, y_offset = x, y
            obj_length = length
        models.append(MODEL_TEMPLATE.format(index=index, x=x_offset, y=y_offset, th=th, length=obj_length, width=width, height=height))
    return "\n".join(models)

def create_world_file(output_filename, models, view_point_x, view_point_y, view_point_z):
    """worldファイルを生成"""
    with open(output_filename, mode='w') as file:
        file.write(WORLD_TEMPLATE.format(models=models, view_point_x=view_point_x, view_point_y=view_point_y, view_point_z=view_point_z))

def create_world(control_points, num_points = 100,filename = "bezier_box.sdf"):

    # 経路用に使う直方体のパラメータ
    length = 0.24
    width = 0.05
    height = 0.0005

    # 視点位置
    view_point_x = 4
    view_point_y = 3
    view_point_z = 12

    # Bézier曲線を生成
    coordinates = bezier_curve(control_points, num_points)
    
    # モデルを生成
    models = generate_models(coordinates, length, width, height)

    create_world_file(filename, models, view_point_x, view_point_y, view_point_z)

    print(f"Worldファイルが生成されました: {filename}")



# def main():
#     # Bézier曲線の制御点
#     control_points = [(0, 0), (5.0, 0.0), (5.0, 4.0), (10.0, 4.0)]

#     # 経路用に使う直方体の数
#     num_points = 100

#     # 経路用に使う直方体のパラメータ
#     length = 0.24
#     width = 0.05
#     height = 0.0005

#     # 視点位置
#     view_point_x = 4
#     view_point_y = 3
#     view_point_z = 12

#     # Bézier曲線を生成
#     coordinates = bezier_curve(control_points, num_points)

#     # モデルを生成
#     models = generate_models(coordinates, length, width, height)

#     # worldファイルを生成
#     filename = "bezier_box.sdf"
#     create_world_file(filename, models, view_point_x, view_point_y, view_point_z)

#     print(f"Worldファイルが生成されました: {filename}")

# if __name__ == "__main__":
#     main()

