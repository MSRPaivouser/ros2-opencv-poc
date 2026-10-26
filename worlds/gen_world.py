# Gera worlds/bocbot_office.sdf (Gazebo Harmonic) a partir do escritorio do repositorio basic_bocbot.
import sys

out_path = sys.argv[1]

BRICK  = "0.88 0.88 0.88 1"        # paredes externas (no original: textura Bricks)
ORANGE = "0.996 0.4745 0.0196 1"
BEIGE  = "0.9216 0.8078 0.6157 1"

H, T = 2.5, 0.15   # altura e espessura das paredes

# (nome, [(sx, sy, sz, px, py, pz), ...], x, y, yaw, cor)
def single(L):
    return [(L, T, H, 0, 0, H / 2)]

walls = [
    ("Wall_0",  single(4.0),  -7.92943,  1.85,     -1.5708, BRICK),
    ("Wall_3",  single(4.0),  -7.92943, -2.0,      -1.5708, BRICK),
    ("Wall_4",  single(16.0), -0.004434, -3.925,    0.0,    BRICK),
    ("Wall_5",  [(0.148947, T, H, -3.92553, 0, 1.25),
                 (6.95105,  T, H,  0.524473, 0, 1.25),
                 (0.9,      T, 0.5, -3.40105, 0, 2.25)],
                 7.92057, 0.0, 1.5708, BRICK),
    ("Wall_6",  [(14.6894,  T, H, -0.655307, 0, 1.25),
                 (0.410614, T, H,  7.79469,  0, 1.25),
                 (0.9,      T, 0.5, 7.13939, 0, 2.25)],
                 -0.004434, 3.925, 3.14159, BRICK),
    ("Wall_8",  single(3.0),  -3.24179,  2.45506, -1.5708, ORANGE),
    ("Wall_9",  single(2.5),  -2.06679,  1.03006,  0.0,    BEIGE),
    ("Wall_10", single(2.5),  -0.891787, -0.14494, -1.5708, ORANGE),
    ("Wall_11", single(2.5),  -2.06679, -1.31994,  3.14159, ORANGE),
    ("Wall_13", single(1.75), -0.908613,  2.9996,  -1.5708, BEIGE),
    ("Wall_14", single(2.75),  0.391387,  2.19961,  0.0,    ORANGE),
    ("Wall_17", single(1.75),  2.63138,   3.01972, -1.5708, BEIGE),
    ("Wall_18", single(2.5),   3.80638,   2.21972,  0.0,    ORANGE),
    ("Wall_20", single(2.5),   5.74898,   2.68495, -1.5708, BEIGE),
    ("Wall_22", single(2.25),  6.81909,   0.722229, 0.0,    BEIGE),
    ("Wall_24", single(1.75),  5.82943,  -0.902428, -1.5708, ORANGE),
    ("Wall_25", single(2.25),  6.87943,  -1.70243,  0.0,    BEIGE),
]

def wall_link(name, boxes, x, y, yaw, color):
    s = f'      <link name="{name}">\n'
    s += f'        <pose>{x} {y} 0 0 0 {yaw}</pose>\n'
    for i, (sx, sy, sz, px, py, pz) in enumerate(boxes):
        s += f'        <collision name="c{i}">\n'
        s += f'          <pose>{px} {py} {pz} 0 0 0</pose>\n'
        s += f'          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>\n'
        s += f'        </collision>\n'
        s += f'        <visual name="v{i}">\n'
        s += f'          <pose>{px} {py} {pz} 0 0 0</pose>\n'
        s += f'          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>\n'
        s += f'          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>\n'
        s += f'        </visual>\n'
    s += '      </link>\n'
    return s

def wheel(name, x, y):
    return f'''
      <link name="{name}">
        <pose relative_to="chassis">{x} {y} 0 -1.5707 0 0</pose>
        <inertial>
          <mass>0.2</mass>
          <inertia>
            <ixx>0.000542</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.000542</iyy><iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
          <surface>
            <friction><ode><mu>0.5</mu><mu2>0.5</mu2></ode></friction>
          </surface>
        </collision>
      </link>
'''

def joint(name, child):
    return f'''
      <joint name="{name}" type="revolute">
        <parent>chassis</parent>
        <child>{child}</child>
        <axis>
          <xyz expressed_in="__model__">0 1 0</xyz>
          <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
          </limit>
        </axis>
      </joint>
'''

sdf = '''<?xml version="1.0" ?>
<!-- Mundo e robo adaptados do repositorio basic_bocbot (ROS 2 Eloquent + Gazebo Classic)
     para ROS 2 Jazzy + Gazebo Harmonic (gz sim). Gerado por gen_world.py -->
<sdf version="1.8">
  <world name="bocbot_office">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sol">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="chao">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Escritorio (paredes do basic_bocbot) -->
    <model name="escritorio">
      <static>true</static>
      <pose>0.673102 -0.134423 0 0 0 0</pose>
'''

for w in walls:
    sdf += wall_link(*w)

sdf += '''    </model>

    <!-- Alvo: bola vermelha, fora do campo de visao inicial do robo -->
    <model name="bola_vermelha">
      <static>true</static>
      <pose>1.0 -2.8 0.15 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>0.15</radius></sphere></geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry><sphere><radius>0.15</radius></sphere></geometry>
        </collision>
      </link>
    </model>

    <!-- bocbot: chassi azul, 4 rodas brancas e camera frontal -->
    <model name="bocbot" canonical_link="chassis">
      <pose>1.0 0 0 0 0 0</pose>

      <link name="chassis">
        <pose relative_to="__model__">0 0 0.1 0 0 0</pose>
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.00833</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.02833</iyy><iyz>0</iyz>
            <izz>0.03333</izz>
          </inertia>
        </inertial>
        <visual name="chassi_visual">
          <geometry><box><size>0.4 0.2 0.1</size></box></geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <collision name="chassi_collision">
          <geometry><box><size>0.4 0.2 0.1</size></box></geometry>
        </collision>

        <!-- corpo da camera (caixa escura na frente) -->
        <visual name="camera_visual">
          <pose>0.225 0 0.075 0 0 0</pose>
          <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
          </material>
        </visual>

        <!-- sensor de camera, logo a frente da caixa -->
        <sensor name="camera" type="camera">
          <pose>0.27 0 0.075 0 0 0</pose>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>false</visualize>
          <topic>camera</topic>
          <camera>
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
              <width>160</width>
              <height>120</height>
              <format>R8G8B8</format>
            </image>
            <clip><near>0.02</near><far>300</far></clip>
          </camera>
        </sensor>
      </link>
'''
sdf += wheel("front_left_wheel",   0.12,  0.15)
sdf += wheel("front_right_wheel",  0.12, -0.15)
sdf += wheel("back_left_wheel",   -0.12,  0.15)
sdf += wheel("back_right_wheel",  -0.12, -0.15)
sdf += joint("front_left_wheel_joint",  "front_left_wheel")
sdf += joint("front_right_wheel_joint", "front_right_wheel")
sdf += joint("back_left_wheel_joint",   "back_left_wheel")
sdf += joint("back_right_wheel_joint",  "back_right_wheel")

sdf += '''
      <!-- Controle das rodas (skid steer: duas rodas de cada lado) -->
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>front_left_wheel_joint</left_joint>
        <left_joint>back_left_wheel_joint</left_joint>
        <right_joint>front_right_wheel_joint</right_joint>
        <right_joint>back_right_wheel_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
        <odom_publish_frequency>1</odom_publish_frequency>
        <topic>cmd_vel</topic>
      </plugin>
    </model>
  </world>
</sdf>
'''

with open(out_path, "w") as f:
    f.write(sdf)
print("Mundo gerado em", out_path)
