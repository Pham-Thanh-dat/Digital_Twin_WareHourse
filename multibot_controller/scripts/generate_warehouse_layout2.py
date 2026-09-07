#!/usr/bin/env python3
"""Sinh world theo bo cuc MOI (theo ban ve): 2 KHOI ke lon (trai/phai),
moi khoi co NUM_ROWS_PER_BLOCK hang ke xep theo chieu sau (truc Y), hang
GIUA moi khoi la 1 BANG CHUYEN DAI xuyen suot chieu rong khoi (thay vi ke).
2 tuong ngoai cung la tuong bao. Giua 2 khoi la loi di chinh rong.

Kich thuoc tung don vi ke (SHELF_LENGTH, SHELF_WIDTH) giu NGUYEN nhu ban
cu ("y het nhu cu" theo yeu cau) - chi doi CACH SAP XEP thanh hang ngang
(nhieu don vi ke noi tiep nhau tao thanh 1 hang dai) thay vi cot doc.

Bang chuyen: dai HON, nhieu hop HON so voi ban truoc, dung dung co che da
sua loi on dinh (deck lien tuc + roller quay + VelocityControl cho hop).
"""

import math

# ==== Kich thuoc don vi ke: GIU NGUYEN nhu world cu ====
SHELF_LENGTH = 3.6
SHELF_WIDTH = 0.6
SEGMENT_GAP = 0.4        # khe ho nho giua cac doan ke lien tiep trong 1 hang

# ==== Bo cuc moi ====
SEGMENTS_PER_ROW = 6      # so doan ke noi tiep tao thanh 1 hang dai
NUM_ROWS_PER_BLOCK = 7    # so hang trong 1 khoi (theo ban ve)
CONVEYOR_ROW_INDEX = 3    # hang thu 4 (dem tu 0) la bang chuyen, theo ban ve
ROW_PITCH = SHELF_WIDTH + 1.6   # khoang cach tam-toi-tam giua 2 hang KE thuong
                                 # (SHELF_WIDTH + loi di giua cac hang de robot di qua)
CONVEYOR_ADJACENT_GAP = 3.5      # khoang cach RONG HON danh rieng cho 2 phia
                                 # sat bang chuyen, de TTB4 co du cho di chuyen
                                 # doc theo canh bang chuyen (bang chuyen rong
                                 # hon 1 hang ke thuong, can nhieu khong gian hon)

MAIN_AISLE_WIDTH = 4.0    # loi di chinh giua 2 khoi trai/phai
SIDE_CONVEYOR_CLEARANCE = 2.5   # khoang cach tu canh ngoai khoi ke toi bang
                                  # chuyen 2 ben, du rong cho TTB4 di qua
WALL_MARGIN = 3.0
WALL_HEIGHT = 2.5
WALL_THICKNESS = 0.2

ROBOT_ZONE_SIZE = 6.0
ROBOT_ZONE_GAP = 3.0

# ==== Bang chuyen: DAI HON, NHIEU HOP HON so voi ban truoc ====
CONVEYOR_WIDTH = 1.0
CONVEYOR_HEIGHT = 0.5
ROLLER_RADIUS = 0.15
PACKAGE_SIZE = 0.3
PACKAGE_HEIGHT = 0.15
PACKAGE_SPEED = 0.4
PACKAGES_PER_BELT = 8      # tang tu 4 -> 8 (nhieu hop hon)

ROLLER_TOP_Z = CONVEYOR_HEIGHT + 2 * ROLLER_RADIUS
PACKAGE_CLEARANCE = 0.02
PACKAGE_Z = ROLLER_TOP_Z + PACKAGE_HEIGHT / 2.0 + PACKAGE_CLEARANCE

ROW_LENGTH = SEGMENTS_PER_ROW * SHELF_LENGTH + (SEGMENTS_PER_ROW - 1) * SEGMENT_GAP
CONVEYOR_LENGTH = ROW_LENGTH   # bang chuyen dai bang ca hang (dai hon nhieu so voi 8m cu)

BLOCK_WIDTH = ROW_LENGTH
# BLOCK_DEPTH tinh gan dung ban dau, se duoc cap nhat chinh xac sau khi
# build_block() chay xong (vi co 2 khoang CONVEYOR_ADJACENT_GAP rong hon xen giua)
BLOCK_DEPTH = (NUM_ROWS_PER_BLOCK - 1) * ROW_PITCH + 2 * (CONVEYOR_ADJACENT_GAP - ROW_PITCH) + ROW_PITCH

START_X = 0.0
START_Y = 0.0

LEFT_BLOCK_X = START_X
RIGHT_BLOCK_X = START_X + BLOCK_WIDTH + MAIN_AISLE_WIDTH

# Bang chuyen 2 BEN NGOAI (them moi): 1 ben trai khoi trai, 1 ben phai khoi phai
WEST_BELT_X = START_X - SIDE_CONVEYOR_CLEARANCE - CONVEYOR_WIDTH / 2.0
EAST_BELT_X = (RIGHT_BLOCK_X + BLOCK_WIDTH) + SIDE_CONVEYOR_CLEARANCE + CONVEYOR_WIDTH / 2.0
east_belt_far_edge = EAST_BELT_X + CONVEYOR_WIDTH / 2.0 + 0.1  # +0.1 cho thanh gio

robot_zone_x_min = east_belt_far_edge + ROBOT_ZONE_GAP
robot_zone_x_max = robot_zone_x_min + ROBOT_ZONE_SIZE
robot_zone_y_min = START_Y
robot_zone_y_max = START_Y + ROBOT_ZONE_SIZE

world_min_x = WEST_BELT_X - CONVEYOR_WIDTH / 2.0 - WALL_MARGIN
world_max_x = robot_zone_x_max + WALL_MARGIN
world_min_y = START_Y - WALL_MARGIN
# world_max_y duoc tinh CHINH XAC sau khi build_block() chay xong (o duoi),
# vi phu thuoc vao row_ys thuc te (khong con deu do CONVEYOR_ADJACENT_GAP)

header = """<?xml version='1.0' encoding='ASCII'?>
<sdf version='1.7'>
  <world name='warehouse_layout2'>
    <physics type="ode">
      <max_step_size>0.006</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin name='ignition::gazebo::systems::Physics' filename='libignition-gazebo-physics-system.so'/>
    <plugin name='ignition::gazebo::systems::UserCommands' filename='libignition-gazebo-user-commands-system.so'/>
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='libignition-gazebo-scene-broadcaster-system.so'/>
    <plugin name='ignition::gazebo::systems::Sensors' filename='libignition-gazebo-sensors-system.so'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name='ignition::gazebo::systems::Imu' filename='libignition-gazebo-imu-system.so'/>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0.0 0.0 1</normal><size>1 1</size></plane></geometry>
        </collision>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>
"""

footer = """  </world>
</sdf>
"""

def shelf_segment(name, x, y):
    """1 doan ke, huong doc theo truc X (nam ngang trong hang), giu nguyen
    kich thuoc SHELF_LENGTH x SHELF_WIDTH nhu ban cu, chi xoay 90 do so
    voi ban truoc (truoc: doc theo Y trong 1 cot; nay: doc theo X trong 1 hang)."""
    return f"""    <include>
      <uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri>
      <name>{name}</name>
      <pose>{x} {y} 0 0 0 0</pose>
    </include>
"""

def wall_model(name, x, y, length, thickness, height, yaw=0.0):
    return f"""    <model name='{name}'>
      <static>true</static>
      <pose>{x} {y} {height/2.0} 0 0 {yaw}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{length} {thickness} {height}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{length} {thickness} {height}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

def goal_zone_marker(name, x, y, size=1.0, color=(0.2, 0.8, 0.2, 0.8)):
    r, g, b, a = color
    return f"""    <model name='{name}'>
      <static>true</static>
      <pose>{x} {y} 0.005 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>{size} {size} 0.005</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
            <emissive>{r*0.3} {g*0.3} {b*0.3} 1</emissive>
          </material>
        </visual>
      </link>
    </model>
"""

def roller(name, x, y, z, length):
    return f"""    <model name='{name}'>
      <pose>{x} {y} {z} 1.5708 0 0</pose>
      <link name='roller_link'>
        <inertial>
          <mass>0.5</mass>
          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <visual name='visual'>
          <geometry><cylinder><radius>{ROLLER_RADIUS}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.12 1</ambient><diffuse>0.1 0.1 0.12 1</diffuse>
            <specular>0.4 0.4 0.4 1</specular></material>
        </visual>
        <collision name='collision'>
          <geometry><cylinder><radius>{ROLLER_RADIUS}</radius><length>{length}</length></cylinder></geometry>
        </collision>
      </link>
      <joint name='spin_joint' type='revolute'>
        <parent>world</parent>
        <child>roller_link</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <plugin filename='libignition-gazebo-joint-controller-system.so'
              name='ignition::gazebo::systems::JointController'>
        <joint_name>spin_joint</joint_name>
        <initial_velocity>6.0</initial_velocity>
      </plugin>
    </model>
"""

def package_box(name, x, y, z, yaw):
    half_z = PACKAGE_HEIGHT / 2.0
    return f"""    <model name='{name}'>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name='box_link'>
        <inertial><mass>0.2</mass>
          <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <visual name='visual_body'>
          <geometry><box><size>{PACKAGE_SIZE} {PACKAGE_SIZE} {PACKAGE_HEIGHT}</size></box></geometry>
          <material><ambient>0.72 0.52 0.28 1</ambient><diffuse>0.72 0.52 0.28 1</diffuse></material>
        </visual>
        <visual name='visual_strap'>
          <pose>0 0 {half_z + 0.001} 0 0 0</pose>
          <geometry><box><size>{PACKAGE_SIZE} {PACKAGE_SIZE*0.15} 0.002</size></box></geometry>
          <material><ambient>0.35 0.22 0.1 1</ambient><diffuse>0.35 0.22 0.1 1</diffuse></material>
        </visual>
        <collision name='collision'>
          <geometry><box><size>{PACKAGE_SIZE} {PACKAGE_SIZE} {PACKAGE_HEIGHT}</size></box></geometry>
          <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>
        </collision>
      </link>
      <plugin filename='libignition-gazebo-velocity-control-system.so'
              name='ignition::gazebo::systems::VelocityControl'>
        <link_name>box_link</link_name>
      </plugin>
    </model>
"""

def conveyor(name_prefix, x, y, yaw, length, num_packages):
    body = ""
    half = length / 2.0

    body += f"""    <model name='{name_prefix}_frame'>
      <static>true</static>
      <pose>{x} {y} {CONVEYOR_HEIGHT/2.0} 0 0 {yaw}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{length} {CONVEYOR_WIDTH} {CONVEYOR_HEIGHT}</size></box></geometry>
        </collision>
        <visual name='visual_base'>
          <geometry><box><size>{length} {CONVEYOR_WIDTH} {CONVEYOR_HEIGHT}</size></box></geometry>
          <material><ambient>0.55 0.56 0.58 1</ambient><diffuse>0.55 0.56 0.58 1</diffuse>
            <specular>0.6 0.6 0.6 1</specular></material>
        </visual>
      </link>
    </model>
"""
    deck_thickness = 0.05
    deck_z = ROLLER_TOP_Z - deck_thickness / 2.0
    body += f"""    <model name='{name_prefix}_deck'>
      <static>true</static>
      <pose>{x} {y} {deck_z} 0 0 {yaw}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{length} {CONVEYOR_WIDTH} {deck_thickness}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{length} {CONVEYOR_WIDTH} {deck_thickness}</size></box></geometry>
          <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.08 0.08 0.08 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

    dx = math.cos(yaw)
    dy = math.sin(yaw)

    rail_h = 0.08
    rail_w = 0.05
    for side_sign, tag in [(1, "left"), (-1, "right")]:
        rail_x = x - dy * side_sign * (CONVEYOR_WIDTH/2.0 + rail_w/2.0)
        rail_y = y + dx * side_sign * (CONVEYOR_WIDTH/2.0 + rail_w/2.0)
        body += f"""    <model name='{name_prefix}_rail_{tag}'>
      <static>true</static>
      <pose>{rail_x} {rail_y} {ROLLER_TOP_Z + rail_h/2.0} 0 0 {yaw}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>{length} {rail_w} {rail_h}</size></box></geometry>
          <material><ambient>0.75 0.55 0.1 1</ambient><diffuse>0.75 0.55 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

    end1_x = x - dx * half
    end1_y = y - dy * half
    end2_x = x + dx * half
    end2_y = y + dy * half
    body += roller(f"{name_prefix}_roller_start", end1_x, end1_y, CONVEYOR_HEIGHT + ROLLER_RADIUS, CONVEYOR_WIDTH)
    body += roller(f"{name_prefix}_roller_end", end2_x, end2_y, CONVEYOR_HEIGHT + ROLLER_RADIUS, CONVEYOR_WIDTH)

    spacing = length / num_packages
    for i in range(num_packages):
        px = x - dx * half + dx * spacing * (i + 0.5)
        py = y - dy * half + dy * spacing * (i + 0.5)
        body += package_box(f"{name_prefix}_pkg{i}", px, py, PACKAGE_Z, yaw)

    return body

def build_block(block_name, block_x):
    """1 khoi ke: NUM_ROWS_PER_BLOCK hang, moi hang la SEGMENTS_PER_ROW
    doan ke noi tiep doc truc X, tru hang CONVEYOR_ROW_INDEX la bang chuyen.

    Khoang cach giua cac hang KHONG DEU: 2 phia sat hang bang chuyen dung
    CONVEYOR_ADJACENT_GAP (rong hon) de TTB4 co du khong gian di chuyen
    doc theo canh bang chuyen; cac hang ke-ke con lai dung ROW_PITCH binh
    thuong nhu truoc."""
    body = ""
    conveyor_positions = []

    # Tinh truoc danh sach Y cua tung hang, dua tren khoang cach TICH LUY
    # (khong con la row*ROW_PITCH deu nhau nua)
    row_ys = [START_Y + ROW_PITCH / 2.0]
    for row in range(1, NUM_ROWS_PER_BLOCK):
        prev_is_conveyor = (row - 1 == CONVEYOR_ROW_INDEX)
        curr_is_conveyor = (row == CONVEYOR_ROW_INDEX)
        gap = CONVEYOR_ADJACENT_GAP if (prev_is_conveyor or curr_is_conveyor) else ROW_PITCH
        row_ys.append(row_ys[-1] + gap)

    for row in range(NUM_ROWS_PER_BLOCK):
        row_y = row_ys[row]
        row_center_x = block_x + BLOCK_WIDTH / 2.0

        if row == CONVEYOR_ROW_INDEX:
            conveyor_positions.append(row_y)
            continue  # bang chuyen se duoc them rieng ben ngoai ham nay

        for seg in range(SEGMENTS_PER_ROW):
            seg_x = block_x + seg * (SHELF_LENGTH + SEGMENT_GAP) + SHELF_LENGTH / 2.0
            name = f"shelf_{block_name}_row{row}_seg{seg}"
            body += shelf_segment(name, seg_x, row_y)

    return body, conveyor_positions, row_ys

body = ""

left_body, left_conveyor_ys, left_row_ys = build_block("left", LEFT_BLOCK_X)
right_body, right_conveyor_ys, right_row_ys = build_block("right", RIGHT_BLOCK_X)
body += left_body
body += right_body
actual_block_depth = max(left_row_ys[-1], right_row_ys[-1]) + ROW_PITCH / 2.0

world_max_y = max(START_Y + actual_block_depth, robot_zone_y_max) + WALL_MARGIN
world_center_x = (world_min_x + world_max_x) / 2.0
world_center_y = (world_min_y + world_max_y) / 2.0
world_width_x = world_max_x - world_min_x
world_width_y = world_max_y - world_min_y

# --- Bang chuyen: dat dung o hang giua moi khoi, huong doc theo truc X
#     (yaw=0), dai bang ca hang (CONVEYOR_LENGTH = ROW_LENGTH), nhieu hop hon ---
for y in left_conveyor_ys:
    x_center = LEFT_BLOCK_X + BLOCK_WIDTH / 2.0
    body += conveyor("inbound_belt", x_center, y, 0.0, CONVEYOR_LENGTH, PACKAGES_PER_BELT)

for y in right_conveyor_ys:
    x_center = RIGHT_BLOCK_X + BLOCK_WIDTH / 2.0
    body += conveyor("outbound_belt", x_center, y, 0.0, CONVEYOR_LENGTH, PACKAGES_PER_BELT)

# --- Bang chuyen 2 BEN NGOAI (them moi): doc theo truc Y, chay suot chieu
#     sau ca 2 khoi, dat ngoai canh trai khoi trai va canh phai khoi phai ---
side_belt_length = actual_block_depth
side_belt_y_center = START_Y + actual_block_depth / 2.0
SIDE_PACKAGES_PER_BELT = 8

body += conveyor("west_belt", WEST_BELT_X, side_belt_y_center, 1.5708, side_belt_length, SIDE_PACKAGES_PER_BELT)
body += conveyor("east_belt", EAST_BELT_X, side_belt_y_center, 1.5708, side_belt_length, SIDE_PACKAGES_PER_BELT)

# --- Goal markers cho robot (dat ranh ro trong loi di chinh giua 2 khoi) ---
aisle_center_x = LEFT_BLOCK_X + BLOCK_WIDTH + MAIN_AISLE_WIDTH / 2.0
GOAL_COLORS = [
    (1.0, 0.2, 0.2, 0.8), (0.2, 0.5, 1.0, 0.8), (1.0, 0.8, 0.0, 0.8),
    (0.2, 0.8, 0.2, 0.8), (0.8, 0.2, 0.8, 0.8), (0.2, 0.8, 0.8, 0.8),
    (0.8, 0.8, 0.2, 0.8), (0.8, 0.2, 0.2, 0.8), (0.2, 0.2, 0.8, 0.8),
    (0.5, 0.5, 0.5, 0.8),
]
for i in range(NUM_ROWS_PER_BLOCK):
    if i == CONVEYOR_ROW_INDEX:
        continue
    y = left_row_ys[i]
    color = GOAL_COLORS[i % len(GOAL_COLORS)]
    body += goal_zone_marker(f"goal_zone_robot{i+1}", aisle_center_x, y, size=1.0, color=color)

# --- Tuong bao (2 tuong doc ngoai cung theo ban ve + tuong bac/nam) ---
body += wall_model("wall_south", world_center_x, world_min_y, world_width_x, WALL_THICKNESS, WALL_HEIGHT, yaw=0.0)
body += wall_model("wall_north", world_center_x, world_max_y, world_width_x, WALL_THICKNESS, WALL_HEIGHT, yaw=0.0)
body += wall_model("wall_west",  world_min_x, world_center_y, world_width_y, WALL_THICKNESS, WALL_HEIGHT, yaw=1.5708)
body += wall_model("wall_east",  world_max_x, world_center_y, world_width_y, WALL_THICKNESS, WALL_HEIGHT, yaw=1.5708)

with open("warehouse_layout2.world", "w") as f:
    f.write(header + body + footer)

print("Da tao warehouse_layout2.world:")
print(f"  2 khoi ke: LEFT tai x={LEFT_BLOCK_X:.1f}, RIGHT tai x={RIGHT_BLOCK_X:.1f}")
print(f"  Moi khoi: {NUM_ROWS_PER_BLOCK} hang x {SEGMENTS_PER_ROW} doan ke, hang giua (index {CONVEYOR_ROW_INDEX}) la bang chuyen")
print(f"  Bang chuyen giua khoi: dai {CONVEYOR_LENGTH:.1f}m (= ca hang), {PACKAGES_PER_BELT} hop/bang")
print(f"  Bang chuyen 2 BEN NGOAI (moi): WEST tai x={WEST_BELT_X:.2f}, EAST tai x={EAST_BELT_X:.2f},")
print(f"    doc theo Y, dai {side_belt_length:.1f}m, tam y={side_belt_y_center:.2f}, {SIDE_PACKAGES_PER_BELT} hop/bang")
print(f"  Loi di chinh giua 2 khoi: x [{LEFT_BLOCK_X+BLOCK_WIDTH:.1f}, {RIGHT_BLOCK_X:.1f}], rong {MAIN_AISLE_WIDTH}m")
print(f"  Tuong bao: x [{world_min_x:.1f}, {world_max_x:.1f}], y [{world_min_y:.1f}, {world_max_y:.1f}]")
print(f"  VUNG ROBOT: x [{robot_zone_x_min:.1f}, {robot_zone_x_max:.1f}], y [{robot_zone_y_min:.1f}, {robot_zone_y_max:.1f}]")
print()
print("Sau khi launch world nay, chay script run_conveyors.sh (sua lai")
print("CONVEYOR_X/INBOUND_Y/OUTBOUND_Y/BELT_LENGTH/NUM_PACKAGES cho khop")
print("voi cac gia tri in ra o tren) de kich hoat chuyen dong bang chuyen.")
