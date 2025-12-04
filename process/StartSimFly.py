import sys
import time
import airsim
import pygame
import os
import math

from UAV.capture import *
from UAV.path import *

################################ 无人机基本参数 ################################

vehicle_name = "Drone"              # 无人机名称（需要与setting.json对应）
vehicle_velocity = 2.0              # 基础的控制速度(m/s)
speedup_ratio = 10.0                # 设置临时加速比例
speedup_flag = False                # 用来设置临时加速
vehicle_yaw_rate = 5.0              # 基础的偏航速率
camera_rotations = 0.               # 无人机摄像头角度
camera_rotation_rate = math.pi / 4  # 无人机摄像头最大旋转角度
# 建立连接
AirSim_client = airsim.MultirotorClient()
AirSim_client.confirmConnection()
AirSim_client.enableApiControl(True, vehicle_name)


################################# 拍摄基本信息 #################################

scene_name = "Real City SF"                         # 场景名称
tot_data_num = len(os.listdir(scene_path))            # 已有数据集的数量
cur_data_num = 0                                    # 拍摄的数据集数量
key_counter = 0                                     # 用于控制拍摄频率，短时间内只会拍摄一次照片

################################### Mesh信息 ###################################

# 设置分割对象ID及对应颜色
AirSim_client.simSetSegmentationObjectID(".*", 0, True)
AirSim_client.simSetSegmentationObjectID("Sedan.*", 50, True)
AirSim_client.simSetSegmentationObjectID(".*tree.*", 245, True)
AirSim_client.simSetSegmentationObjectID(".*buld.*", 149, True)
AirSim_client.simSetSegmentationObjectID(".*buiding.*", 149, True)
AirSim_client.simSetSegmentationObjectID(".*shed.*", 149, True)

# 设置物体检测参数（通配符）
AirSim_client.simSetDetectionFilterRadius("bottom_center", airsim.ImageType.Scene, 100 * 100, vehicle_name = 'Drone')
AirSim_client.simAddDetectionFilterMeshName("bottom_center", airsim.ImageType.Scene, "Sedan*", vehicle_name = 'Drone')

# 获取移动物体列表（正则）
movable_object = AirSim_client.simListSceneObjects(name_regex = "Sedan.*")

##############################################################################


def manual_control(): 

    global key_counter, tot_data_num, cur_data_num, camera_rotations

    #! AirSim起飞
    AirSim_client.armDisarm(True, vehicle_name)
    AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

    # AirSim_client.moveToZAsync(-60, 10, vehicle_name=vehicle_name).join()

    AirSim_client.moveToPositionAsync(						# 将无人机移动到指定的三维坐标位置
        0, 
        0, 
        0, 											# 目标位置的 x, y, z 坐标
        10, 									# 移动到目标位置的速度
        vehicle_name = 'Drone'
    )

    # 初始化pygame
    pygame.init()
    screen = pygame.display.set_mode((320, 240))
    pygame.display.set_caption('keyboard ctrl')
    screen.fill((0, 0, 0))

    # 提供了三种关闭方式：若用户点击窗口、ESC关闭按钮，程序会直接退出，无人机会悬停在空中；若用户通过.关闭，程序会等待无人机降落后再退出
    while True:
        yaw_rate = 0.0
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0

        time.sleep(0.02)  # 检测时间间隔为0.02s

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("本次飞行共拍摄" + str(cur_data_num) + "张相片!")
                print("目前数据集中已有" + str(tot_data_num + cur_data_num) + "张相片!")
                pygame.quit()
                sys.exit()

        # 读取键盘指令
        scan_wrapper = pygame.key.get_pressed()

        # 按下空格键加速10倍
        if scan_wrapper[pygame.K_SPACE]:
            scale_ratio = speedup_ratio  # 加速倍率,若按空格则为10倍,否则是1倍
        else:
            scale_ratio = speedup_ratio / speedup_ratio

        # 根据 'A' 和 'D' 按键来设置偏航速率变量
        if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
            yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * vehicle_yaw_rate  # d-a为1顺时针偏航,否则逆时针

        # 根据 'UP' 和 'DOWN' 按键来设置pitch轴速度变量(NED坐标系，x为机头向前)	同时也是前进后退
        if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
            velocity_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio

        # 根据 'LEFT' 和 'RIGHT' 按键来设置roll轴速度变量(NED坐标系，y为正右方)	 同时也是左右飞行
        if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
            velocity_y = -(scan_wrapper[pygame.K_LEFT] - scan_wrapper[pygame.K_RIGHT]) * scale_ratio

        # 根据 'W' 和 'S' 按键来设置z轴速度变量(NED坐标系，z轴向上为负)			同时也是上升下降
        if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
            velocity_z = -(scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio

        # 根据 'C' 按键来拍摄数据集
        if scan_wrapper[pygame.K_c] and key_counter == 0:
            cur_data_num += 1
            image_capture(AirSim_client, cur_data_num)
            key_counter = 3

        # 控制0.06s才拍摄一次
        key_counter = (key_counter - 1) % 3

        if scan_wrapper[pygame.K_q] or scan_wrapper[pygame.K_e]:
            camera_rotations += (scan_wrapper[pygame.K_q] - scan_wrapper[pygame.K_e]) * math.pi / 6 * 0.02 * scale_ratio
            if camera_rotations > 0:
                camera_rotations = 0
            if camera_rotations < - math.pi / 2:
                camera_rotations = - math.pi / 2
            camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(camera_rotations, 0, 0))
            AirSim_client.simSetCameraPose(0, camera_pose)

        # 设置速度控制以及设置偏航控制
        AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=0.02, yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate))

        if scan_wrapper[pygame.K_ESCAPE]:
            print("本次飞行共拍摄" + str(cur_data_num) + "张相片!")
            print("目前数据集中已有" + str(tot_data_num + cur_data_num) + "张相片!")
            pygame.quit()
            sys.exit()

        if scan_wrapper[pygame.K_PERIOD]:
            print("本次飞行共拍摄" + str(cur_data_num) + "张相片!")
            print("目前数据集中已有" + str(tot_data_num + cur_data_num) + "张相片!")
            pygame.quit()
            break

    #! AirSim降落
    AirSim_client.landAsync(vehicle_name=vehicle_name).join()
    AirSim_client.armDisarm(False, vehicle_name)
    AirSim_client.enableApiControl(False)


    sys.exit()


def image_capture(AirSim_client, cur_data_num):  # 拍摄数据集

    global tot_data_num

    num = tot_data_num + cur_data_num

    #* START
    print("\x1b[32m" + "Start capturing!" + "\x1b[0m")

    # TODO: 添加 token 支持

    token = scene_path + "Scene" + str(num) + ".png"

    # Scene
    response = AirSim_client.simGetImage("bottom_center", airsim.ImageType.Scene) 
    with open(scene_path + "Scene" + str(num) + ".png", 'wb') as f:
        f.write(response) 

    # GroundTruth
    drone_position, drone_orientation = save_groundtruth(AirSim_client.simGetGroundTruthKinematics(vehicle_name='Drone'), num, token, scene_name)

    # PointCloud
    parse_lidarData(AirSim_client.getLidarData(), num)

    #? Pose and bbox may not be accurate when the drone is moving(need to be further checked)
    # Object Position & Orientation & 3Dbox
    # position & orientation
    poses = []
    for obj in movable_object:
        pose = AirSim_client.simGetObjectPose(obj)
        poses.append(pose)
    object_position, object_orientation = save_poses(poses, num, token, scene_name, movable_object)
    # 3Dbox
    detections = AirSim_client.simGetDetections("bottom_center", airsim.ImageType.Scene, vehicle_name = 'Drone')
    save_detections(
        detections,
        drone_position,
        drone_orientation,
        object_position,
        object_orientation,
        num,
        token,
        scene_name
    )

    # Voxel
    AirSim_client.simCreateVoxelGrid(drone_position, 200, 200, 200, 1, temp_path + "Voxel" + str(num) + ".binvox")
    vox_process(num, drone_position)

    # SemanticVoxel
    semantic_process(num)

    #* END
    print("\x1b[32m" + "Capture Succeed" + "\x1b[0m")


if __name__ == "__main__":
    manual_control()
    
