import numpy as np
import os

from .utils import *
from .path import *
import open3d as o3d
import xml.etree.ElementTree as ET
import quaternion


__all__ = [ 
    'parse_lidarData',
    'save_detections',
    'save_poses',
    'vox_process',
    'semantic_process',
    'save_groundtruth'
]


def parse_lidarData(data, num): # 解析点云数据
    
    points = np.array(data.point_cloud, dtype=np.dtype('f4')).reshape(-1, 3)
    seg = data.segmentation
    
    if (len(data.point_cloud) < 3):
        print("\tNo points received from Lidar data")
    elif (len(seg) != len(points)):
        print("\tSeg mismatch with points")
    else:
        with open(pcd_path + "PointCloud" + str(num) + ".txt", "w") as f:
            for i in range(points.shape[0]):
                f.write("%f %f %f %d\n" % (points[i, 0], points[i, 1], points[i, 2], seg[i]))
        

def save_detections(
    detections,
    drone_position,
    drone_orientation,
    object_position,
    object_orientation,
    num,
    SceneImage,
    scene_name
):
    # TODO: 添加 prev next 支持

    with open(detection_path + "Detection" + str(num) + ".xml", "w") as f:
        f.write("<annotation>\n")
        f.write("\t<file>" + SceneImage + "</file>\n")
        f.write("\t<scene>" + scene_name + "</scene>\n")
        f.write("\t<prev>" + "pass" + "</prev>\n")
        f.write("\t<next>" + "pass" + "</next>\n")
        f.write("\t<timestamp>" + "pass" + "</timestamp>\n")
        for obj in detections:

            ### Convert Frame ###
            # x, y, z = z, -y, x
            xmin, ymin, zmin = obj.box3D.min.z_val, -obj.box3D.min.y_val, obj.box3D.min.x_val
            xmax, ymax, zmax = obj.box3D.max.z_val, -obj.box3D.max.y_val, obj.box3D.max.x_val

            box = np.array([
                [xmin, ymin, zmin], [xmax, ymin, zmin],
                [xmax, ymax, zmin], [xmin, ymax, zmin],
                [xmin, ymin, zmax], [xmax, ymin, zmax],
                [xmax, ymax, zmax], [xmin, ymax, zmax]
            ])

            #! TODO: rotate according to object orientation

            box = box @ quaternion.as_rotation_matrix(
                np.quaternion(drone_orientation.w_val, drone_orientation.x_val, drone_orientation.y_val, drone_orientation.z_val)
            )
            box += [drone_position.x_val, drone_position.y_val, drone_position.z_val]

            f.write("\t<object>\n")
            f.write("\t\t<name>" + obj.name + "</name>\n")

            f.write("\t\t<box2D>\n")
            f.write("\t\t\t<xmin>" + str(obj.box2D.min.x_val) + "</xmin>\n")
            f.write("\t\t\t<ymin>" + str(obj.box2D.min.y_val) + "</ymin>\n")
            f.write("\t\t\t<xmax>" + str(obj.box2D.max.x_val) + "</xmax>\n")
            f.write("\t\t\t<ymax>" + str(obj.box2D.max.y_val) + "</ymax>\n")
            f.write("\t\t</box2D>\n")

            f.write("\t\t<box3D>\n")
            f.write("\t\t\t<x0>" + str(box[0][0]) + "</x0> <y0>" + str(box[0][1]) + "</y0> <z0>" + str(box[0][2]) + "</z0>\n")
            f.write("\t\t\t<x1>" + str(box[1][0]) + "</x1> <y1>" + str(box[1][1]) + "</y1> <z1>" + str(box[1][2]) + "</z1>\n")
            f.write("\t\t\t<x2>" + str(box[2][0]) + "</x2> <y2>" + str(box[2][1]) + "</y2> <z2>" + str(box[2][2]) + "</z2>\n")
            f.write("\t\t\t<x3>" + str(box[3][0]) + "</x3> <y3>" + str(box[3][1]) + "</y3> <z3>" + str(box[3][2]) + "</z3>\n")
            f.write("\t\t\t<x4>" + str(box[4][0]) + "</x4> <y4>" + str(box[4][1]) + "</y4> <z4>" + str(box[4][2]) + "</z4>\n")
            f.write("\t\t\t<x5>" + str(box[5][0]) + "</x5> <y5>" + str(box[5][1]) + "</y5> <z5>" + str(box[5][2]) + "</z5>\n")
            f.write("\t\t\t<x6>" + str(box[6][0]) + "</x6> <y6>" + str(box[6][1]) + "</y6> <z6>" + str(box[6][2]) + "</z6>\n")
            f.write("\t\t\t<x7>" + str(box[7][0]) + "</x7> <y7>" + str(box[7][1]) + "</y7> <z7>" + str(box[7][2]) + "</z7>\n")
            f.write("\t\t</box3D>\n")

            f.write("\t\t<position>\n")
            f.write("\t\t\t<x>" + str(object_position[obj.name].x_val) + "</x>\n")
            f.write("\t\t\t<y>" + str(object_position[obj.name].y_val) + "</y>\n")
            f.write("\t\t\t<z>" + str(object_position[obj.name].z_val) + "</z>\n")
            f.write("\t\t</position>\n")

            f.write("\t\t<orientation>\n")
            f.write("\t\t\t<w>" + str(object_orientation[obj.name].w_val) + "</w>\n")
            f.write("\t\t\t<x>" + str(object_orientation[obj.name].x_val) + "</x>\n")
            f.write("\t\t\t<y>" + str(object_orientation[obj.name].y_val) + "</y>\n")
            f.write("\t\t\t<z>" + str(object_orientation[obj.name].z_val) + "</z>\n")
            f.write("\t\t</orientation>\n")

            f.write("\t\t<geo_point>\n")
            f.write("\t\t\t<altitude>" + str(obj.geo_point.altitude) + "</altitude>\n")
            f.write("\t\t\t<latitude>" + str(obj.geo_point.latitude) + "</latitude>\n")
            f.write("\t\t\t<longitude>" + str(obj.geo_point.longitude) + "</longitude>\n")
            f.write("\t\t</geo_point>\n")

            f.write("\t</object>\n")

        f.write("</annotation>\n")


def save_poses(poses, num, SceneImage, scene_name, movable_object):

    # TODO: 添加 prev next 支持

    object_position = {}
    object_orientation = {}

    with open(pose_path + "Pose" + str(num) + ".xml", "w") as f:
        f.write("<annotation>\n")
        f.write("\t<file>" + SceneImage + "</file>\n")
        f.write("\t<scene>" + scene_name + "</scene>\n")
        f.write("\t<prev>" + "pass" + "</prev>\n")
        f.write("\t<next>" + "pass" + "</next>\n")
        f.write("\t<timestamp>" + "pass" + "</timestamp>\n")
        for i, obj in enumerate(poses):

            ### Convert Frame ###
            # Don't know why yet, but it just works
            # x, y, z, w = -y, x, -w, -z
            obj.orientation.w_val, obj.orientation.x_val, obj.orientation.y_val, obj.orientation.z_val = \
                -obj.orientation.z_val, -obj.orientation.y_val, obj.orientation.x_val, -obj.orientation.w_val
            
            object_orientation[movable_object[i]] = obj.orientation
            object_position[movable_object[i]] = obj.position

            f.write("\t<object>\n")
            f.write("\t\t<name>" + movable_object[i] + "</name>\n")

            f.write("\t\t<orientation>\n")
            f.write("\t\t\t<w>" + str(obj.orientation.w_val) + "</w>\n")
            f.write("\t\t\t<x>" + str(obj.orientation.x_val) + "</x>\n")
            f.write("\t\t\t<y>" + str(obj.orientation.y_val) + "</y>\n")
            f.write("\t\t\t<z>" + str(obj.orientation.z_val) + "</z>\n")
            f.write("\t\t</orientation>\n")

            f.write("\t\t<position>\n")
            f.write("\t\t\t<x>" + str(obj.position.x_val) + "</x>\n")
            f.write("\t\t\t<y>" + str(obj.position.y_val) + "</y>\n")
            f.write("\t\t\t<z>" + str(obj.position.z_val) + "</z>\n")
            f.write("\t\t</position>\n")

            f.write("\t</object>\n")

        f.write("</annotation>\n")

    return object_position, object_orientation


def save_groundtruth(KinematicsState, num, SceneImage, scene_name):

    # TODO: 添加 prev next 支持

    drone_orientation = KinematicsState.orientation
    drone_position = KinematicsState.position

    ### Convert Frame ###
    # Don't know why yet, but it just works
    # x, y, z, w = -y, x, -w, -z
    drone_orientation.w_val, drone_orientation.x_val, drone_orientation.y_val, drone_orientation.z_val = \
        -drone_orientation.z_val, -drone_orientation.y_val, drone_orientation.x_val, -drone_orientation.w_val

    with open(groundtruth_path + "GroundTruth" + str(num) + ".xml", "w") as f:
        f.write("<annotation>\n")
        f.write("\t<file>" + SceneImage + "</file>\n")
        f.write("\t<scene>" + scene_name + "</scene>\n")
        f.write("\t<prev>" + "pass" + "</prev>\n")
        f.write("\t<next>" + "pass" + "</next>\n")
        f.write("\t<timestamp>" + "pass" + "</timestamp>\n")

        f.write("\t<orientation>\n")
        f.write("\t\t<w>" + str(drone_orientation.w_val) + "</w>\n")
        f.write("\t\t<x>" + str(drone_orientation.x_val) + "</x>\n")
        f.write("\t\t<y>" + str(drone_orientation.y_val) + "</y>\n")
        f.write("\t\t<z>" + str(drone_orientation.z_val) + "</z>\n")
        f.write("\t</orientation>\n")

        f.write("\t<position>\n")
        f.write("\t\t<x>" + str(drone_position.x_val) + "</x>\n")
        f.write("\t\t<y>" + str(drone_position.y_val) + "</y>\n")
        f.write("\t\t<z>" + str(drone_position.z_val) + "</z>\n")
        f.write("\t</position>\n")
        
        f.write("</annotation>\n")

    return drone_position, drone_orientation


def vox_process(num, drone_position):
    with open(temp_path + "Voxel" + str(num) + ".binvox", 'rb') as f:
        model = binvox.read_as_3d_array(f)
    voxels = model.data
    dims = model.dims
    translate = model.translate
    scale = model.scale
    voxel_size = 1.0 / scale / dims[0]
    
    x, y, z = np.indices(dims)
    filled = voxels.flatten()
    x = x.flatten()[filled]
    y = y.flatten()[filled]
    z = z.flatten()[filled]

    x = (x + 0.5) * voxel_size
    y = (y + 0.5) * voxel_size
    z = (z + 0.5) * voxel_size

    x += translate[0]
    y += translate[1]
    z += translate[2]

    ### Convert Frame ###
    x, y, z = y, x, -z
    x += drone_position.x_val
    y += drone_position.y_val
    z += drone_position.z_val

    with open(voxel_path + "Voxel" + str(num) + ".txt", 'w') as f:
        for j in range(len(x)):
            f.write(str(x[j]) + " " + str(y[j]) + " " + str(z[j]) + "\n")

    os.remove(temp_path + "Voxel" + str(num) + ".binvox")
    print("[Info] Voxel Processed!")

    
def semantic_process(num):

    def nn_correspondance(verts1, verts2):
        """ for each vertex in verts2 find the nearest vertex in verts1

            Args:
                nx3 np.array's
            Returns:
                ([indices], [distances])

        """
        indices = []
        distances = []

        if len(verts1) == 0 or len(verts2) == 0:
            return indices, distances

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(verts1)
        kdtree = o3d.geometry.KDTreeFlann(pcd)
        for vert in verts2:
            _, inds, dist = kdtree.search_knn_vector_3d(vert, 1)
            indices.append(inds[0])
            distances.append(np.sqrt(dist[0]))

        return indices, distances
    
    dense_voxels = []
    pcd = []

    with open(voxel_path + "Voxel" + str(num) + ".txt", 'r') as f:
        lines = f.readlines()
        for line in lines:
            data = line.split(" ")
            dense_voxels.append([float(data[0]), float(data[1]), float(data[2])])
    dense_voxels = np.array(dense_voxels)
    vox = dense_voxels.copy()

    with open(pcd_path + "PointCloud" + str(num) + ".txt", 'r') as f:
        lines = f.readlines()
        for line in lines:
            data = line.split(" ")
            pcd.append([float(data[0]), float(data[1]), float(data[2]), int(data[3])])
    pcd = np.array(pcd)

    indices, _ = nn_correspondance(pcd[:,:3], dense_voxels)


    semantic = pcd[:, 3][np.array(indices)]
    result = np.concatenate([vox, semantic[:, np.newaxis]], axis=1)

    # remove self vox
    # with open(groundtruth_path + "GroundTruth" + str(num) + ".txt", 'r') as f:
    #     line = f.readline()
    #     data = line.split(" ")
    #     self_position = [float(data[0]), float(data[1]), float(data[2])]
    
    self_position = [0., 0., 0.]

    # radius to be tested
    radius = 1.5  

    self_mask = ((np.fabs(result[:, 0] - self_position[0]) > radius) 
                | (np.fabs(result[:, 1] - self_position[1]) > radius) 
                | (np.fabs(result[:, 2] - self_position[2]) > radius))
    
    result = result[self_mask]

    with open(semanticvox_path + "SemanticVoxel" + str(num) + ".txt", 'w') as f:
        for i in range(result.shape[0]):
            f.write(str(result[i][0]) + " " + str(result[i][1]) + " " + str(result[i][2]) + " " + str(int(result[i][3])) + "\n")
    print("[Info] Semantic Processed!")