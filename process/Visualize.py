import numpy as np
from mayavi import mlab
import xml.etree.ElementTree as ET
import quaternion
from UAV.path import *
from UAV.utils import *

################################## 关键帧信息 ##################################
# index of the data to visualize
KEY = 1
# PATH
groundtruth_path = groundtruth_path + "GroundTruth" + str(KEY) + ".xml"
pcd_path = pcd_path + "PointCloud" + str(KEY) + ".txt"
pose_path = pose_path + "Pose" + str(KEY) + ".xml"
detection_path = detection_path + "Detection" + str(KEY) + ".xml"
voxel_path = voxel_path + "Voxel" + str(KEY) + ".txt"
semanticvox_path = semanticvox_path + "SemanticVoxel" + str(KEY) + ".txt"

# todo
INTER = []
OBJECT = {50:"SM_bus_articulated_14"}

################################### 色彩LUT ###################################

palette = np.load("E:/__WORKSPACE__/Python/UAV/process/ColorLUT/palette.npy")
palette = np.hstack((palette, np.ones((palette.shape[0], 1), dtype=np.uint8) * 255))
palette[50] = [255, 0, 0, 255]

################################### 绘图信息 ###################################

class DataInfo:
    drone = {
        'position': None,
        'orientation': None
    }
    object = {
        'position': {},
        'orientation': {},
        'box3D': {}
    }


class Draw:

    def __init__(self, datainfo):

        ### init figure ###
        # mlab.options.offscreen = True
        fig = mlab.figure(bgcolor=(1, 1, 1), size=(1920, 1080))
        # origin point
        mlab.points3d(0, 0, 0, color=(1, 1, 1), mode="sphere", scale_factor=0.2)
        # xyz axis
        axes = np.array([[20.0, 0.0, 0.0, 0.0], [0.0, 20.0, 0.0, 0.0], [0.0, 0.0, 20.0, 0.0]], dtype=np.float64,)
        mlab.plot3d([0, axes[0, 0]], [0, axes[0, 1]], [0, axes[0, 2]], color=(1, 0, 0), tube_radius=None, figure=fig)   # x轴
        mlab.plot3d([0, axes[1, 0]], [0, axes[1, 1]], [0, axes[1, 2]], color=(0, 1, 0), tube_radius=None, figure=fig)   # y轴
        mlab.plot3d([0, axes[2, 0]], [0, axes[2, 1]], [0, axes[2, 2]], color=(0, 0, 1), tube_radius=None, figure=fig)   # z轴
        self.fig = fig 
 
        ### filepath ###
        self.pcd_path = pcd_path
        self.pose_path = pose_path
        self.detection_path = detection_path
        self.voxel_path = voxel_path
        self.semanticvox_path = semanticvox_path

        ### data info ###
        self.datainfo = datainfo


    def pcd(self):
        with open(self.pcd_path, 'r') as f:
            tmp = []
            lines = f.readlines()
            for line in lines:
                tmp.append(line.split(" "))
        pcd = np.array(tmp, dtype=np.dtype('f4')).reshape(-1, 4)
        pts = mlab.points3d(
            pcd[:, 0],
            pcd[:, 1],
            pcd[:, 2],
            pcd[:, 3],
            colormap="viridis",
            scale_factor = 0.75,
            # scale_factor = voxel_size - 0.05 * voxel_size,
            mode="sphere",
            opacity=1.0,
            vmin=0,
            vmax=19,
            figure=self.fig
        )
        pts.glyph.scale_mode = "scale_by_vector"
        pts.module_manager.scalar_lut_manager.lut._vtk_obj.SetTableRange(0, palette.shape[0])
        pts.module_manager.scalar_lut_manager.lut.number_of_colors = palette.shape[0]
        pts.module_manager.scalar_lut_manager.lut.table = palette


    def object_orientation(self, color=(0,0,1), line_width=6, z_translate=10, scale_factor=3):
        # only show orientation of detected objects
        for name in self.datainfo.object["box3D"].keys():
            quiver = np.array([1, 0, 0]) @ quaternion.as_rotation_matrix(self.datainfo.object["orientation"][name])
            pos = self.datainfo.object["position"][name]
            mlab.quiver3d(
                pos['x'], pos['y'], pos['z']-z_translate, quiver[0], quiver[1], quiver[2],
                line_width=line_width, scale_factor=scale_factor, figure=self.fig, color=color
            )

    
    def box_3d(self, color=(0,1,0), line_width=1):
        for name, box in self.datainfo.object["box3D"].items():
            # draw position point
            position = self.datainfo.object["position"][name]
            mlab.points3d(position['x'], position['y'], position['z'], color=(0, 1, 0), mode="sphere", scale_factor=0.5, figure=self.fig)
            # draw box
            for k in range(0,4):
                i, j = k, (k+1)%4
                mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=self.fig)
                i, j = k+4, (k+3)%4+4
                mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=self.fig)
                i, j = k, k+4
                mlab.plot3d([box[i,0], box[j,0]], [box[i,1], box[j,1]], [box[i,2], box[j,2]], color=color, tube_radius=None, line_width=line_width, figure=self.fig)


    def voxel(self, semantic=True):
        if semantic:
            self._semantic_voxel()
        else:
            self._vanilla_voxel()


    def _vanilla_voxel(self, color=(220/255, 220/255, 220/255), opacity=0.75, scale_factor=1):
        voxel = []
        with open(self.voxel_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                data = line.split(" ")
                voxel.append([float(data[0]), float(data[1]), float(data[2])])
        voxel = np.array(voxel)

        mlab.points3d(voxel[:, 0], voxel[:, 1], voxel[:, 2], mode='cube', color=color, opacity=opacity, scale_factor=scale_factor, figure=self.fig)


    def _semantic_voxel(self, scale_factor = 0.75, opacity=1.0):
        voxel = []
        with open(self.semanticvox_path) as f:
            lines = f.readlines()
            for line in lines:
                data = line.split(" ")
                voxel.append([float(data[0]), float(data[1]), float(data[2]), int(data[3])])

        voxel = np.array(voxel)
        voxel_size = 0.5

        vox = mlab.points3d(
            voxel[:, 0],
            voxel[:, 1],
            voxel[:, 2],
            voxel[:, 3],
            colormap="viridis",
            scale_factor = scale_factor,
            # scale_factor = voxel_size - 0.05 * voxel_size,
            mode="cube",
            opacity=opacity,
            vmin=0,
            vmax=19,
            figure=self.fig
        )

        vox.glyph.scale_mode = "scale_by_vector"
        vox.module_manager.scalar_lut_manager.lut._vtk_obj.SetTableRange(0, palette.shape[0])
        vox.module_manager.scalar_lut_manager.lut.number_of_colors = palette.shape[0]
        vox.module_manager.scalar_lut_manager.lut.table = palette


def prepare_info():

    datainfo = DataInfo()

    ### GroundTruth ###
    root = ET.parse(groundtruth_path).getroot()

    datainfo.drone["position"] = {
        'x': float(root.find("position").find("x").text),
        'y': float(root.find("position").find("y").text),
        'z': float(root.find("position").find("z").text)
    }

    datainfo.drone["orientation"] = np.quaternion(
        float(root.find("orientation").find("w").text),
        float(root.find("orientation").find("x").text),
        float(root.find("orientation").find("y").text),
        float(root.find("orientation").find("z").text)
    )

    ### Object Pose ###
    for obj in ET.parse(pose_path).getroot().findall('object'):

        obj_name = obj.find('name').text
        
        # orientation
        local_w = float(obj.find('orientation').find('w').text)
        local_x = float(obj.find('orientation').find('x').text)
        local_y = float(obj.find('orientation').find('y').text)
        local_z = float(obj.find('orientation').find('z').text)

        # position
        x = float(obj.find('position').find('x').text)
        y = float(obj.find('position').find('y').text)
        z = float(obj.find('position').find('z').text)

        datainfo.object["orientation"][obj_name] = np.quaternion(local_w, local_x, local_y, local_z)
        datainfo.object["position"][obj_name] = {'x': x, 'y': y, 'z': z}

    
    ### Object 3Dbox ###

    for obj in ET.parse(detection_path).getroot().findall('object'):

        obj_name = obj.find('name').text

        box = []

        for i in range(8):
            x = float(obj.find('box3D').find('x' + str(i)).text)
            y = float(obj.find('box3D').find('y' + str(i)).text)
            z = float(obj.find('box3D').find('z' + str(i)).text)
            box.append([x, y, z])

        datainfo.object["box3D"][obj_name] = np.array(box)

    return Draw(datainfo)


if __name__ == '__main__':

    draw = prepare_info()
    
    ### DRAW ###
    draw.pcd()
    draw.object_orientation()
    draw.box_3d()
    draw.voxel(semantic=True)
    # draw.voxel(semantic=False)

    # mlab.savefig('temp/mayavi.png')
    mlab.show()