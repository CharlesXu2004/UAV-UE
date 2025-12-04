import os

path = os.path.abspath(__file__)
path = os.path.dirname(path)
path = os.path.dirname(path)

__all__ = ['absolute_path', 
           'scene_path', 
           'segmentation_path', 
           'segmentationnorm_path', 
           'surface_path', 
           'depth_path', 
           'infred_path', 
           'depthperspective_path', 
           'depthplanar_path', 
           'pcd_path', 
           'detection_path', 
           'pose_path', 
           'voxel_path', 
           'semanticvox_path', 
           'groundtruth_path', 
           'temp_path']

absolute_path            = path + "/dataset/"
scene_path               = absolute_path + "SceneImage/"
segmentation_path        = absolute_path + "SegmentationImage/"
segmentationnorm_path    = absolute_path + "SegmentationNormalImage/"
surface_path             = absolute_path + "SurfaceNormalsImage/"
depth_path               = absolute_path + "DepthVisImage/"
infred_path              = absolute_path + "InfraredImage/"
depthperspective_path    = absolute_path + "DepthPerspectiveImage/"
depthplanar_path         = absolute_path + "DepthPlanarImage/"
pcd_path                 = absolute_path + "PointCloud/"
detection_path           = absolute_path + "Detection/"
pose_path                = absolute_path + "Pose/"
voxel_path               = absolute_path + "VoxelGrid/"
semanticvox_path          = absolute_path + "SemanticVoxel/"
groundtruth_path         = absolute_path + "GroundTruth/"
temp_path                = absolute_path + "Temp/"