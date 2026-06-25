#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/22
import cv2
import math
import yaml
import numpy as np
import transforms3d as tfs
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion

def loginfo(msg):
    Node.get_logger().info('\033[1;32m%s\033[0m' % msg)

def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def empty_func(img=None):
    return img

def set_range(x, x_min, x_max):
    tmp = x if x > x_min else x_min
    tmp = tmp if tmp < x_max else x_max
    return tmp

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()

    data = yaml.load(file_data, Loader=yaml.FullLoader)

    return data

def save_yaml_data(data, yaml_file):
    f = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, f)

    f.close()

def distance(point_1, point_2):
    """
    calculate the distance between two points. 计算两个点间的距离
    :param point_1: Point 1. 点1
    :param point_2: Point 2. 点2
    :return: Distance between two points. 两点间的距离
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)

def box_center(box):
    """
    Calculate the center of quadrangle box. 计算四边形box的中心
    :param box: box （x1, y1, x2, y2)形式(box （x1, y1, x2, y2)type)
    :return:  Center coordinate. 中心坐标（x, y)
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

def point_remapped(point, now, new, data_type=float):
    """
    Map the coordinate of one point from a picture to a new picture of different size. 将一个点的坐标从一个图片尺寸映射的新的图片上
    :param point: Coordinate of point.  点的坐标
    :param now: Size of current picture. 现在图片的尺寸
    :param new: New picture size. 新的图片尺寸
    :return: New point coordinate. 新的点坐标
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)

def vector_2d_angle(v1, v2):
    """
    Calculate the angle between two vectors -pi ~ pi. 计算两向量间的夹角 -pi ~ pi
    :param v1: first vector 第一个向量
    :param v2: second vector 第二个向量
    :return: angle 角度
    """
    d_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (d_v1_v2)
    sin = np.cross(v1, v2) / (d_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def warp_affine(image, points, scale=1.0):
    """
     Simple alignment. Calculate the angle of the line connecting the two points. Rotate the picture around the image
     center to make the line horizontal can be used to align the face简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平
    可以用在人脸对齐上

    :param image: select face picture 要选择的人脸图片
    :param points: coordinate of two points 两个点的坐标 ((x1, y1), (x2, y2))
    :param scale: scaling 缩放比例
    :return: rotated picture 旋转后的图片
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # Calculate the rotation angle and rotate picture. 计算旋转角度并旋转图片
    angle = cv2.fastAtan2(dy, dx)
    rot = cv2.getRotationMatrix2D((int(w / 2), int(h / 2)), angle, scale=scale)
    return cv2.warpAffine(image, rot, dsize=(h, w))

def perspective_transform(img, src, dst, debug=False):
    """
    perform perspective transformation: converting a skewed view of a road image taken from an oblique angle into a top-down
    view, aligning the camera's viewpoint parallel to the road. 执行透视变换：将倾斜视角拍摄到的道路图像转换成鸟瞰图，即将摄像机的视角转换到和道路平行。
    :param img: input image 输入图像
    :param src: the coordinates of the four points of the rectangle to be measured in the source image 源图像中待测矩形的四点坐标
    :param dst: the coordinates of the four points of the rectangle in the target image 目标图像中矩形的四点坐标
    """
    img_size = (img.shape[1], img.shape[0])
    # Manually extract the vertices for performing the perspective transformation. 手动提取用于执行透视变换的顶点
    '''
    # left_down
    # left_up
    # right_up
    # right_down
    src = np.float32(
        [[89, 370],
         [128, 99],
         [436, 99],
         [472, 371]])
    
    dst = np.float32(
        [[65, 430],
         [65, 55],
         [575,55],
         [575,430]])
    '''
    m = cv2.getPerspectiveTransform(src, dst)  # Calculate the perspective transformation matrix. 计算透视变换矩阵
    if debug:
        m_inv = cv2.getPerspectiveTransform(dst, src)
    else:
        m_inv = None
    # Perform perspective transformation with parameters: input image, output image, target image size, and the cv2.INTER_LINEAR
    # interpolation method. 进行透视变换 参数：输入图像、输出图像、目标图像大小、cv2.INTER_LINEAR插值方法
    warped = cv2.warpPerspective(img, m, img_size, flags=cv2.INTER_LINEAR)
    #unwarped = cv2.warpPerspective(warped, m_inv, (warped.shape[1], warped.shape[0]), flags=cv2.INTER_LINEAR)  # debugging 调试

    return warped, m, m_inv

def pixels_to_world(pixels, K, T):
    """
    Convert the pixel coordinates to world coordinates. 像素坐标转世界坐标
    Pixel coordinates list. pixels像素坐标列表
    Intrinsic camera parameters matrix K. K 相机内参 np 矩阵
    Extrinsic camera parameters matrix T. T 相机外参 np 矩阵
    """
    invK = K.I
    t, r, _, _ =  tfs.affines.decompose(np.matrix(T))
    invR = np.matrix(r).I
    R_inv_T = np.dot(invR, np.matrix(t).T)
    world_points = []
    for p in pixels:
        coords = np.float64([p[0], p[1], 1.0]).reshape(3, 1)
        cam_point = np.dot(invK, coords)
        world_point = np.dot(invR, cam_point)
        scale = R_inv_T[2][0] / world_point[2][0]
        scale_world = np.multiply(scale, world_point)
        world_point = np.array((np.asmatrix(scale_world) - np.asmatrix(R_inv_T))).reshape(-1,)
        world_points.append(world_point)
    return world_points

def world_to_pixels(world_points, K, T):
    """
    Convert 3D world coordinates to 2D pixel coordinates. 将世界坐标点转换为像素坐标
    Args:
        world_points: A list of 3D points in world coordinates. 世界坐标点列表
        K: Camera intrinsic matrix. 相机内参矩阵
        T: Camera extrinsic matrix. 外参矩阵 [R|t]
    Returns:
        pixel_points: A list of 2D pixel coordinates. 像素坐标点列表
    """
    pixel_points = []
    for wp in world_points:
        # Convert to homogeneous coordinates. 将世界坐标转换为齐次坐标
        world_homo = np.append(wp, 1).reshape(4, 1)
        # Transform to camera coordinates using extrinsic matrix. 通过外参矩阵转换到相机坐标系
        camera_point = np.dot(T, world_homo)
        # Project to image plane. 投影到像素平面
        pixel_homo = np.dot(K, camera_point[:3])
        # Normalization 归一化
        pixel = (pixel_homo / pixel_homo[2])[:2].reshape(-1)
        pixel_points.append(pixel)
    return pixel_points

def extristric_plane_shift(tvec, rmat, delta_z):
    delta_t = np.array([[0], [0], [delta_z]])
    tvec_new = tvec + np.dot(rmat, delta_t)
    return tvec_new, rmat

pixel_to_world = pixels_to_world

def ros_pose_to_list(pose):
    t = np.asarray([pose.position.x, pose.position.y, pose.position.z])
    q = np.asarray([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    return t, q

def qua2rpy(qua):
    if type(qua) == Quaternion:
        x, y, z, w = qua.x, qua.y, qua.z, qua.w
    else:
        x, y, z, w = qua[0], qua[1], qua[2], qua[3]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(np.clip(2 * (w * y - x * z), -1.0, 1.0))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def xyz_quat_to_mat(xyz, quat):
    mat = tfs.quaternions.quat2mat(np.asarray(quat))
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def xyz_rot_to_mat(xyz, rot):
    return np.row_stack((np.column_stack((rot, xyz)), np.array([[0, 0, 0, 1]])))

def xyz_euler_to_mat(xyz, euler, degrees=True):
    if degrees:
        mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
    else:
        mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def mat_to_xyz_euler(mat, degrees=True):
    t, r, _, _ = tfs.affines.decompose(mat)
    if degrees:
        euler = np.degrees(tfs.euler.mat2euler(r))
    else:
        euler = tfs.euler.mat2euler(r)
    return t, euler


