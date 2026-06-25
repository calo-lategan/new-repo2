import cv2
import math
import numpy as np
import transforms3d as tfs


def distance(point_1, point_2 ):
    """
    Calculate the Euclidean distance between two points. 计算两个点间欧氏距离
    :param point_1: point1 点1
    :param point_2: point2 点2
    :return: Distance between two points 两点间的距离
    """
    if len(point_1) != len(point_2):
        raise ValueError("两点的维度不一致")
    return math.sqrt(sum([(point_2[i] - point_1[i]) ** 2 for i in range(len(point_1))]))


def box_center(box):
    """
    Calculate the center of a quadrilateral box. 计算四边形box的中心
    :param box: Box （x1, y1, x2, y2) form. box （x1, y1, x2, y2)形式
    :return:  Center coordinate（x, y). 中心坐标（x, y)
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def bgr8_to_jpeg(value, quality=75):
    """
    Convert cv bgr8 format data to jpg format. 将cv bgr8格式数据转换为jpg格式
    :param value: Original data 原始数据
    :param quality:  jpg quality the maximal value is 100. jpg质量 最大值100
    :return:
    """
    return bytes(cv2.imencode('.jpg', value)[1])


def point_remapped(point, now, new, data_type=float):
    """
    Mapping the coordinates of a point from one image size to another. 将一个点的坐标从一个图片尺寸映射的新的图片上
    :param point: The coordinates of point. 点的坐标
    :param now: The current image size. 现在图片的尺寸
    :param new: The new image size. 新的图片尺寸
    :return: New point coordinates. 新的点坐标
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)


def get_area_max_contour(contours, threshold=100):
    """
    Get the most significant contour in terms of area, filtering out cases with areas that are too small. 获取轮廓中面积最重大的一个, 过滤掉面积过小的情况
    :param contours: Contour list 轮廓列表
    :param threshold: Contour threshold, contours with an area smaller than this value will be filtered out. 面积阈值, 小于这个面积的轮廓会被过滤
    :return: If the area of the largest contour is greater than a threshold, return the largest contour; otherwise, return None. 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None
    """
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None


def vector_2d_angle(v1, v2):
    """
    Calculate the angle between two vectors, ranging from -π to π. 计算两向量间的夹角 -pi ~ pi
    :param v1: The first vector 第一个向量
    :param v2: The second vector 第二个向量
    :return: Angle 角度
    """
    norm_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (norm_v1_v2)
    sin = np.cross(v1, v2) / (norm_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def warp_affine(image, points, scale=1.0):
    """
    Aligning simply, calculating the angle of the line between two points, rotating the image around its center to make the line horizontal. 简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平
    This can be applied to facial alignment. 可以用在人脸对齐上

    :param image: The facial images to be selected. 要选择的人脸图片
    :param points: The coordinates of two points: ((x1, y1), (x2, y2)). 两个点的坐标 ((x1, y1), (x2, y2))
    :param scale: The scaling ratio. 缩放比例
    :return: The rotated image. 旋转后的图片
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # Calculate rotation angle and rotate the image. 计算旋转角度并旋转图片
    angle = cv2.fastAtan2(dy, dx)
    rot = cv2.getRotationMatrix2D((int(w / 2), int(h / 2)), angle, scale=scale)
    return cv2.warpAffine(image, rot, dsize=(h, w))


def perspective_transform(img, src, dst, debug=False):
    """
    Perform perspective transformation: converting a skewed view of a road image taken from an oblique angle into a top-down view, aligning the camera's viewpoint parallel to the road. 执行透视变换：将倾斜视角拍摄到的道路图像转换成鸟瞰图，即将摄像机的视角转换到和道路平行。
    :param img: Input image 输入图像
    :param src: The coordinates of the four points of the rectangle to be measured in the source image. 源图像中待测矩形的四点坐标
    :param dst: The coordinates of the four points of the rectangle in the target image. 目标图像中矩形的四点坐标
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
    # Perform perspective transformation with parameters: input image, output image, target image size, and the cv2.INTER_LINEAR interpolation method. 进行透视变换 参数：输入图像、输出图像、目标图像大小、cv2.INTER_LINEAR插值方法
    warped = cv2.warpPerspective(img, m, img_size, flags=cv2.INTER_LINEAR)
    #unwarped = cv2.warpPerspective(warped, m_inv, (warped.shape[1], warped.shape[0]), flags=cv2.INTER_LINEAR)  # debugging 调试

    return warped, m, m_inv



def pixels_to_world(pixels, K, T):
    """
    Convert the pixel coordinates to world coordinates. 像素坐标转世界坐标
    Pixel coordinates list. pixels 像素坐标列表
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

def extristric_plane_shift(tvec, rmat, delta_z):
    delta_t = np.array([[0], [0], [delta_z]])
    tvec_new = tvec + np.dot(rmat, delta_t)
    return tvec_new, rmat

pixel_to_world = pixels_to_world

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

def ros_pose_to_list(pose):
    t = np.asarray([pose.position.x, pose.position.y, pose.position.z])
    q = np.asarray([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    return t, q


    
def draw_tags(image, tags, corners_color=(0, 125, 255), center_color=(0, 255, 0)):
    for tag in tags:
        corners = tag.corners.astype(int)
        center = tag.center.astype(int)
        cv2.putText(image, "%d"%tag.tag_id, (int(center[0] - (7 * len("%d"%tag.tag_id))), int(center[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if corners_color is not None:
            for p in corners:
                cv2.circle(image, tuple(p.tolist()), 5, corners_color, -1)
        if center_color is not None:
            cv2.circle(image, tuple(center.tolist()), 8, center_color, -1)
    return image