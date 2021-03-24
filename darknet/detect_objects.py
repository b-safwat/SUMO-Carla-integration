import json

from darknet2 import *
import cv2
import os
import glob


VISUALIZATION = True


from geometric_transformations import image_to_cam_coordinate, get_vector_from_vector_and_angle, \
    get_vector_from_2points, angle_between_2vectors, euclidean_dist


def proccess_img(car_world_pos_yaw, img_path, img_depth_path, cam_direction, net, meta):
    img = cv2.imread(img_path)
    r = detect(net, meta, img)

    if VISUALIZATION:
        objects_img_space = []
        objects_car_space = []

    objects_world_space = []

    camera_front_matrix = np.array([[1,  0,  0, 0.4],
                                    [0,  1,  0, 0  ],
                                    [0,  0,  1, 1.5],
                                    [0,  0,  0,  1 ]])
    yaw=(60/180) * np.pi
    y_diff = 0.5744562646538028
    camera_right_matrix = np.array([[np.cos(yaw),  -np.sin(yaw),  0,  0.33 ],
                                    [np.sin(yaw),  np.cos(yaw),   0, y_diff],
                                    [    0,             0,        1,   1.5 ],
                                    [    0,             0,        0,    1  ]])

    yaw=(300.0/180) * np.pi
    camera_left_matrix = np.array([[np.cos(yaw),  -np.sin(yaw),  0,   0.33 ],
                                   [np.sin(yaw),  np.cos(yaw),  0, -y_diff],
                                   [    0,             0,       1,    1.5 ],
                                   [    0,             0,       0,     1  ]])

    x, y, yaw = car_world_pos_yaw[1:]
    x, y, yaw = float(x), float(y), float(yaw)

    for rect in r:
        if rect[2][1] >= 450 or rect[1] < 0.5 or rect[0] != b'car':
            continue

        object_img_space = [int(rect[2][0]), int(rect[2][1] )] # + rect[2][3] / 2
        ######################################################################################################
        # For visualization
        if VISUALIZATION:
            objects_img_space.append([int(rect[2][0]), int(rect[2][1] + rect[2][3] / 2)])
            img = cv2.rectangle(img, (int(rect[2][0] - rect[2][2] / 2), int(rect[2][1] - rect[2][3] / 2)),
                                (int(rect[2][0] + rect[2][2] / 2), int(rect[2][1] + rect[2][3] / 2)), (0, 128, 0))
        ######################################################################################################
        # Camera Space
        img_d = cv2.imread(img_depth_path)
        B, G, R = img_d[:, :, 0], img_d[:, :, 1], img_d[:, :, 2]
        normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        in_meters = 1000 * normalized#/normalized.max()
        pts_camera_space = image_to_cam_coordinate(in_meters)
        # Post process the image space
        pts_camera_space = pts_camera_space[:,:,[2,0,1]] # x forward, y right, z is up
        obj_camera_space = pts_camera_space[object_img_space[1], object_img_space[0]]
        suroundings_camera_space = []
        kernel = 5
        for i in range(-kernel, kernel+1):
            for j in range(-kernel, kernel+1):
                if i == 0 and j == 0: continue
                suroundings_camera_space.append(pts_camera_space[object_img_space[1]+i, object_img_space[0]+j])

        ######################################################################################################
        # World Space
        t = np.ones(4)
        t[0:3] = obj_camera_space
        obj_camera_space = t

        if cam_direction == "front":
            cam_matrix = camera_front_matrix
        elif cam_direction == "right":
            cam_matrix = camera_right_matrix
        else:
            cam_matrix = camera_left_matrix

        obj_car_space = cam_matrix.dot(obj_camera_space)

        for i in range(len(suroundings_camera_space)):
            t = np.ones(4)
            t[0:3] = suroundings_camera_space[i]
            suroundings_camera_space[i] = t
            obj_car_space+=cam_matrix.dot(suroundings_camera_space[i])

        obj_car_space /= len(suroundings_camera_space)+1

        # return
        if VISUALIZATION:
            print(obj_car_space)
            objects_car_space.append(obj_car_space)

        car_matrix = np.array([[np.cos(yaw),  -np.sin(yaw),  0,    x],
                               [np.sin(yaw),  np.cos(yaw),   0,    y],
                               [    0,             0,        1,    0],
                               [    0,             0,        0,    1]])

        obj_world_space = car_matrix.dot(obj_car_space)
        objects_world_space.append(obj_world_space)
        ######################################################################################################

    if VISUALIZATION:
        if len(objects_world_space)!=0:
            print("point in camera coordinate: ", pts_camera_space[objects_img_space[0][1], objects_img_space[0][0]])
            print("point in car coordinate: ", objects_car_space[0])
            print("point in world coordinate: ", objects_world_space[0])

            print(in_meters[objects_img_space[0][1], objects_img_space[0][0]])
            # print("Cam pos in world", car_world_pos_yaw)
            img_d = cv2.circle(img_d, (objects_img_space[0][0], objects_img_space[0][1]), 1, (0, 255, 0), -1)
            img = cv2.circle(img, (objects_img_space[0][0], objects_img_space[0][1]), 3, (0, 255, 0), -1)
            cv2.imshow("img", img)
            cv2.imshow("depth in meter", np.log(in_meters+0.001)/ np.log(in_meters+0.001).max())
            cv2.waitKey(0)

    vec_horiz = get_vector_from_2points([x, y], [x, y+1]) # horizontal_axis
    zero_vec = get_vector_from_vector_and_angle(vec_horiz, -42)

    for i in range(len(objects_world_space)):
        objects_world_space[i] = objects_world_space[i].tolist()
        vec_obj = get_vector_from_2points([x, y], objects_world_space[i][0:2])
        ceta = angle_between_2vectors(zero_vec, vec_obj)
        dist = euclidean_dist([x, y], objects_world_space[i][:2])
        grid_pos = [ceta // 42, dist // 25]
        if grid_pos[0]>5 or grid_pos[1] > 3:
            raise ValueError("grid cannot have more than 5 and 3")
        objects_world_space[i].extend([grid_pos])

    with open(img_path.replace("png", "txt"), 'w') as json_file:
        json.dump(objects_world_space, json_file)

    return objects_world_space

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Read Car Id for Camera')
    parser.add_argument(
        '-p', '--path',
        default="/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/workplace/Car50/VehicleData_BSM/BSM_0.txt",
        type=str,
        help='working directory')

    args = parser.parse_args()

    net = load_net(b'cfg/yolov3.cfg',
                   b'cfg/yolov3.weights',
                   0)
    meta = load_meta(b"cfg/coco.data")

    # car_path = '../CARLA_PythonCode/workplace/Car50/VehicleData_BSM/BSM_0.txt'
    car_cameras_imgs_path = "/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Car50/data/0/"
    car_path = args.path
    car_num = int(car_path.split("/")[-1].split("BSM_")[1].split(".txt")[0])
    car_cameras_imgs_path = "/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Car50/data/" + str(
        car_num) + "/"
    # cars = os.listdir(car_path)
    with open(car_path) as fr:
        cars_world_pos = fr.readlines()
        cars_world_pos = [car.strip().split('\t') for car in cars_world_pos]

    c = 0

    for car_pos in cars_world_pos:
        path_front = os.path.join(car_cameras_imgs_path, "front", "rgb_" + str(car_pos[0]) + ".png")
        path_frontD = os.path.join(car_cameras_imgs_path, "frontD", "depth_" + str(car_pos[0]) + ".png")

        path_front_Left = os.path.join(car_cameras_imgs_path, "front_Left300", "rgb_" + str(car_pos[0]) + ".png")
        path_front_LeftD = os.path.join(car_cameras_imgs_path, "front_Left300D", "depth_" + str(car_pos[0]) + ".png")

        path_front_Right = os.path.join(car_cameras_imgs_path, "front_Right60", "rgb_" + str(car_pos[0]) + ".png")
        path_front_RightD = os.path.join(car_cameras_imgs_path, "front_Right60D", "depth_" + str(car_pos[0]) + ".png")

        if not os.path.isfile(path_front) or not os.path.isfile(path_frontD) \
                or not os.path.isfile(path_front_Left) or not os.path.isfile(path_front_LeftD) \
                or not os.path.isfile(path_front_Right) or not os.path.isfile(path_front_RightD):
            print("Shall not happen but about 2 or 3 times!", car_pos[0])
            continue

        c += 1

        # time.sleep(15)
        proccess_img(car_pos, path_front, path_frontD, "front", net, meta)
        proccess_img(car_pos, path_front_Left, path_front_LeftD, "left", net, meta)
        proccess_img(car_pos, path_front_Right, path_front_RightD, "right", net, meta)

if __name__ == "__main__":
   main()
   # car_pos = "32.103348500386346	158.79605102539062	34.794776916503906	-89.92001342773438".strip().split('\t')
   # car_cameras_imgs_path = "/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Car50/data/27/"
   # path_front = os.path.join(car_cameras_imgs_path, "front", "rgb_32.103348500386346.png")
   # path_frontD = os.path.join(car_cameras_imgs_path, "frontD", "depth_32.103348500386346.png")
   # path_right = os.path.join(car_cameras_imgs_path, "front_Right60", "rgb_32.103348500386346.png")
   # path_rightD = os.path.join(car_cameras_imgs_path, "front_Right60D", "depth_32.103348500386346.png")

   # proccess_img(car_pos, path_front, path_frontD, "front")
   # proccess_img(car_pos, path_right, path_rightD, "right")