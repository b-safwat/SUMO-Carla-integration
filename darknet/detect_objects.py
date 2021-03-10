import sys
sys.path.append("../")
from darknet import *
import cv2
import os
import glob

if __name__ == "__main__":
    # net = load_net("cfg/densenet201.cfg", "/home/pjreddie/trained/densenet201.weights", 0)
    # im = load_image("data/wolf.jpg", 0, 0)
    # meta = load_meta("cfg/imagenet1k.data")
    # r = classify(net, meta, im)
    # print r[:10]
    net = load_net(b'cfg/yolov3.cfg',
                   b'yolov3.weights',
                   0)
    meta = load_meta(b"cfg/coco.data")
    r = detect(net, meta, b"cfg/dog.jpg")
    print(r)

    # img = cv2.imread("cfg/dog.jpg")
    # img = cv2.rectangle(img, (int(r[0][2][0]-r[0][2][2]/2), int(r[0][2][1]-r[0][2][3]/2)),
    #                     (int(r[0][2][0]+r[0][2][2]/2), int(r[0][2][1]+r[0][2][3]/2)), (0, 128, 0))
    # cv2.imshow("", img)
    # cv2.waitKey(0)

    car_path = '../Car50/data/'
    cars = os.listdir(car_path)

    for car in cars:
        path_front = os.path.join(car_path, car, "front")
        path_front_Left = os.path.join(car_path, car, "front_Left300")
        path_front_Right = os.path.join(car_path, car, "front_Right60")
        view_dirs = [path_front, path_front_Left, path_front_Right]

        for view in view_dirs:
            frames = glob.glob(view+"/*png")

