import cv2
import numpy as np

def cal_iou(b1,b2):
    width = 608
    height = 608

    background1 = np.zeros((width,height))
    background2 = np.zeros((width,height))

    box1_coord = b1
    box2_coord = b2

    box1 = cv2.fillConvexPoly(background1,box1_coord,1)
    box2 = cv2.fillConvexPoly(background2,box2_coord,1)

    inter = cv2.countNonZero(cv2.bitwise_and(box1,box2))
    union = cv2.countNonZero(cv2.bitwise_or(box1,box2))
    print(union)
    print(inter)
    
    iou = inter/union
    
    return iou
    # cv2.imshow("a",cv2.bitwise_and(box1,box2))
    # cv2.imshow("a1",cv2.bitwise_or(box1,box2))

    # cv2.waitKey(0)