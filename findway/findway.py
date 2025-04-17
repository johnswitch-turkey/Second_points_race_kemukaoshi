import time
from unicodedata import name
import cv2
import numpy as np
#import serial
import config
import utils
import globals



def main():
    cam = cv2.VideoCapture(config.CAM_INDEX)
    # cam = cv2.VideoCapture(config.CAM_INDEX,cv2.CAP_V4L2)    适用于linxu系统
    cam.set(4,config.FRAME_HEIGHT)
    cam.set(3,config.FRAME_WIDTH)

    # ser = serial.Serial('/dev/ttyAMA0',115200)

    mode = 1
    while True:
        # 检测工作模式
        mode=utils.identify_mode(mode)

        if cam.isOpened():
            frame=cam.read()[1]
            # 得到canny边缘
            edges = utils.get_edges(frame)
            # 把roi区域绘制出来
            h,w=edges.shape[:2]
            left_top=(int(config.ROI_LEFT_RATIO*w),int(config.ROI_TOP_RATIO*h))
            right_bottom=(int(config.ROI_RIGHT_RATIO*w),int(config.ROI_BOTTOM_RATIO*h))
            orgb=frame.copy()
            orgb=cv2.rectangle(orgb,left_top,right_bottom,(0,0,255),1)

            # 执行倒车任务
            utils.reversing_task(mode,edges,orgb)
            #bgblack=np.zeros_like(frame)
            #imageArray=[[frame,bin,med,opened],[closed,edges,orgb,bgblack]]
            imageArray=[[frame,edges,orgb]]
            imgstacked=utils.stack_images(imageArray,config.STACK_SCALE)
            cv2.imshow('stack',imgstacked)

        if cv2.waitKey(1) & 0xFF == 27:
            break


    cam.release()
    cv2.destroyAllWindows()



if __name__=="__main__":
    main()