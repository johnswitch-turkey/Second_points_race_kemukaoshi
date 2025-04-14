import time
from unicodedata import name
import cv2
import numpy as np
#import serial
import config
from utils import get_mode,stack_images,find_vertical


def main():
    cam = cv2.VideoCapture(config.CAM_INDEX)
    # cam = cv2.VideoCapture(config.CAM_INDEX,cv2.CAP_V4L2)    适用于linxu系统
    cam.set(4,config.FRAME_HEIGHT)
    cam.set(3,config.FRAME_WIDTH)


    # ser = serial.Serial('/dev/ttyAMA0',115200)

    # 倒车检测
    reverse_count=0
    reversing=False

    start_time,end_time,diff_time=0,0,0
    detect=0

    # 切换工作模式
    mode=1
    last_mode=mode

    while True:
        new_mode = get_mode()
        if new_mode != last_mode:
            print(f"[INFO] from mode{str(last_mode)} to mode{str(new_mode)}")
            mode = new_mode
            last_mode = new_mode

            # 重置状态
            detect = 0
            reverse_count = 0
            reversing = False
            start_time = 0

        if cam.isOpened():
            frame=cam.read()[1]
            # 预处理
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            bin=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)[1]
            
            # 做一些形态学操作
            med=cv2.medianBlur(bin,5)
            kernel=np.zeros((5,5),dtype=np.uint8)
            opened=cv2.morphologyEx(med,cv2.MORPH_OPEN,kernel=kernel,iterations=5)
            closed=cv2.morphologyEx(med,cv2.MORPH_CLOSE,kernel=kernel,iterations=5)

            edges=cv2.Canny(med,50,150)
            # 把roi区域绘制出来
            h,w=edges.shape[:2]
            left_top=(int(config.ROI_LEFT_RATIO*w),int(config.ROI_TOP_RATIO*h))
            right_bottom=(int(config.ROI_RIGHT_RATIO*w),int(config.ROI_BOTTOM_RATIO*h))
            orgb=frame.copy()
            orgb=cv2.rectangle(orgb,left_top,right_bottom,(0,0,255),1)

            if mode in (1,2):
                if reversing:
                # 每检测到有效竖线后就等待一段时间
                    if diff_time>=config.DETECTION_TIME_INTERVAL:
                            flag, orgb = find_vertical(edges, orgb, config)
                            if flag:
                                detect += 1
                                start_time=time.time()
                                print(f"[DEBUG] 当前检测竖线总数：{detect}")
                    

                    # 到第4条竖线即第2，3个框交线开始倒车
                    if detect>=config.LINE_DETECTION_COUNT:
                        if mode==1:
                            pass
                            # ser.write(f{mode}.encode())
                        elif mode==2:
                            pass
                            # ser.write(f{mode}.encode())
                        start_time=0

                else:
                    if diff_time>=config.DETECTION_TIMEOUT:
                        print('[INFO]finished!')

 
                end_time=time.time()
                diff_time=end_time - start_time
            

            # 连续倒车入库
            if mode==3:
                # 完成全部倒车任务后退出
                if reverse_count>=config.MAX_REVERSE:
                    print('[INFO] finnshed!')
                    #break
                    continue
                
                # 进行倒车检测
                if not reversing:
                    if diff_time>=config.DETECTION_TIME_INTERVAL:
                        flag, orgb = find_vertical(edges, orgb)
                        if flag:
                            detect += 1
                            start_time=time.time()
                            print(f"[DEBUG] 当前检测竖线总数：{detect}")

                    # 第一次准备倒车车入库
                    if detect>=config.DETECTION_TIME_INTERVAL and reverse_count==0:
                        print('[INFO]first mission')
                        message='1'
                        # ser.write(message.encode())
                        reverse_count+=1
                        detect=0
                        reversing=True
                        start_time=time.time()

                    # 第二次倒车入库
                    if detect>=config.DETECTION_TIME_INTERVAL and reverse_count==1:
                        print('[INFO]second mission')
                        message='1'
                        # ser.write(message.encode())
                        reverse_count+=1
                        detect=0
                        reversing=True

                # 不进行倒车检测  
                else:
                    if diff_time>=config.DETECTION_TIMEOUT:
                        reversing=False
                        print('[INFO]go on!')

                end_time=time.time()
                diff_time=end_time-start_time

            bgblack=np.zeros_like(frame)
            imageArray=[[frame,bin,med,opened],[closed,edges,orgb,bgblack]]
            imgstacked=stack_images(imageArray,config.STACK_SCALE)
            cv2.imshow('stack',imgstacked)

        if cv2.waitKey(1) & 0xFF == 27:
            break


    cam.release()
    cv2.destroyAllWindows()



if __name__=="__main__":
    main()