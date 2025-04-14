import cv2
import numpy as np
# import serial
import time
import config


# 拼接图像
def stack_images(imgArray, scale, lables=[]):
    rows = len(imgArray)
    cols = len(imgArray[0])
    row_available = isinstance(imgArray[0], list)
    width, height = imgArray[0][0].shape[1::-1]
    if row_available:
        for x in range(rows):
            for y in range(cols):
                imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0),
                                            fx=scale, fy=scale)
                if len(imgArray[x][y].shape) == 2:
                    imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
    imgBlack = np.zeros((height, width, 3), np.uint8)
    hor = [imgBlack] * rows
    for x in range(rows):
        hor[x] = np.hstack(imgArray[x])
    ver = np.vstack(hor)
    return ver


def find_vertical(edges,o):
    h,w=edges.shape[:2]
    ltop=(int(config.ROI_LEFT_RATIO*w),int(config.ROI_TOP_RATIO*h))
    rbottom=(int(config.ROI_RIGHT_RATIO*w),int(config.ROI_BOTTOM_RATIO*h))
    roi=edges[ltop[1]:rbottom[1],ltop[0]:rbottom[0]]
    rh,rw=roi.shape[:2]
    lines = cv2.HoughLinesP(roi, 1, np.pi / 180, 1, minLineLength=100, maxLineGap=60)
    # fn=0
    # n=0
    tan=1.0
    T=0.0
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 进行坐标转化
            cv2.line(o, (x1+ltop[0], y1+ltop[1]), (x2+ltop[0], y2+ltop[1]), (255, 0, 0), 5)
            # n+=2
            # fn+=x1
            # fn+=x2
            if x1!=x2:
                tan=(y1-y2)/(x1-x2)
                # 如果斜率很大判定为直线
                if abs(tan)>=config.SLOPE_VERTICAL_THRESHOLD  and abs(x1-x2)>config.MIN_LINE_WIDTH_RATI*rw:
                        T+=1
            else:
                T+1                                       
    # average=fn/n
    # delta=average-rw/2
    return T,o

# 切换工作模式
def get_mode():
    # if ser.in_waiting:     # 检测串口缓冲区是否有数据
    #     try:
    #         data = ser.read().decode('utf-8').strip()  
    #         if data in ('1', '2', '3'):
    #             print(f"get {data}")
    #             return int(data)
    #     except Exception as e:
    #         print(f"ERROR: {e}")
    # return None  

    # pc端没有serial库，在树莓派上取消注释上一行，删除下一行
    return 1