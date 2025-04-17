import cv2
import numpy as np
# import serial
import time
import config
import globals


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


def get_edges(frame):
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    bin=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)[1]
            
    # 做一些形态学操作
    med=cv2.medianBlur(bin,5)
    edges=cv2.Canny(med,50,150)
    kernel=np.zeros((5,5),dtype=np.uint8)
    opened=cv2.morphologyEx(med,cv2.MORPH_OPEN,kernel=kernel,iterations=5)
    return opened

    
# 对图像进行概率霍夫变换处理，如果直线的斜率很大就判定为竖线
# 后期可以对非竖线进行处理，实现循迹等功能
def detect_line(edges,o):
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
            # 竖线有2种情况，斜率很大可近似看作，或者是理想的 斜率不存在
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

# 与单片机通信，解码得到的数据
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

# 识别工作模式
def identify_mode(mode):
    new_mode = get_mode()
    if new_mode!=mode:
        mode=new_mode

         # 重置状态
        globals.detect = 0
        globals.reverse_count = 0
        globals.reversing = False
        globals.start_time = 0

    return mode


def reversing_task(mode,edges,orgb):
    if mode in (1,2):
        # 如果不在执行倒车任务，就进行倒车检测
        if not globals.reversing:
        # 每检测到有效竖线后就等待一段时间
            if globals.diff_time>=config.DETECTION_TIME_INTERVAL:
                flag, orgb = detect_line(edges, orgb)
                if flag:
                    globals.detect += 1
                    globals.start_time=time.time()
                   
            # 到第4条竖线即第2，3个框交线开始倒车
            if globals.detect>=config.LINE_DETECTION_COUNT:
                    if mode==1:
                        pass
                        # ser.write(f{mode}.encode())
                    elif mode==2:
                        pass
                        # ser.write(f{mode}.encode())
                    globals.start_time=0

        # 如果在执行倒车任务，而时间间隔大于设定值，就表示倒车完毕,退出倒车任务
        else:
            if globals.diff_time>=config.REVERSING_TIMEOUT:
                return


    # 连续倒车入库
    if mode==3:
        # 完成全部倒车任务后退出
        if globals.reverse_count>=config.MAX_REVERSE:
            return  
                
        # 进行倒车检测
        if not globals.reversing:
            if globals.diff_time>=config.DETECTION_TIME_INTERVAL:
                flag, orgb = detect_line(edges, orgb)
                if flag:
                    globals.detect += 1
                    globals.start_time=time.time()
                    # print(f"[DEBUG] 当前检测竖线总数：{detect}")

            # 第一次准备倒车车入库,向单片机传输数据，并重置检测到的竖线数量，设置为正在倒车状态
            if globals.detect>=config.DETECTION_TIME_INTERVAL and globals.reverse_count==0:
                # print('[INFO]first mission')
                message='1'
                # ser.write(message.encode())
                globals.reverse_count+=1
                globals.detect=0
                globals.reversing=True
                globals.start_time=time.time()

            # 第二次倒车入库
            if globals.detect>=config.DETECTION_TIME_INTERVAL and globals.reverse_count==1:
                print('[INFO]second mission')
                message='2'
                # ser.write(message.encode())
                globals.reverse_count+=1
                globals.detect=0
                globals.reversing=True

        # 如果在执行倒车任务，而时间间隔大于设定值，就表示出库完毕,继续发车 
        else:
            if globals.diff_time>=(config.REVERSING_TIMEOUT+config.LEAVE_GARAGE_TIMEOUT):
                globals.reversing=False
                # print('[INFO]go on!')

    globals.end_time=time.time()
    globals.diff_time=globals.end_time-globals.start_time
    