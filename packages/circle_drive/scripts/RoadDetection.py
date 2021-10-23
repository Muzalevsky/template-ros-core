import cv2.cv2 as cv2
import time
import os
import logging
import inspect
from SearchCenterMarks import SearchMarks
from USB import SerialUsb

if __name__ == "__main__":
    log = logging.getLogger(__name__)
    usb = SerialUsb('COM4',115200)
    dirname=(os.path.dirname(inspect.getfile(inspect.currentframe())))
    name_1 = 'EV_zawrka_2_left_line.mp4'
    name_2 = 'EV_zawrka_1_right_line.mp4'
    name_3 = 'EV_zawrka_3_center.mp4'
    # cap = cv2.VideoCapture(os.path.join(dirname,'video',name_1))
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        log.warning("Video doesn't open")
        exit()
    while(True):
        ret, frame = cap.read()
        if cv2.waitKey(1) & 0xFF == ord('q') or ret == False:
            break
        mark_road = SearchMarks(frame,0,0)
        result_img, alpha, speed = mark_road.search_contours()
        result = usb.send(100,alpha,speed)
        cv2.imshow('frame',result_img)
        time.sleep(0.01)
    cap.release()
    cv2.destroyAllWindows()
    usb.close_serial()





