import cv2.cv2 as cv2
import time
import os
import inspect
from SearchCenterMarks import SearchMarks

if __name__ == "__main__":
    while(True):
        ret, frame = cap.read()

        mark_road = SearchMarks(frame,0,0)
        result_img, alpha, speed = mark_road.search_contours()
