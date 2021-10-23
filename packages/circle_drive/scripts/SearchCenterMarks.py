import cv2.cv2 as cv2
import numpy as np

class SearchMarks:
    """
    This class is designed to search for center road markings
    """
    def __init__(self,image:np.ndarray,alpha,speed):
        """
        @param image: initial image
        @type image: uint8
        """
        self.image = image
        self.alpha = alpha
        self.speed = speed

    @staticmethod
    def convert(img:np.ndarray):
        """
        Сonverts the image to grayscale and applies blur and detector Canny

        @param img: image
        @return: processed image 
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img,(15,15),0)
        canny = cv2.Canny(blur, 50, 150)
        return canny

    @staticmethod
    def region(img:np.ndarray):
        """
        an ROI is created and superimposed on the image

        @param img: image
        @return: image with region of interest
        """
        # x1 = 100
        # x2 = 500
        # y1 = 200
        x1 = 50
        x2 = 500
        y1 = 200
        y2 = img.shape[0]
        polygons = np.array([[(x1,y2),(x2,y2),(x2,y1),(x1,y1)]])
        mask = np.zeros_like(img)
        cv2.fillPoly(mask,polygons,255)
        mask_pg = cv2.bitwise_and(img, mask)
        return mask_pg

    def search_contours(self):
        """
        This method searches contours 
        @return: image with found road markings
        """
        count = 0
        core = np.ones((3,3), np.uint8)
        frame_img = np.copy(self.image)
        canny = self.convert(frame_img)
        reg = self.region(canny)
        img_dil = cv2.dilate(reg,core, iterations = 1)
        contour, hh = cv2.findContours(img_dil, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        ind_contours = []
        for i in range(len(contour)):
            if hh[0][i][3] == -1 and hh[0][i][2] != -1 :
                ind_contours.append(i)
        for i in ind_contours:
            area = cv2.contourArea(contour[i])
            rect = cv2.boundingRect(contour[i])
            if area != 0:
                if abs(area - (rect[2]*rect[3]))/area*100 < 180:
                    x_0 = rect[2] * 0.5
                    x_c = rect[0] + x_0
                    y_c=rect[0]+rect[3]*0.5
                    print(x_c, y_c)
                    self.alpha = x_c * 0.078 - 24.96
                    self.alpha = round(self.alpha)
                    print(self.alpha)
                    angle_text = 'angle ='+str(self.alpha)
                    cv2.putText(self.image,angle_text,(50,100),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),2)
                    cv2.drawContours(self.image,contour,i,(0,0,255),3,cv2.LINE_AA)
                    self.speed = 15
                    break
            else:
                if count < 2:
                    count+=1
                    self.speed = 15
                else:
                    count = 0
                    self.speed = 0  
        return self.image, self.alpha, self.speed
