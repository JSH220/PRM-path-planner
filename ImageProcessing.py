import cv2 

class ImageProcessing(object):
    def TranformJPGto2DArray(self, ImageName):
        TwoDMatrix = cv2.imread(ImageName, 0)
        return TwoDMatrix 