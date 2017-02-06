import numpy as np
import cv2


class Template(object):
    
    def __init__(self, tid, bbox):
        self.tid    = tid
        self.bbox   = bbox
        
    def highlite_marker(self, image, text_color=(255, 0, 0), linewidth=5):
        cv2.polylines(image, [np.int32(self.bbox)], True, 255, linewidth)
        center = 100, 100
        cv2.putText(image, str(self.tid), center, cv2.FONT_HERSHEY_SIMPLEX, 2, text_color)
