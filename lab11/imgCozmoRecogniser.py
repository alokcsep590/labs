#!/usr/bin/env python3



import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import numpy as np
import re
from sklearn.neighbors import KNeighborsClassifier
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
from skimage.viewer import ImageViewer
import imutils
import cv2

import asyncio
import sys
import time

from imgclassification import ImageClassifier

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
    
class ImageAnnotator(cozmo.annotate.Annotator):

    image = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)

        
async def classifyImage(robot: cozmo.robot.Robot):

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')     
    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)    
    
    robot.world.image_annotator.add_annotator('image', ImageAnnotator)
    
    await robot.set_head_angle(degrees(0)).wait_for_completed()
    print("Press CTRL-C to quit")

    try:
    
        while True:
            robot.move_lift(-1)
            #get camera image    
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
            
            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            
            ImageAnnotator.image = opencv_image
            
            opencv_images = []
            opencv_images.append(opencv_image)
            
            image_data = img_clf.extract_image_features(opencv_images)
            image_name = img_clf.predict_labels(image_data)[0]
            
            print(image_name)
            await robot.say_text(image_name).wait_for_completed()
            robot.move_lift(10)
            time.sleep(3)
        
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)

    
    
if __name__ == '__main__':
    cozmo.run_program(classifyImage, use_viewer = True, force_viewer_on_top = True)
