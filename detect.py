# # import os
# # import sys
# # import glob
# # import random
# # import math
# # import warnings
# # import numpy as np
# # import skimage.io
# # from PIL import Image
# # import matplotlib
# # import matplotlib.pyplot as plt
# # import argparse
# # import tensorflow as tf

# # from mrcnn import utils
# # import mrcnn.model as modellib
# # from mrcnn import visualize
# # # Import BOOK config
# # # sys.path.append(os.path.join(ROOT_DIR, "book/"))  # To find local version
# # import book

# # # Ignore depracation warnings
# # tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

# # class BookDetector():
# #     def __init__(self, root_dir="./Mask_RCNN", model_path="models/mask_rcnn_book_0999.h5"):

# #         # Root directory of the project
# #         ROOT_DIR = os.path.abspath(root_dir)

# #         # Import Mask RCNN
# #         sys.path.append(ROOT_DIR)  # To find local version of the library

# #         # Local path to trained weights file
# #         # BOOK_MODEL_PATH = os.path.join(ROOT_DIR, model_path) # REMEMBER TO CHANGE THIS
# #         BOOK_MODEL_PATH = "./models/mask_rcnn_book_0999.h5"
        
# #         # Directory to save logs and trained model
# #         MODEL_DIR = os.path.join(ROOT_DIR, "logs")
        
# #         # Load config
# #         config = book.BookConfig()
# #         # config.display()

# #         # Destination directory
# #         self.dest_dir = "/home/student/16662_RobotAutonomy/src/devel_packages/team8B/Library_Book_Detection/result"

# #         # Create model object in inference mode.
# #         self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

# #         # Load weights trained on MS-COCO
# #         self.model.load_weights(BOOK_MODEL_PATH, by_name=True)

# #     # Check presence of detections
# #     def num_instances_check(self, results):
# #         # Number of instances
# #         r = results
# #         boxes = r['rois']
# #         masks = r['masks']
# #         class_ids = r['class_ids']
# #         N = boxes.shape[0]
# #         if not N:
# #             print("\n*** No instances to display *** \n")
# #             exit()
# #         else:
# #             assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]
# #         print('{} instances detected'.format(N))
# #         return N

# #     # Model forward pass
# #     def detect(self, image):
# #         # Run detection
# #         results = self.model.detect([image], verbose=1)
# #         N = self.num_instances_check(results[0])

# #         return results, N

# #     # TODO
# #     def visualize(self, results):
# #         # Visualize results
# #         r = results[0]
# #         class_names = ['BG', 'book']
# #         visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], 
# #                                     class_names, r['scores'])


# #     # Saves images at dest_dir
# #     def save_image(self, image, original_file_path, instance_idx=0):
# #         dot_idx = original_file_path.rfind('.')
# #         slash_idx = original_file_path.rfind('/')
# #         if slash_idx == -1:
# #             tmp_name = original_file_path[:dot_idx]
# #         else:
# #             tmp_name = original_file_path[slash_idx+1:dot_idx]
# #         extension = original_file_path[dot_idx:]

# #         if instance_idx:
# #             new_file_path = os.path.join(self.dest_dir, tmp_name + '_' + str(instance_idx) + extension)
# #         else:
# #             new_file_path = os.path.join(self.dest_dir, tmp_name + '_all' + extension)

# #         image = Image.fromarray(image)
# #         image.save(new_file_path)
# #         print('Image {} saved as {}'.format(original_file_path, new_file_path))

# #     # Segment detections, show, and save if required
# #     def segment(self, image, results, image_path, all_at_once=True, show=False):
# #         """
# #         image: Input 3-channel image
# #         results: The output/detections from detect function
# #         all_at_once: If True, then ALL detected instances are returned in one image
# #                      If False, EACH detected instance is returned in an individual image
# #         """
# #         r = results[0]
# #         N = self.num_instances_check(r)
# #         mask_img = np.zeros_like(image)
# #         stitched = np.zeros((image.shape[0]*2, image.shape[1], image.shape[2]), dtype=np.uint8)

# #         for i in range(r['masks'].shape[-1]):
# #             mask_img[[r['masks'][:, :, i]]] = image[[r['masks'][:, :, i]]]
# #             stitched[0:image.shape[0], :, :] = image
# #             stitched[image.shape[0]:, :, :] = mask_img
# #             if not all_at_once:
# #                 if self.dest_dir:
# #                     self.save_image(mask_img, image_path, instance_idx=i+1)
# #                 mask_img = np.zeros_like(image)
# #                 if show:
# #                     plt.imshow(stitched)
# #                     plt.show()
# #                     plt.axis('off')

# #         if all_at_once:
# #             if self.dest_dir:
# #                 self.save_image(mask_img, image_path)
# #             if show:
# #                 plt.imshow(stitched)
# #                 plt.show()
# #                 plt.axis('off')



# # if __name__ == '__main__':

# #     # Adding arguments
# #     parser = argparse.ArgumentParser()

# #     parser.add_argument('--all_at_once', dest='all_at_once', action='store_true', help='If True, then ALL detected instances are returned in one image. \
# #                                                                                         If False, EACH detected instance is returned in an individual image')
# #     parser.add_argument('--show', dest='show', action='store_true', help='If provided, results will be shown')
# #     parser.add_argument('--image_path', dest='image_path', action='store', type=str, help='Path to image file')
# #     parser.add_argument('--image_dir', dest='image_dir', action='store', type=str, help='Directory of images')
# #     parser.add_argument('--dest_dir', dest='dest_dir', action='store', type=str, help='If provided, destination directory where output will be saved')
    

# #     # Argument parsing
# #     args = parser.parse_args()

# #     image_path = args.image_path
# #     image_dir = args.image_dir
# #     dest_dir = args.dest_dir
# #     all_at_once = args.all_at_once
# #     show = args.show

# #     if not (image_path or image_dir):
# #         print('An image path or an image directory must be provided!')
# #         exit()
    
# #     # Destination dir check
# #     if dest_dir and not os.path.isdir(dest_dir):
# #         print('Destination directory is invalid!')
# #         exit()

# #     # Instantiate BookDetector object for segmentation
# #     book_detector = BookDetector()
# #     book_detector.dest_dir = dest_dir


# #     # Note: The following snippet could be merged as one, but then 2 sequential
# #     # loops would be needed

# #     # Single image input case 
# #     if image_path and os.path.isfile(image_path):
# #         image = skimage.io.imread(image_path)
# #         if len(image.shape) == 1:
# #             image = image.reshape((image.shape[0], image.shape[1], 1))
# #             image = np.concatenate((image, image, image), axis=2)
# #         assert image.shape[2] == 3, "Image does not have 3-channels!"
        
# #         results, N = book_detector.detect(image)
# #         book_detector.segment(image, results, image_path, all_at_once=all_at_once, show=show)

# #     # Directory input case
# #     elif image_dir and os.path.isdir(image_dir):
# #         for file_path in glob.glob(os.path.join(image_dir, '*')):
# #             image = skimage.io.imread(file_path)
# #             print ("Image shape: ", image.shape)
# #             if len(image.shape) == 2:  # Grayscale image
# #                 image = skimage.color.gray2rgb(image)
# #             elif image.shape[2] == 4:  # RGBA image
# #                 image = image[..., :3]  # Remove the alpha channel

# #             results, N = book_detector.detect(image)
# #             book_detector.segment(image, results, file_path, all_at_once=all_at_once, show=show)

# #     else:
# #         print('Not a file or directory')
# #         exit()

# # print('Done!')
# import os
# import sys
# import glob
# import random
# import math
# import warnings
# import numpy as np
# import skimage.io
# import cv2
# from PIL import Image
# import matplotlib
# import matplotlib.pyplot as plt
# import argparse
# import tensorflow as tf

# from mrcnn import utils
# import mrcnn.model as modellib
# from mrcnn import visualize
# # Import BOOK config
# # sys.path.append(os.path.join(ROOT_DIR, "book/"))  # To find local version
# import book

# # Ignore depracation warnings
# tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

# class BookDetector():
#     def __init__(self, root_dir="./Mask_RCNN", model_path="models/mask_rcnn_book_0999.h5"):

#         # Root directory of the project
#         ROOT_DIR = os.path.abspath(root_dir)

#         # Import Mask RCNN
#         sys.path.append(ROOT_DIR)  # To find local version of the library

#         # Local path to trained weights file
#         # BOOK_MODEL_PATH = os.path.join(ROOT_DIR, model_path) # REMEMBER TO CHANGE THIS
#         BOOK_MODEL_PATH = "./models/mask_rcnn_book_0999.h5"
        
#         # Directory to save logs and trained model
#         MODEL_DIR = os.path.join(ROOT_DIR, "logs")
        
#         # Load config
#         config = book.BookConfig()
#         # config.display()

#         # Destination directory
#         self.dest_dir = "/home/student/16662_RobotAutonomy/src/devel_packages/team8B/Library_Book_Detection/result"

#         # Create model object in inference mode.
#         self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

#         # Load weights trained on MS-COCO
#         self.model.load_weights(BOOK_MODEL_PATH, by_name=True)

#     # Check presence of detections
#     def num_instances_check(self, results):
#         # Number of instances
#         r = results
#         boxes = r['rois']
#         masks = r['masks']
#         class_ids = r['class_ids']
#         N = boxes.shape[0]
#         if not N:
#             print("\n*** No instances to display *** \n")
#             exit()
#         else:
#             assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]
#         print('{} instances detected'.format(N))
#         return N

#     # Model forward pass
#     def detect(self, image):
#         # Run detection
#         results = self.model.detect([image], verbose=1)
#         N = self.num_instances_check(results[0])

#         return results, N

#     # Saves images at dest_dir
#     def save_image(self, image, original_file_path, instance_idx=0):
#         dot_idx = original_file_path.rfind('.')
#         slash_idx = original_file_path.rfind('/')
#         if slash_idx == -1:
#             tmp_name = original_file_path[:dot_idx]
#         else:
#             tmp_name = original_file_path[slash_idx+1:dot_idx]
#         extension = original_file_path[dot_idx:]

#         if instance_idx:
#             new_file_path = os.path.join(self.dest_dir, tmp_name + '_' + str(instance_idx) + extension)
#         else:
#             new_file_path = os.path.join(self.dest_dir, tmp_name + '_all' + extension)

#         image = Image.fromarray(image)
#         image.save(new_file_path)
#         print('Image {} saved as {}'.format(original_file_path, new_file_path))
#pip install protobuf==3.12.4

#     # Segment detections, show, and save if required
#     def segment(self, image, results, image_path, all_at_once=True, show=False):
#         """
#         image: Input 3-channel image
#         results: The output/detections from detect function
#         all_at_once: If True, then ALL detected instances are returned in one image
#                      If False, EACH detected instance is returned in an individual image
#         """
#         r = results[0]
#         N = self.num_instances_check(r)
#         mask_img = np.zeros_like(image)
#         stitched = np.zeros((image.shape[0]*2, image.shape[1], image.shape[2]), dtype=np.uint8)

#         for i in range(r['masks'].shape[-1]):
#             # Extract thpip install protobuf==3.12.4
#e mask for the current object
#             mask = r['masks'][:, :, i]

#             # Compute the centroid of the mask
#             y, x = np.where(mask)
#             center = (int(np.mean(x)), int(np.mean(y)))

#             # Draw the center point on the image
#             cv2.circle(mask_img, center, 3, (255, 0, 0), -1)  # Draw a blue circle at the center

#             mask_img[[mask]] = image[[mask]]
#             stitched[0:image.shape[0], :, :] = image
#             stitched[image.shape[0]:, :, :] = mask_img

#             if not all_at_once:
#                 if self.pip install protobuf==3.12.4
#dest_dir:
#                     self.save_image(mask_img, image_path, instance_idx=i+1)
#                 mask_img = np.zeros_like(image)
#                 if show:
#                     plt.imshow(stitched)
#                     plt.show()
#                     plt.axis('off')

#         if all_at_once:
#             if self.dest_dir:
#                 # Draw the center points on the stitched image
#                 for i in range(r['masks'].shape[-1]):
#                     mask = r['masks'][:, :, i]
#                     y, x = np.where(mask)
#                     center = (int(np.mean(x)), int(np.mean(y)))
#                     cv2.circle(stitched, center, 3, (255, 0, 0), -1)  # Draw a blue circle at the center
#                 self.save_image(stitched, image_path)
#             if show:
#                 plt.imshow(stitched)
#                 plt.show()
#                 plt.axis('off')


# if __name__ == '__main__':

#     # Adding arguments
#     parser = argparse.ArgumentParser()

#     parser.add_argument('--all_at_once', dest='all_at_once', action='store_true', help='If True, then ALL detected instances are returned in one image. \
#                                                                                         If False, EACH detected instance is returned in an individual image')
#     parser.add_argument('--show', dest='show', action='store_true', help='If provided, results will be shown')
#     parser.add_argument('--image_path', dest='image_path', action='store', type=str, help='Path to image file')
#     parser.add_argument('--image_dir', dest='image_dir', action='store', type=str, help='Directory of images')
#     parser.add_argument('--dest_dir', dest='dest_dir', action='store', type=str, help='If provided, destination directory where output will be saved')
    

#     # Argument parsing
#     args = parser.parse_args()

#     image_path = args.image_path
#     image_dir = args.image_dir
#     dest_dir = args.dest_dir
#     all_at_once = args.all_at_once
#     show = args.show

#     if not (image_path or image_dir):
#         print('An image path or an image directory must be provided!')
#         exit()
    
#     # Destination dir check
#     if dest_dir and not os.path.isdir(dest_dir):
#         print('Destination directory is invalid!')
#         exit()

#     # Instantiate BookDetector object for segmentation
#     book_detector = BookDetector()
#     book_detector.dest_dir = dest_dir


#     # Note: The following snippet could be merged as one, but then 2 sequential
#     # loops would be needed

#     # Single image input case 
#     if image_path and os.path.isfile(image_path):
#         image = skimage.io.imread(image_path)
#         if len(image.shape) == 1:
#             image = image.reshape((image.shape[0], image.shape[1], 1))
#             image = np.concatenate((image, image, image), axis=2)
#         assert image.shape[2] == 3, "Image does not have 3-channels!"
        
#         results, N = book_detector.detect(image)
#         book_detector.segment(image, results, image_path, all_at_once=all_at_once, show=show)

#     # Directory input case
#     elif image_dir and os.path.isdir(image_dir):
#         for file_path in glob.glob(os.path.join(image_dir, '*')):
#             image = skimage.io.imread(file_path)
#             print ("Image shape: ", image.shape)
#             if len(image.shape) == 2:  # Grayscale image
#                 image = skimage.color.gray2rgb(image)
#             elif image.shape[2] == 4:  # RGBA image
#                 image = image[..., :3]  # Remove the alpha channel

#             results, N = book_detector.detect(image)
#             book_detector.segment(image, results, file_path, all_at_once=all_at_once, show=show)

#     else:
#         print('Not a file or directory')
#         exit()

# print('Done!')

import os
import sys
import glob
import random
import math
import warnings
import numpy as np
import skimage.io
import cv2
from PIL import Image
import matplotlib
import matplotlib.pyplot as plt
import argparse
import tensorflow as tf

from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import visualize
# Import BOOK config
# sys.path.append(os.path.join(ROOT_DIR, "book/"))  # To find local version
import book

# Ignore depracation warnings
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

class BookDetector():
    def __init__(self, root_dir="./Mask_RCNN", model_path="models/mask_rcnn_book_0999.h5"):

        # Root directory of the project
        ROOT_DIR = os.path.abspath(root_dir)

        # Import Mask RCNN
        sys.path.append(ROOT_DIR)  # To find local version of the library

        # Local path to trained weights file
        # BOOK_MODEL_PATH = os.path.join(ROOT_DIR, model_path) # REMEMBER TO CHANGE THIS
        BOOK_MODEL_PATH = "./models/mask_rcnn_book_0999.h5"
        
        # Directory to save logs and trained model
        MODEL_DIR = os.path.join(ROOT_DIR, "logs")
        
        # Load config
        config = book.BookConfig()
        # config.display()

        # Destination directory
        self.dest_dir = "/home/student/16662_RobotAutonomy/src/devel_packages/team8B/Library_Book_Detection/result"

        # Create model object in inference mode.
        self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

        # Load weights trained on MS-COCO
        self.model.load_weights(BOOK_MODEL_PATH, by_name=True)

    # Check presence of detections
    def num_instances_check(self, results):
        # Number of instances
        r = results
        boxes = r['rois']
        masks = r['masks']
        class_ids = r['class_ids']
        N = boxes.shape[0]
        if not N:
            print("\n*** No instances to display *** \n")
            exit()
        else:
            assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]
        print('{} instances detected'.format(N))
        return N

    # Model forward pass
    def detect(self, image):
        # Run detection
        results = self.model.detect([image], verbose=1)
        N = self.num_instances_check(results[0])

        return results, N

    # Saves images at dest_dir
    def save_image(self, image, original_file_path, instance_idx=0, with_bbox=False):
        dot_idx = original_file_path.rfind('.')
        slash_idx = original_file_path.rfind('/')
        if slash_idx == -1:
            tmp_name = original_file_path[:dot_idx]
        else:
            tmp_name = original_file_path[slash_idx+1:dot_idx]
        extension = original_file_path[dot_idx:]

        if instance_idx:
            if with_bbox:
                new_file_path = os.path.join(self.dest_dir, tmp_name + '_bbox_' + str(instance_idx) + extension)
            else:
                new_file_path = os.path.join(self.dest_dir, tmp_name + '_seg_' + str(instance_idx) + extension)
        else:
            if with_bbox:
                new_file_path = os.path.join(self.dest_dir, tmp_name + '_bbox_all' + extension)
            else:
                new_file_path = os.path.join(self.dest_dir, tmp_name + '_seg_all' + extension)

        image = Image.fromarray(image)
        image.save(new_file_path)
        print('Image {} saved as {}'.format(original_file_path, new_file_path))

    # Compute the orientation of the bounding box
    def compute_orientation(self, bbox):
        """
        Compute the orientation (angle) of the bounding box
        :param bbox: Tuple containing (y1, x1, y2, x2) coordinates of the bounding box
        :return: Orientation angle in degrees
        """
        y1, x1, y2, x2 = bbox
        angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        return angle

    # Segment detections, show, and save if required
# Segment detections, show, and save if required
    def segment(self, image, results, image_path, all_at_once=True, show=False, segment_color=(0, 255, 0)):
        """
        image: Input 3-channel image
        results: The output/detections from detect function
        all_at_once: If True, then ALL detected instances are returned in one image
                    If False, EACH detected instance is returned in an individual image
        segment_color: Color for segmenting the detected instances
        """
        r = results[0]
        N = self.num_instances_check(r)
        stitched = np.zeros((image.shape[0]*2, image.shape[1], image.shape[2]), dtype=np.uint8)

        orientations = []

        for i in range(r['masks'].shape[-1]):
            # Extract the mask for the current object
            mask = r['masks'][:, :, i]

            # Compute the bounding box
            bbox = utils.extract_bboxes(np.expand_dims(mask, -1))[0]
            y1, x1, y2, x2 = bbox

            # Compute the orientation of the bounding box
            orientation = self.compute_orientation(bbox)
            orientations.append(orientation)

            # Convert image to compatible format for OpenCV
            image_cv2 = np.uint8(image)
            
            # Draw the bounding box
            cv2.rectangle(image_cv2, (x1, y1), (x2, y2), segment_color, 2)

            # Draw the center point on the image
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            cv2.circle(image_cv2, center, 3, segment_color, -1)  # Draw a circle at the center

            stitched[0:image.shape[0], :, :] = image_cv2
            stitched[image.shape[0]:, :, :] = image_cv2

            if not all_at_once:
                if self.dest_dir:
                    self.save_image(image_cv2, image_path, instance_idx=i+1, with_bbox=True)
                    self.save_image(image_cv2 * np.expand_dims(mask, -1), image_path, instance_idx=i+1, with_bbox=False)

        if all_at_once:
            if self.dest_dir:
                # Draw the center points on the stitched image
                for i in range(r['masks'].shape[-1]):
                    mask = r['masks'][:, :, i]
                    bbox = utils.extract_bboxes(np.expand_dims(mask, -1))[0]
                    y1, x1, y2, x2 = bbox
                    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    cv2.circle(stitched, center, 3, segment_color, -1)  # Draw a circle at the center

                    # Compute the orientation of the bounding box
                    orientation = self.compute_orientation(bbox)

                    # Draw the bounding box
                    cv2.rectangle(stitched, (x1, y1), (x2, y2), segment_color, 2)

                    # Draw the orientation line
                    length = max(abs(x2 - x1), abs(y2 - y1)) // 2
                    x_center = (x1 + x2) // 2
                    y_center = (y1 + y2) // 2
                    x_end = int(x_center + length * np.cos(np.radians(orientation)))
                    y_end = int(y_center + length * np.sin(np.radians(orientation)))
                    cv2.line(stitched, (x_center, y_center), (x_end, y_end), segment_color, 2)

                self.save_image(stitched, image_path, with_bbox=True)
                self.save_image(image_cv2 * r['masks'], image_path, with_bbox=False)
                
                print("Orientations of bounding boxes:", orientations)
                
            if show:
                plt.imshow(stitched)
                plt.show()
                plt.axis('off')




if __name__ == '__main__':

    # Adding arguments
    parser = argparse.ArgumentParser()

    parser.add_argument('--all_at_once', dest='all_at_once', action='store_true', help='If True, then ALL detected instances are returned in one image. \
                                                                                        If False, EACH detected instance is returned in an individual image')
    parser.add_argument('--show', dest='show', action='store_true', help='If provided, results will be shown')
    parser.add_argument('--image_path', dest='image_path', action='store', type=str, help='Path to image file')
    parser.add_argument('--image_dir', dest='image_dir', action='store', type=str, help='Directory of images')
    parser.add_argument('--dest_dir', dest='dest_dir', action='store', type=str, help='If provided, destination directory where output will be saved')
    

    # Argument parsing
    args = parser.parse_args()

    image_path = args.image_path
    image_dir = args.image_dir
    dest_dir = args.dest_dir
    all_at_once = args.all_at_once
    show = args.show

    if not (image_path or image_dir):
        print('An image path or an image directory must be provided!')
        exit()
    
    # Destination dir check
    if dest_dir and not os.path.isdir(dest_dir):
        print('Destination directory is invalid!')
        exit()

    # Instantiate BookDetector object for segmentation
    book_detector = BookDetector()
    book_detector.dest_dir = dest_dir


    # Note: The following snippet could be merged as one, but then 2 sequential
    # loops would be needed

    # Single image input case 
    # if image_path and os.path.isfile(image_path):
    #     image = skimage.io.imread(image_path)
    #     if len(image.shape) == 1:
    #         image = image.reshape((image.shape[0], image.shape[1], 1))
    #         image = np.concatenate((image, image, image), axis=2)
    #     assert image.shape[2] == 3, "Image does not have 3-channels!"
        
    #     results, N = book_detector.detect(image)
    #     book_detector.segment(image, results, image_path, all_at_once=all_at_once, show=show)

    # Directory input case
    if image_dir and os.path.isdir(image_dir):
        for file_path in glob.glob(os.path.join(image_dir, '*')):
            image = skimage.io.imread(file_path)
            print("ABC")
            print ("Image shape: ", image.shape)
            if len(image.shape) == 2:  # Grayscale image
                image = skimage.color.gray2rgb(image)
            elif image.shape[2] == 4:  # RGBA image
                image = image[..., :3]  # Remove the alpha channel

            results, N = book_detector.detect(image)
            book_detector.segment(image, results, file_path, all_at_once=all_at_once, show=show)

    else:
        print('Not a file or directory')
        exit()

print('Done!')


