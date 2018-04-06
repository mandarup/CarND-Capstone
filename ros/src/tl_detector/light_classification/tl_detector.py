import logging
import sys

import numpy as np
import tensorflow as tf

logging.basicConfig(level=logging.DEBUG,
                    format='[%(asctime)s] %(levelname)s in %(module)s: %(message)s')

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("/opt/tensorflow_models/research/")
sys.path.append("/opt/tensorflow_models/research/slim")
from object_detection.utils import ops as utils_ops

if tf.__version__ < '1.4.0':
    raise ImportError(
        'Please upgrade your tensorflow installation to v1.4.* or later!')

# What model do we want to use
MODEL_NAME = 'faster_rcnn_inception_resnet_v2_atrous_lowproposals_coco_2018_01_28'

# Path to frozen detection graph. This is the actual model that is used
# for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = '/opt/tensorflow_models/research/object_detection/data/mscoco_label_map.pbtxt'

NUM_CLASSES = 90


class TLDetector(object):
    def __init__(self):
        self.graph = tf.Graph()
        self.sess = tf.Session(graph=self.graph)

        # Load the actual model into the graph
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def run_inference_for_single_image(self, image):
        # Get handles to input and output tensors
        ops = self.graph.get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        tensor_dict = {}
        for key in [
            'num_detections', 'detection_boxes', 'detection_scores',
            'detection_classes', 'detection_masks'
        ]:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                tensor_dict[key] = self.graph.get_tensor_by_name(
                    tensor_name)
        if 'detection_masks' in tensor_dict:
            # The following processing is only for single image
            detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
            detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
            # Reframe is required to translate mask from box coordinates to image
            # coordinates and fit the image size.
            real_num_detection = tf.cast(
                tensor_dict['num_detections'][0], tf.int32)
            detection_boxes = tf.slice(
                detection_boxes, [0, 0], [real_num_detection, -1])
            detection_masks = tf.slice(
                detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                detection_masks, detection_boxes, image.shape[0], image.shape[1])
            detection_masks_reframed = tf.cast(
                tf.greater(detection_masks_reframed, 0.5), tf.uint8)
            # Follow the convention by adding back the batch dimension
            tensor_dict['detection_masks'] = tf.expand_dims(
                detection_masks_reframed, 0)
        image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

        # Run inference
        output_dict = self.sess.run(tensor_dict,
                                    feed_dict={image_tensor: np.expand_dims(image, 0)})

        # all outputs are float32 numpy arrays, so convert types as appropriate
        output_dict['num_detections'] = int(output_dict['num_detections'][0])
        output_dict['detection_classes'] = output_dict[
            'detection_classes'][0].astype(np.uint8)
        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
        output_dict['detection_scores'] = output_dict['detection_scores'][0]
        if 'detection_masks' in output_dict:
            output_dict['detection_masks'] = output_dict['detection_masks'][0]
        return output_dict

    def extract_traffic_light(self, src_image):
        """
        Image is expected to be in OpenCV BGR format. Results will be returned in BGR format.

        :param src_image:
        :return:
        """
        # Actual detection.
        output_dict = self.run_inference_for_single_image(src_image[..., ::-1])
        detected_tl_boxes = output_dict['detection_boxes'][
            output_dict['detection_classes'] == 10]

        traffic_lights = []
        for i, b in enumerate(detected_tl_boxes):
            x = (b[[1, 3]] * src_image.shape[1]).astype(int)
            y = (b[[0, 2]] * src_image.shape[0]).astype(int)
            traffic_lights.append(src_image[y[0]:y[1], x[0]:x[1]])

        return traffic_lights
