import rospy
import numpy as np
import tensorflow as tf

if tf.__version__ < '1.4.0':
    raise ImportError(
        'Please upgrade your tensorflow installation to v1.4.* or later!')

NUM_CLASSES = 90


class TLExtractor(object):
    def __init__(self):
        self.graph = tf.Graph()
        self.sess = tf.Session(graph=self.graph)

        model_path = rospy.get_param('~od_path')
        path_to_ckpt = model_path + rospy.get_param('~od_model')

        rospy.loginfo('Attempt to load frozen model: ' + path_to_ckpt)
        # Load the actual model into the graph
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                rospy.loginfo('At end of TLExtractor loader')

    def _run_inference_for_single_image(self, image):
        """
        Uses pre-loaded RCNN to detect bounding boxes of objects in an image

        :param image: expects image in numpy array in RGB format
        :return:
        """
        # Get handles to input and output tensors
        tensor_dict = dict()
        tensor_dict['num_detections'] = self.graph.get_tensor_by_name('num_detections:0')
        tensor_dict['detection_boxes'] = self.graph.get_tensor_by_name('detection_boxes:0')
        tensor_dict['detection_scores'] = self.graph.get_tensor_by_name('detection_scores:0')
        tensor_dict['detection_classes'] = self.graph.get_tensor_by_name('detection_classes:0')
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
        return output_dict

    def extract_traffic_light(self, src_image):
        """
        Image is expected to be in OpenCV BGR format. Results will be returned in BGR format.

        :param src_image: expects image in numpy array in BGR format
        :return: list with extracted traffic lights or empty list if none were found
        """
        # Actual detection.
        output_dict = self._run_inference_for_single_image(src_image[..., ::-1])
        detected_tl_boxes = output_dict['detection_boxes'][
            output_dict['detection_classes'] == 10]

        traffic_lights = []
        for i, b in enumerate(detected_tl_boxes):
            x = (b[[1, 3]] * src_image.shape[1]).astype(int)
            y = (b[[0, 2]] * src_image.shape[0]).astype(int)
            traffic_lights.append(src_image[y[0]:y[1], x[0]:x[1]])

        return traffic_lights
