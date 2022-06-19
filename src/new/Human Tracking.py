import pyrealsense2 as rs
import numpy as np
import cv2
import tensorflow as tf

W = 848
H = 480

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)


print("[INFO] start streaming...")
pipeline.start(config)

aligned_stream = rs.align(rs.stream.color)
point_cloud = rs.pointcloud()

print("[INFO] loading model...")
PATH_TO_CKPT = r"C:\Users\harut\Desktop\Puguette_AI\src\dnn\frozen_inference_graph_coco.pb"

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.compat.v1.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.compat.v1.import_graph_def(od_graph_def, name='')
    sess = tf.compat.v1.Session(graph=detection_graph)

image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

while True:
    frames = pipeline.wait_for_frames()
    frames = aligned_stream.process(frames)
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    points = point_cloud.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)

    color_image = np.asanyarray(color_frame.get_data())
    scaled_size = (int(W), int(H))
    image_expanded = np.expand_dims(color_image, axis=0)
    (boxes, scores, classes, num) = sess.run([detection_boxes, detection_scores, detection_classes, num_detections],
                                             feed_dict={image_tensor: image_expanded})

    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes).astype(np.int32)
    scores = np.squeeze(scores)

    for idx in range(int(num)):
        class_ = classes[idx]
        score = scores[idx]
        box = boxes[idx]

        if score > 0.8 and class_ == 1: # 1 for human
            print(" [DEBUG] class : ", class_, "idx : ", idx, "num : ", num)
            left = box[1] * W
            top = box[0] * H
            right = box[3] * W
            bottom = box[2] * H

            width = right - left
            height = bottom - top
            bbox = (int(left), int(top), int(width), int(height))
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            midpoint_x = int((bbox[0] + bbox[2] + bbox[0])/2)
            midpoint_y = int((bbox[1] + bbox[3] + bbox[1])/2)

            # draw box
            cv2.rectangle(color_image, p1, p2, (255,0,0), 2, 1)
            cv2.circle(color_image,(midpoint_x, midpoint_y), radius=10, color=(255, 255, 255), thickness=-1)

            distance = round(depth_frame.get_distance(midpoint_x, midpoint_y)*100)

            # Write some Text
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (p1[0], p1[1] + 20)
            fontScale = 1
            fontColor = (255, 255, 255)
            lineType = 2
            cv2.putText(color_image, str(distance) + "cm",
                        bottomLeftCornerOfText,
                        font,
                        fontScale,
                        fontColor,
                        lineType)

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)

# Stop streaming
pipeline.stop()
