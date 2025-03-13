import rospy
import os
import cv2
import numpy as np
from collections import defaultdict
from sensor_msgs.msg import Image, PointStamped
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import threading

# Global variables for storing drone position
drone_position = None
position_lock = threading.Lock()

# Parameters for filtering
MAX_RELATIVE_SIZE = 0.3
MIN_ABSOLUTE_AREA = 100
MIN_CENTER_DISTANCE = 30
MAX_AGE = 15
CENTER_REGION_RATIO = 0.6
SIZE_VARIANCE = 0.3
TRACK_HISTORY = 20
APPEARANCE_THRESHOLD = 0.8

# Paths to YOLO model
weights_path = "yolov4-tiny-obj_best.weights"
config_path = "yolov4-tiny-obj.cfg"
classes = ["long_crack", "many_holes", "short_cracks", "two_holes"]

# Initialize ROS node
rospy.init_node("object_detection_node")
bridge = CvBridge()

# Initialize YOLO model
net = cv2.dnn.readNet(config_path, weights_path)
yolo_model = cv2.dnn.DetectionModel(net)
yolo_model.setInputParams(size=(256, 256), scale=1/255, swapRB=True)

# Data structures for tracking
active_objects = defaultdict(lambda: {
    'box': None,
    'age': 0,
    'history': [],
    'color_hist': None,
    'class': None,
    'position': None  # Add drone position field
})
all_objects = {}
next_track_id = 1


def calculate_color_hist(image, box):
    x1, y1, x2, y2 = map(int, box)
    roi = image[y1:y2, x1:x2]
    if roi.size == 0:
        return None
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [50, 60], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return hist


def compare_hists(hist1, hist2):
    if hist1 is None or hist2 is None:
        return 0
    return cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)


def predict_next_position(history):
    if len(history) < 2:
        return history[-1] if history else None
    dx = history[-1][0] - history[-2][0]
    dy = history[-1][1] - history[-2][1]
    return (history[-1][0] + dx, history[-1][1] + dy)


def is_near_center(center, frame_size, ratio):
    w, h = frame_size
    cx, cy = w // 2, h // 2
    region_w, region_h = w * ratio, h * ratio
    return (cx - region_w / 2 < center[0] < cx + region_w / 2 and
            cy - region_h / 2 < center[1] < cy + region_h / 2)


def object_similarity(new_obj, existing_obj, frame):
    new_area = (new_obj['box'][2] - new_obj['box'][0]) * (new_obj['box'][3] - new_obj['box'][1])
    existing_area = (existing_obj['box'][2] - existing_obj['box'][0]) * (existing_obj['box'][3] - existing_obj['box'][1])
    size_sim = 1 - abs(new_area - existing_area) / max(new_area, existing_area)

    pred_pos = predict_next_position(existing_obj['history'])
    pos_sim = 0
    if pred_pos:
        curr_pos = np.mean([new_obj['box'][:2], new_obj['box'][2:]], axis=0)
        dist = np.linalg.norm(np.array(pred_pos) - np.array(curr_pos))
        pos_sim = max(0, 1 - dist / 100)

    hist = calculate_color_hist(frame, new_obj['box'])
    app_sim = compare_hists(hist, existing_obj['color_hist'])

    return 0.4 * size_sim + 0.3 * pos_sim + 0.3 * app_sim


def position_callback(msg):
    """Callback for receiving the current drone position"""
    global drone_position
    with position_lock:
        drone_position = (msg.point.x, msg.point.y, msg.point.z)


def image_callback(msg):
    global active_objects, all_objects, next_track_id, drone_position

    # Get the current drone position
    with position_lock:
        current_position = drone_position

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    display_frame = frame.copy()
    frame_size = frame.shape[1], frame.shape[0]

    # Object detection
    class_ids, scores, boxes = yolo_model.detect(frame, 0.6, 0.4)
    current_objects = []

    for box, cls in zip(boxes, class_ids):
        x, y, w, h = box
        area = w * h
        center = (x + w / 2, y + h / 2)

        if (area > MIN_ABSOLUTE_AREA and
                area / (frame_size[0] * frame_size[1]) < MAX_RELATIVE_SIZE and
                is_near_center(center, frame_size, CENTER_REGION_RATIO)):

            current_objects.append({
                'box': [x, y, x + w, y + h],
                'class': classes[cls],
                'center': center,
                'area': area,
                'hist': calculate_color_hist(frame, [x, y, x + w, y + h])
            })

    # Group objects by class
    grouped_objects = defaultdict(list)
    for obj in current_objects:
        grouped_objects[obj['class']].append(obj)

    # Matching objects
    matched = set()
    for class_name, objects in grouped_objects.items():
        # If there are multiple objects of the same type, treat them as one
        if len(objects) > 1:
            # Calculate the average center and box for the group
            avg_center = np.mean([obj['center'] for obj in objects], axis=0)
            avg_box = [
                int(np.mean([obj['box'][0] for obj in objects])),
                int(np.mean([obj['box'][1] for obj in objects])),
                int(np.mean([obj['box'][2] for obj in objects])),
                int(np.mean([obj['box'][3] for obj in objects]))
            ]
            avg_hist = calculate_color_hist(frame, avg_box)

            # Create a new object representing the group
            group_obj = {
                'box': avg_box,
                'class': class_name,
                'center': avg_center,
                'area': sum(obj['area'] for obj in objects),
                'hist': avg_hist
            }

            # Try to match this group with existing objects
            best_match = None
            max_sim = 0

            for track_id, existing_obj in active_objects.items():
                if track_id in matched:
                    continue

                sim = object_similarity(group_obj, existing_obj, frame)
                if sim > max_sim and sim > APPEARANCE_THRESHOLD:
                    max_sim = sim
                    best_match = track_id

            if best_match:
                # Update object with drone coordinates
                active_objects[best_match].update({
                    'box': group_obj['box'],
                    'class': group_obj['class'],
                    'history': active_objects[best_match]['history'][-TRACK_HISTORY:] + [group_obj['center']],
                    'color_hist': group_obj['hist'],
                    'age': 0,
                    'position': current_position  # Add drone position
                })
                matched.add(best_match)
                rospy.loginfo(f"Updated group object: {group_obj['class']} ID: {best_match} at {current_position}")
            else:
                # Create a new object with drone coordinates
                active_objects[next_track_id] = {
                    'box': group_obj['box'],
                    'class': group_obj['class'],
                    'history': [group_obj['center']],
                    'color_hist': group_obj['hist'],
                    'age': 0,
                    'position': current_position  # Add drone position
                }
                all_objects[next_track_id] = {
                    'class': group_obj['class'],
                    'first_seen': rospy.get_time(),
                    'last_seen': rospy.get_time(),
                    'positions': [current_position]  # Save position history
                }
                rospy.loginfo(f"New group object: {group_obj['class']} ID: {next_track_id} at {current_position}")
                next_track_id += 1
        else:
            # Handle single object as before
            obj = objects[0]
            best_match = None
            max_sim = 0

            for track_id, existing_obj in active_objects.items():
                if track_id in matched:
                    continue

                sim = object_similarity(obj, existing_obj, frame)
                if sim > max_sim and sim > APPEARANCE_THRESHOLD:
                    max_sim = sim
                    best_match = track_id

            if best_match:
                # Update object with drone coordinates
                active_objects[best_match].update({
                    'box': obj['box'],
                    'class': obj['class'],
                    'history': active_objects[best_match]['history'][-TRACK_HISTORY:] + [obj['center']],
                    'color_hist': obj['hist'],
                    'age': 0,
                    'position': current_position  # Add drone position
                })
                matched.add(best_match)
                rospy.loginfo(f"Updated object: {obj['class']} ID: {best_match} at {current_position}")
            else:
                # Create a new object with drone coordinates
                active_objects[next_track_id] = {
                    'box': obj['box'],
                    'class': obj['class'],
                    'history': [obj['center']],
                    'color_hist': obj['hist'],
                    'age': 0,
                    'position': current_position  # Add drone position
                }
                all_objects[next_track_id] = {
                    'class': obj['class'],
                    'first_seen': rospy.get_time(),
                    'last_seen': rospy.get_time(),
                    'positions': [current_position]  # Save position history
                }
                rospy.loginfo(f"New object: {obj['class']} ID: {next_track_id} at {current_position}")
                next_track_id += 1

    # Update age of objects
    to_delete = [tid for tid, obj in active_objects.items() if obj['age'] > MAX_AGE]
    for tid in to_delete:
        del active_objects[tid]

    # Visualization
    for track_id, obj in active_objects.items():
        if obj['box'] is None:
            continue

        x1, y1, x2, y2 = map(int, obj['box'])
        cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(display_frame, f"{obj['class']} {track_id}",
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if "DISPLAY" in os.environ:
        cv2.imshow("Object Tracking", display_frame)
        cv2.waitKey(1)


def shutdown_hook():
    print("\nFinal statistics with coordinates:")
    print("--------------------------------")
    class_stats = {}
    unique_classes = set()  # To store unique classes

    for track_id, data in all_objects.items():
        class_name = data['class']
        positions = data.get('positions', [])
        last_position = positions[-1] if positions else None

        class_stats.setdefault(class_name, []).append((track_id, last_position))
        unique_classes.add(class_name)  # Add class to the set of unique classes

    # Print statistics for each class
    for class_name, objects in class_stats.items():
        print(f"Class: {class_name}")
        print(f"Count: {len(objects)}")
        for obj_id, pos in objects:
            if pos:
                print(f"ID: {obj_id}, Last position: (X: {pos[0]:.2f}, Y: {pos[1]:.2f}, Z: {pos[2]:.2f})")
            else:
                print(f"ID: {obj_id}, Position: unknown")
        print("--------------------------------")

    # Print the number of unique classes detected
    print(f"Total unique classes detected: {len(unique_classes)}")
    print("--------------------------------")


if __name__ == "__main__":
    # Initialize subscribers
    rospy.Subscriber("/pioneer_max_camera/image_raw", Image, image_callback)
    rospy.Subscriber("/geoscan/navigation/local/position", PointStamped, position_callback)

    rospy.on_shutdown(shutdown_hook)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()