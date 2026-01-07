#ros lib
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#common lib
import os
import cv2 as cv
import numpy as np
from yahboomcar_astra.common.follow_common import color_follow
from yahboomcar_astra.common.astra_common import ManyImgs


class ColorSelector(Node):
    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")

    def __init__(self, name):
        super().__init__(name)
        camera_type = os.getenv('CAMERA_TYPE', 'usb')
        topic_name = '/ascamera_hp60c/camera_publisher/rgb0/image' if camera_type == 'nuwa' else '/usb_cam/image_raw'
        self.bridge = CvBridge()
        self.selected_count = 1
        self.colors = [()] * 5
        self.finish_flag = False
        self.hsv_text = "/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_voice_ctrl/yahboomcar_voice_ctrl/colorHSV.text"

        self.cv_image = None
        cv.namedWindow('Color Selector')
        cv.setMouseCallback('Color Selector', self.on_mouse)

        self.color_tool = color_follow()
        self.Roi_init = ()
        self.select_flags = False
        self.cols, self.rows = (0, 0), (0, 0)
        self.preview_hsv = None

        self.sub_image = self.create_subscription(Image, topic_name, self.image_callback, 1)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.select_flags = True
            self.Mouse_XY = (x, y)
            self.preview_hsv = None
            self.cols = (x, y)
            self.rows = (x, y)

        elif event == cv.EVENT_MOUSEMOVE:
            if self.select_flags and self.cv_image is not None:
                img_h, img_w = self.cv_image.shape[:2]
                x = max(0, min(x, img_w - 1))
                y = max(0, min(y, img_h - 1))
                self.cols = (min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y))
                self.rows = (max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y))
                self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

        elif event == cv.EVENT_LBUTTONUP:
            self.select_flags = False
            if self.Roi_init and abs(x - self.Mouse_XY[0]) > 2 and abs(y - self.Mouse_XY[1]) > 2:
                try:
                    full_frame = self.cv_image.copy()
                    roi_frame = full_frame[self.Roi_init[1]:self.Roi_init[3], self.Roi_init[0]:self.Roi_init[2]]
                    _, fixed_hsv = self.color_tool.Roi_hsv(roi_frame, (0, 0, roi_frame.shape[1], roi_frame.shape[0]))
                    if fixed_hsv:
                        self.preview_hsv = fixed_hsv
                except Exception as e:
                    self.get_logger().error(f"HSV Error: {str(e)}")

    def process_frame(self, action):
        if self.cv_image is None:
            return

        frame = self.cv_image.copy()
        frame = cv.resize(frame, (640, 480))
        binary = []

        if self.select_flags:
            x1, y1 = max(0, self.cols[0]), max(0, self.cols[1])
            x2, y2 = min(frame.shape[1], self.rows[0]), min(frame.shape[0], self.rows[1])
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if self.preview_hsv and not self.select_flags:
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            binary = cv.inRange(hsv, self.preview_hsv[0], self.preview_hsv[1])

            hsv_max = f"MAX: HSV({self.preview_hsv[1][0]}, {self.preview_hsv[1][1]}, {self.preview_hsv[1][2]})"
            hsv_min = f"MIN: HSV({self.preview_hsv[0][0]}, {self.preview_hsv[0][1]}, {self.preview_hsv[0][2]})"

            (text_w, text_h), _ = cv.getTextSize(hsv_max, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            x_pos = (frame.shape[1] - text_w) // 2

            cv.putText(frame, hsv_max, (x_pos, 25),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv.putText(frame, hsv_min, (x_pos, 45),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        color_map = {1: "RED", 2: "GREEN", 3: "BLUE", 4: "YELLOW"}
        prefix = color_map.get(self.selected_count, "RGBY")
        status_text = f"{prefix} Color {self.selected_count}/4"
        if self.finish_flag:
            status_text = "FINISH"
        cv.putText(frame, status_text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # === 修正版核心逻辑 ===
        if action == 32:  # 空格键保存
            if not self.finish_flag and self.preview_hsv and 1 <= self.selected_count <= 4:
                try:
                    with open(self.hsv_text, 'r') as f:
                        lines = f.readlines()
                except FileNotFoundError:
                    lines = ['\n'] * 4

                lines = lines[:4]
                while len(lines) < 4:
                    lines.append('\n')

                new_line = ', '.join(map(str, self.preview_hsv[0] + self.preview_hsv[1])) + '\n'
                lines[self.selected_count - 1] = new_line

                with open(self.hsv_text, 'w') as f:
                    f.writelines(lines)

                self.colors[self.selected_count - 1] = self.preview_hsv

                # ✅ 第4次保存后才finish
                if self.selected_count == 4:
                    self.finish_flag = True
                else:
                    self.selected_count += 1

                self.preview_hsv = None
                self.Roi_init = ()

        elif action == ord('e') or action == ord('E'):
            self.selected_count += 1
            self.preview_hsv = None
            if self.selected_count > 4:
                self.selected_count = 1

        # 更新显示文字
        if self.finish_flag:
            status_text = "FINISH"
        cv.putText(frame, status_text, (30, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 显示图像
        if binary is not None and len(binary) > 0 and self.preview_hsv is not None:
            display_frame = ManyImgs(1, ([frame, cv.cvtColor(binary, cv.COLOR_GRAY2BGR)]))
        else:
            display_frame = frame

        cv.imshow('Color Selector', display_frame)


def main(args=None):
    rclpy.init(args=args)
    selector = ColorSelector("color_selector")

    try:
        while rclpy.ok():
            rclpy.spin_once(selector, timeout_sec=0.1)
            action = cv.waitKey(10) & 0xFF
            selector.process_frame(action)
            if action == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        selector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
