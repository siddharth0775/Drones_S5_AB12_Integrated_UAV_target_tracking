import time
import cv2
import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image


CAMERA_TOPIC = (
    "/world/default/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)


class RedTargetDetector:

    def __init__(self):
        self.node = Node()

        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0

        self.latest_frame = None

        # Subscribe to Gazebo camera
        success = self.node.subscribe(
            Image,
            CAMERA_TOPIC,
            self.camera_callback
        )

        if not success:
            raise RuntimeError("Failed to subscribe to camera topic")

        print("Camera subscription successful!")

    def camera_callback(self, msg):
        """
        Convert Gazebo Image message into an OpenCV BGR image.
        """

        try:
            width = msg.width
            height = msg.height

            # Gazebo camera is expected to provide RGB data.
            data = np.frombuffer(msg.data, dtype=np.uint8)

            # RGB image
            frame = data.reshape((height, width, 3))

            # OpenCV uses BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            self.latest_frame = frame

            self.frame_count += 1

        except Exception as e:
            print("Image conversion error:", e)

    def detect_red(self, frame):
        """
        Detect red objects using HSV thresholding.
        """

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red wraps around the HSV hue boundary,
        # so we need TWO ranges.

        lower_red_1 = np.array([0, 100, 80])
        upper_red_1 = np.array([10, 255, 255])

        lower_red_2 = np.array([170, 100, 80])
        upper_red_2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2
        )

        mask = mask1 | mask2

        # Remove small noise
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        target = None

        if contours:

            # Select largest red object
            largest = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(largest)

            # Ignore tiny detections
            if area > 100:

                x, y, w, h = cv2.boundingRect(largest)

                # Target center in pixel coordinates
                center_x = x + w // 2
                center_y = y + h // 2

                target = {
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "center_x": center_x,
                    "center_y": center_y,
                    "area": area
                }

                # Draw bounding box
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 0),
                    2
                )

                # Draw target center
                cv2.circle(
                    frame,
                    (center_x, center_y),
                    6,
                    (255, 0, 0),
                    -1
                )

                # Label
                cv2.putText(
                    frame,
                    "RED TARGET",
                    (x, max(y - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                # Center coordinates
                cv2.putText(
                    frame,
                    f"Center: ({center_x}, {center_y})",
                    (x, y + h + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

        return frame, mask, target

    def run(self):

        print("Waiting for camera frames...")
        print("Starting red target detector...")
        print("Press Q or Ctrl+C to stop.")

        last_report = time.time()

        try:

            while True:

                if self.latest_frame is None:
                    time.sleep(0.01)
                    continue

                # Copy frame so callback can continue safely
                frame = self.latest_frame.copy()

                # Detect red target
                output, mask, target = self.detect_red(frame)

                # FPS calculation
                now = time.time()

                if now - self.last_time >= 1.0:

                    self.fps = self.frame_count / (
                        now - self.last_time
                    )

                    self.frame_count = 0
                    self.last_time = now

                # Display FPS
                cv2.putText(
                    output,
                    f"FPS: {self.fps:.1f}",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )

                # Display detection status
                if target is not None:

                    text = (
                        f"TARGET DETECTED | "
                        f"X={target['center_x']} "
                        f"Y={target['center_y']} "
                        f"Area={target['area']:.0f}"
                    )

                    print(
                        f"\r{target['center_x']=} "
                        f"{target['center_y']=} "
                        f"{target['area']=:.0f}",
                        end=""
                    )

                else:

                    text = "TARGET NOT DETECTED"

                cv2.putText(
                    output,
                    text,
                    (20, output.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )

                # Show camera
                cv2.imshow(
                    "Tracker Drone - Red Target Detector",
                    output
                )

                # Show binary mask
                cv2.imshow(
                    "Red Detection Mask",
                    mask
                )

                # Quit
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    break

        except KeyboardInterrupt:
            pass

        finally:

            print("\nStopping detector...")

            cv2.destroyAllWindows()


def main():

    print("========================================")
    print(" Tracker Drone Red Target Detector")
    print("========================================")
    print()
    print("Camera topic:")
    print(CAMERA_TOPIC)
    print()

    detector = RedTargetDetector()

    detector.run()


if __name__ == "__main__":
    main()
