from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

import cv2
import numpy as np
import time


CAMERA_TOPIC = (
    "/world/default/model/x500_mono_cam_0/"
    "link/camera_link/sensor/camera/image"
)


class CameraReceiver:

    def __init__(self):
        self.node = Node()

        self.frame_count = 0
        self.last_frame = None

        print("Subscribing to Gazebo camera...")
        print("Topic:", CAMERA_TOPIC)

        # IMPORTANT:
        # subscribe(MessageType, Topic, Callback)
        success = self.node.subscribe(
            Image,
            CAMERA_TOPIC,
            self.image_callback
        )

        if not success:
            raise RuntimeError("Failed to subscribe to camera topic")

        print("Camera subscription successful!")


    def image_callback(self, msg: Image):

        try:
            width = msg.width
            height = msg.height

            data = np.frombuffer(
                msg.data,
                dtype=np.uint8
            )

            expected_size = width * height * 3

            if data.size != expected_size:
                print(
                    f"Unexpected image size: "
                    f"{data.size}, expected {expected_size}"
                )
                return

            frame = data.reshape(
                (height, width, 3)
            )

            # Gazebo camera is RGB8
            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

            self.last_frame = frame
            self.frame_count += 1

        except Exception as e:
            print("Image processing error:", e)


def main():

    camera = CameraReceiver()

    print("Waiting for camera frames...")
    print("Press Ctrl+C to stop.")

    last_count = 0

    try:

        while True:

            if camera.last_frame is not None:

                cv2.imshow(
                    "Tracker Drone Camera",
                    camera.last_frame
                )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        print("\nStopping camera receiver...")


if __name__ == "__main__":
    main()
