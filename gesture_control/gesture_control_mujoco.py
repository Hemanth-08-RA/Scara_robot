#!/usr/bin/env python3

import os
import time
import cv2
import numpy as np
import mujoco
import mujoco.viewer
import mediapipe as mp


def get_scene_xml_path():
    return "/home/robotics/scara_description_ws/urdf/scara_scene_mujoco.xml"

def main():

    print("""
============================================================
        SCARA SIMPLE ONE-HAND GESTURE CONTROL
============================================================

  🖐 OPEN HAND  = MOVE ROBOT
  ✊ FIST       = HOLD / STOP
  🤏 PINCH      = CLOSE GRIPPER

  HAND MOVEMENT:
    LEFT / RIGHT  → BASE ROTATION
    UP / DOWN     → FOREARM ROTATION
    HAND SIZE     → Z-AXIS

  Gripper:
    🤏 PINCH      → CLOSE
    🖐 OPEN       → OPEN

  Q / ESC        → EXIT
============================================================
""")

    # ---------------------------------------------------------
    # Load MuJoCo
    # ---------------------------------------------------------

    xml_path = get_scene_xml_path()

    print("Loading MuJoCo model from:")
    print(xml_path)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # ---------------------------------------------------------
    # MediaPipe
    # ---------------------------------------------------------

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils

    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    )

    # ---------------------------------------------------------
    # Camera
    # ---------------------------------------------------------

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[ERROR] Camera could not be opened.")
        return

    # ---------------------------------------------------------
    # Robot targets
    # ---------------------------------------------------------

    base_target = 0.0
    z_target = 0.0
    forearm_target = 0.0
    wrist_target = 0.0

    gripper_target = -0.05

    # Robot limits
    BASE_MIN = -2.0
    BASE_MAX = 2.0

    Z_MIN = -0.15
    Z_MAX = 0.02

    FOREARM_MIN = -2.0
    FOREARM_MAX = 2.0

    WRIST_MIN = -4.71
    WRIST_MAX = 4.71

    GRIP_OPEN = -0.05
    GRIP_CLOSED = 0.0

    # ---------------------------------------------------------
    # Hand tracking variables
    # ---------------------------------------------------------

    previous_x = None
    previous_y = None
    previous_size = None

    # Smoothing
    smooth_x = None
    smooth_y = None
    smooth_size = None

    alpha = 0.25

    # Movement sensitivity
    BASE_SPEED = 0.035
    FOREARM_SPEED = 0.035
    Z_SPEED = 0.002

    # ---------------------------------------------------------
    # Start MuJoCo viewer
    # ---------------------------------------------------------

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

            ret, frame = cap.read()

            if not ret:
                continue

            # Mirror camera
            frame = cv2.flip(frame, 1)

            h, w, _ = frame.shape

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = hands.process(rgb)

            status = "NO HAND"
            status_color = (180, 180, 180)

            if results.multi_hand_landmarks:

                hand = results.multi_hand_landmarks[0]

                mp_draw.draw_landmarks(
                    frame,
                    hand,
                    mp_hands.HAND_CONNECTIONS
                )

                lm = hand.landmark

                # -------------------------------------------------
                # Palm center
                # -------------------------------------------------

                palm_x = (
                    lm[0].x +
                    lm[5].x +
                    lm[9].x +
                    lm[13].x +
                    lm[17].x
                ) / 5.0

                palm_y = (
                    lm[0].y +
                    lm[5].y +
                    lm[9].y +
                    lm[13].y +
                    lm[17].y
                ) / 5.0

                # -------------------------------------------------
                # Hand size = distance wrist to middle finger
                # Used as approximate depth
                # -------------------------------------------------

                dx = lm[12].x - lm[0].x
                dy = lm[12].y - lm[0].y

                hand_size = np.sqrt(dx * dx + dy * dy)

                # -------------------------------------------------
                # Smooth hand position
                # -------------------------------------------------

                if smooth_x is None:
                    smooth_x = palm_x
                    smooth_y = palm_y
                    smooth_size = hand_size

                smooth_x = (
                    alpha * palm_x +
                    (1 - alpha) * smooth_x
                )

                smooth_y = (
                    alpha * palm_y +
                    (1 - alpha) * smooth_y
                )

                smooth_size = (
                    alpha * hand_size +
                    (1 - alpha) * smooth_size
                )

                # -------------------------------------------------
                # Detect pinch
                # -------------------------------------------------

                thumb = lm[4]
                index = lm[8]

                pinch_distance = np.sqrt(
                    (thumb.x - index.x) ** 2 +
                    (thumb.y - index.y) ** 2
                )

                pinch = pinch_distance < 0.055

                # -------------------------------------------------
                # Detect fist
                # -------------------------------------------------

                fingertips = [8, 12, 16, 20]
                pips = [6, 10, 14, 18]

                folded = 0

                for tip, pip in zip(fingertips, pips):

                    if lm[tip].y > lm[pip].y:
                        folded += 1

                fist = folded >= 3 and not pinch

                # -------------------------------------------------
                # GRIPPER
                # -------------------------------------------------

                if pinch:

                    gripper_target = GRIP_CLOSED

                    status = "PINCH → GRIPPER CLOSE"
                    status_color = (0, 0, 255)

                elif not fist:

                    gripper_target = GRIP_OPEN

                    status = "OPEN HAND → MOVE"
                    status_color = (0, 255, 0)

                else:

                    status = "FIST → HOLD"
                    status_color = (0, 255, 255)

                # -------------------------------------------------
                # Robot movement
                # -------------------------------------------------

                if not fist and not pinch:

                    # ---------------------------------------------
                    # LEFT / RIGHT → BASE
                    # ---------------------------------------------

                    if previous_x is not None:

                        movement_x = smooth_x - previous_x

                        if abs(movement_x) > 0.003:

                            base_target += (
                                -movement_x * 2.0
                            )

                    # ---------------------------------------------
                    # UP / DOWN → FOREARM
                    # ---------------------------------------------

                    if previous_y is not None:

                        movement_y = smooth_y - previous_y

                        if abs(movement_y) > 0.003:

                            forearm_target += (
                                -movement_y * 2.0
                            )

                    # ---------------------------------------------
                    # HAND SIZE → Z AXIS
                    # ---------------------------------------------

                    if previous_size is not None:

                        size_change = (
                            smooth_size -
                            previous_size
                        )

                        if abs(size_change) > 0.002:

                            z_target += (
                                -size_change * 0.5
                            )

                    # Clamp
                    base_target = float(
                        np.clip(
                            base_target,
                            BASE_MIN,
                            BASE_MAX
                        )
                    )

                    forearm_target = float(
                        np.clip(
                            forearm_target,
                            FOREARM_MIN,
                            FOREARM_MAX
                        )
                    )

                    z_target = float(
                        np.clip(
                            z_target,
                            Z_MIN,
                            Z_MAX
                        )
                    )

                # -------------------------------------------------
                # Save previous values
                # -------------------------------------------------

                previous_x = smooth_x
                previous_y = smooth_y
                previous_size = smooth_size

                # -------------------------------------------------
                # Display values
                # -------------------------------------------------

                cv2.rectangle(
                    frame,
                    (10, 10),
                    (w - 10, 180),
                    (30, 30, 30),
                    -1
                )

                cv2.putText(
                    frame,
                    "SCARA SIMPLE HAND CONTROL",
                    (25, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    status,
                    (25, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    status_color,
                    2
                )

                cv2.putText(
                    frame,
                    f"BASE    : {base_target:+.2f}",
                    (25, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1
                )

                cv2.putText(
                    frame,
                    f"FOREARM : {forearm_target:+.2f}",
                    (200, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1
                )

                cv2.putText(
                    frame,
                    f"Z       : {z_target:+.3f}",
                    (25, 140),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1
                )

                cv2.putText(
                    frame,
                    f"GRIP    : {'CLOSED' if gripper_target == 0.0 else 'OPEN'}",
                    (200, 140),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1
                )

            else:

                previous_x = None
                previous_y = None
                previous_size = None

                cv2.putText(
                    frame,
                    "NO HAND DETECTED",
                    (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

            # -----------------------------------------------------
            # Send commands to MuJoCo
            # -----------------------------------------------------

            data.ctrl[0] = base_target
            data.ctrl[1] = z_target
            data.ctrl[2] = forearm_target
            data.ctrl[3] = wrist_target
            data.ctrl[4] = gripper_target
            data.ctrl[5] = gripper_target

            # -----------------------------------------------------
            # Step physics
            # -----------------------------------------------------

            mujoco.mj_step(model, data)

            viewer.sync()

            cv2.imshow(
                "SCARA Gesture Control",
                frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q"):
                break

            time.sleep(model.opt.timestep)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
