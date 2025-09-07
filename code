import cv2
import mediapipe as mp
import asyncio
import websockets
import math

# Setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_drawing = mp.solutions.drawing_utils

# WebSocket server details
SERVER_URI = "ws://192.168.175.232/ws"  # << your ESP32 local IP with /ws endpoint
servo_map = {'base': 0, 'shoulder': 1, 'elbow': 2, 'wrist': 3, 'claw': 4}

# States
claw_closed = False
tap_cooldown = False

def calculate_angle(p1, p2, p3):
    """Helper to calculate angle between 3 points."""
    a = ((p2[0]-p1[0])*2 + (p2[1]-p1[1])2)*0.5
    b = ((p2[0]-p3[0])*2 + (p2[1]-p3[1])2)*0.5
    c = ((p1[0]-p3[0])*2 + (p1[1]-p3[1])2)*0.5
    try:
        angle = math.acos((a*2 + b2 - c*2) / (2*a*b))
        return math.degrees(angle)
    except:
        return 0

async def main():
    async with websockets.connect(SERVER_URI) as websocket:
        cap = cv2.VideoCapture(0)

        global claw_closed, tap_cooldown

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)

            if result.multi_hand_landmarks:
                hand = result.multi_hand_landmarks[0]
                landmarks = hand.landmark

                # Wrist Position
                wrist = landmarks[0]
                wrist_x = int(wrist.x * frame.shape[1])
                wrist_y = int(wrist.y * frame.shape[0])

                # Base Movement (x only)
                base_angle = int((wrist_x / frame.shape[1]) * 180)

                # Map wrist Y position to arm servos
                # Higher Y (lower on screen) = more forward (higher angle)
                # Lower Y (higher on screen) = more backward (lower angle)
                
                # Shoulder Movement (60-160° range)
                # 110° = vertical up, 60° = backward, 160° = forward
                shoulder_angle = int((wrist_y / frame.shape[0]) * 100) + 60  # Maps to 60-160
                shoulder_angle = max(60, min(160, shoulder_angle))  # Constrain to 60-160

                # Elbow Movement (30-130° range)
                # 80° = vertical up, 130° = backward, 30° = forward
                # Inverted: higher Y (lower on screen) = forward (30°), lower Y (higher on screen) = backward (130°)
                elbow_angle = 130 - int((wrist_y / frame.shape[0]) * 100)  # Maps to 30-130
                elbow_angle = max(30, min(130, elbow_angle))  # Constrain to 30-130

                # Wrist Movement (20-200° range)
                # 110° = vertical up, 200° = forward, 20° = backward
                wrist_angle = int((wrist_y / frame.shape[0]) * 180) + 20  # Maps to 20-200
                wrist_angle = max(20, min(200, wrist_angle))  # Constrain to 20-200

                # Claw Control (open/closed based on thumb-index distance)
                thumb_tip = landmarks[4]
                index_tip = landmarks[8]
                dist = math.hypot(
                    (thumb_tip.x - index_tip.x) * frame.shape[1],
                    (thumb_tip.y - index_tip.y) * frame.shape[0]
                )

                # Claw Control Logic
                if dist < 30 and not claw_closed and not tap_cooldown:
                    # Close Claw
                    claw_closed = True
                    tap_cooldown = True
                elif dist > 60 and claw_closed and not tap_cooldown:
                    # Open Claw
                    claw_closed = False
                    tap_cooldown = True

                # Cooldown Timer
                if tap_cooldown:
                    await asyncio.sleep(0.2)  # 200ms debounce
                    tap_cooldown = False

                # Claw Angle
                claw_angle = 30 if claw_closed else 100  # 30 for closed, 100 for open

                # Prepare command
                angles = [base_angle, shoulder_angle, elbow_angle, wrist_angle, claw_angle]

                # Send each servo command
                for idx, angle in enumerate(angles):
                    cmd = f"{idx} {angle}"
                    await websocket.send(cmd)
                    await asyncio.sleep(0.01)  # small delay to not flood

                # Draw
                mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

                # Add visual feedback for all angles
                base_text = f"Base: {base_angle}° (90°=forward)"
                shoulder_text = f"Shoulder: {shoulder_angle}° (90°=up)"
                elbow_text = f"Elbow: {elbow_angle}° (90°=up)"
                wrist_text = f"Wrist: {wrist_angle}° (90°=up)"
                claw_text = f"Claw: {'Closed' if claw_closed else 'Open'}"

                cv2.putText(frame, base_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, shoulder_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, elbow_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, wrist_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, claw_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Draw line between thumb and index
                thumb_pos = (int(thumb_tip.x * frame.shape[1]), int(thumb_tip.y * frame.shape[0]))
                index_pos = (int(index_tip.x * frame.shape[1]), int(index_tip.y * frame.shape[0]))
                cv2.line(frame, thumb_pos, index_pos, (0, 255, 0), 2)

                # Add instructions
                instructions = "Move hand left/right for base | Move wrist up/down for arm | Pinch for claw"
                cv2.putText(frame, instructions, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow('Robotic Arm Control', frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
                break

        cap.release()
        cv2.destroyAllWindows()

asyncio.run(main())
