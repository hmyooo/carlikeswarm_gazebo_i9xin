import cv2
import mediapipe as mp
import time

cap = cv2.VideoCapture("/home/rancho/1hmy/OpenFace/OpenFace/samples/multi_face.avi")

mpPose=mp.solutions.pose
pose=mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

pTime = 0


while True:
    success, img = cap.read()
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 2 = to
    results = pose.process(imgRGB)
    # print(results.multi_hand_landmarks)//检查手坐标输出
    if results.pose_landmarks:
        mpDraw.draw_landmarks(img,results.pose_landmarks,mpPose.POSE_CONNECTIONS)

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
                (255, 255, 255), 2)

    cv2.imshow("Image", img)
    cv2.waitKey(1)
