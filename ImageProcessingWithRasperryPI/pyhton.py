 from picamera2 import Picamera2
 import cv2
 import numpy as np
 import serial
 import time
 
 ser = serial.Serial('/dev/serial0', 9600, timeout=1)
 time.sleep(2)
 
 picam2 = Picamera2()
 picam2.configure(picam2.create_preview_configuration(main={"size": (320, 240)}))
 picam2.start()


 lower_mor = np.array([160, 100, 100])
 upper_mor = np.array([180, 255, 255])

 kalman = cv2.KalmanFilter(4, 2)
 kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
 kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
 np.float32)
 kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03


 prev_gray = None
 prev_points = None

 while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_mor, upper_mor)

    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    if contours:
     largest_contour = max(contours, key=cv2.contourArea)
     (x, y), radius = cv2.minEnclosingCircle(largest_contour)
     center = np.array([[np.float32(x)], [np.float32(y)]])
    
     error_x = x - frame_center_x
     error_y = y - frame_center_y
    
     if radius > 15:
      kalman.correct(center)
      prediction = kalman.predict()
      pred_x = prediction[0][0]
      pred_y = prediction[1][0]
      error_x = x - frame_center_x
      error_y = y - frame_center_y
      speed_x = prediction[2][0]
      speed_y = prediction[3][0]
    
      frame_center_x = frame.shape[1] // 2
      frame_center_y = frame.shape[0] // 2

     error_x = x - frame_center_x
     error_y = y - frame_center_y
    
     speed_x = error_x
     speed_y = error_y
    
     cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
     cv2.circle(frame, (int(prediction[0]), int(prediction[1])), 10, (255, 0, 0), 2)
     cv2.putText(frame, "Mavi Obje", (int(x) - 20, int(y) - 20),
     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
     if prev_gray is not None and prev_points is not None:
        new_points, st, err = cv2.calcOpticalFlowPyrLK(prev_gray,
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
        prev_points, None,
        winSize=(15, 15), maxLevel=2)
     if new_points is not None and st[0][0] == 1:
         dx = new_points[0][0][0] - prev_points[0][0][0]
         dy = new_points[0][0][1] - prev_points[0][0][1]

    if abs(error_x) > 5 or abs(error_y) > 5:
         mesaj = f"X:{error_x:.2f},Y:{error_y:.2f}\n"
         ser.write(mesaj.encode('utf-8'))
    
     cv2.putText(frame, f"Speed: {speed_x:.2f}, {speed_y:.2f}", (50, 50),
     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
     prev_points = new_points
    
     prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     prev_points = np.array([[x, y]], np.float32).reshape(-1, 1, 2)
    
     cv2.imshow("Original", frame)
     cv2.imshow("Blue Mask", mask)
    
     if cv2.waitKey(1) & 0xFF == ord('q'):
         break
    
     cv2.destroyAllWindows()