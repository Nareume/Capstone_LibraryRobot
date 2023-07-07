import cv2
import pytesseract
import torch
import numpy as np
import math

# 커스텀데이터
model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt', force_reload = True)

def detect_book_label(image):
    
    results = model(image)
    texts = []
    boxs = []
    # YOLO를 이용하여 책 라벨 검출
    

    # 검출된 책 라벨 영역 추출
    pred = results.pandas().xyxy[0]
    label_area = pred[(pred['name'] == 'label') & (pred['confidence'] > 0.5)]
    if len(label_area) == 0:
        return None, None
    for it in range(label_area.shape[0]):
    # 책 라벨 영역 이미지 추출
        xmin, ymin, xmax, ymax = label_area.iloc[it][['xmin', 'ymin', 'xmax', 'ymax']]
        label_image = (image.copy())[int(ymin):int(ymax), int(xmin):int(xmax)]
        
    # Tesseract OCR을 이용하여 텍스트 추출
        image_2 = label_image.copy()
        h, w = image.shape[:2]
        gray = cv2.cvtColor(label_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        lines = cv2.HoughLines(edges, 1, np.pi / 180,70)
        min_theta = np.pi/2
        if lines is not None:
            for line in lines:
        
                r,theta = line[0]

                if (theta < min_theta and theta > 0) :
                    min_theta = theta

                tx, ty = np.cos(theta), np.sin(theta)
                x0, y0 = tx*r, ty*r
                cv2.circle(image_2, (int(abs(x0)), int(abs(y0))), 3, (0,0,255), -1)

                x1, y1 = int(x0 + w*(-ty)), int(y0 + h*tx)
                x2, y2 = int(x0 - w*(-ty)), int(y0 - h*tx)

                cv2.line(image_2, (x1, y1), (x2, y2), (0,255,0), 1)
        ver, hor = gray.shape
        diag = int(((hor*hor + ver*ver)**0.5))
        center = int(hor/2), int(ver/2)
        degree = -math.degrees((np.pi/2)-min_theta)
        rotate = cv2.getRotationMatrix2D(center, degree, 1)
        res_rotate = cv2.warpAffine(gray, rotate, (hor, ver), borderValue= 200)

        label_text = pytesseract.image_to_string(res_rotate, lang = 'digits')
        texts.append(label_text)
        boxs.append([int(xmin), int(ymin), int(xmax), int(ymax)])
        
    return texts


cap = cv2.VideoCapture('samplev.mp4')
while True:
    # 프레임 설정
    ret, frame = cap.read()
    if not ret:
        break

    # 책 라벨 검출
    label_text, boxs = detect_book_label(frame)
    for it in range(len(label_text)):
        cv2.rectangle(frame, (boxs[it][0], boxs[it][1]), (boxs[it][2], boxs[it][3]), (255, 0, 0), 1)
        cv2.putText(frame, label_text[it], (boxs[it][0], boxs[it][1]-10), cv2.FONT_ITALIC, 0.5, (255, 0, 0), 1)
        
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
