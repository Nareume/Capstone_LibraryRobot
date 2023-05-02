import cv2
import pytesseract
import torch
import numpy as np

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
        gray = cv2.cvtColor(label_image, cv2.COLOR_BGR2GRAY)
        label_text = pytesseract.image_to_string(gray, lang = 'digits')
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
