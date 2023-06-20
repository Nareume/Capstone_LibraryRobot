import cv2
import numpy as np
import math
import pytesseract

image_path = 'test2/test_data/test (118).jpg'

image = cv2.imread(image_path)
image_2 = image.copy()
h, w = image.shape[:2]
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)



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
B = 10
text = pytesseract.image_to_string(res_rotate + B, lang = 'digits')
cv2.imwrite("tilt.jpg", res_rotate+B)
print('text:',text)


