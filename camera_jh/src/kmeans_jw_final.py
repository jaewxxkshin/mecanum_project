import cv2
import numpy as np
from matplotlib import pyplot as plt
import sys

src = cv2.imread('kmeans_bg.png')
src_bin =  np.zeros_like(src)
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
img_bin = np.zeros_like(gray)
if src is None:
    print('Image load failed')
    sys.exit()
    
def rgb2hsv(rgb):
    #print("rgb:", rgb) 
    r, g, b = rgb/255. 
    # Value
    v = np.max(rgb/255.)
    color = np.argmax(rgb)
    #print('color:', color)
    
    # Saturation
    m = np.min(rgb/255.)
    gap = v - m 
    if v > 0:
        s = gap/v
    else:
        s = 0.0
    
    # Hue
    if abs(gap) < sys.float_info.min:
        h = 0.
    elif color == 0: # R
        h =  0. + 60.*(g-b)/gap
        if h < 0.:
            h += 360.
    elif color == 1: # G
        h = 120. + 60.*(b-r)/gap
    elif color == 2: # B
        h = 240. + 60.*(r-g)/gap
    else:
        print('something wrong')
    
    # original 
    #return h, s, v
    # for opencv (U8)
    # 360 => (> 255),  [0, 1] => [0, 255] int 
    return int(h/2), int(255*s), int(255*v)



# 차원 변환 $ np.float32 자료형 변환
# 307200, 3
data = src.reshape((-1, 3)).astype(np.float32)

# K-means 알고리즘 -> parameter가 의미하는 바와 메서드의 동작 메커니즘 이해해야함[W]
# 최대 10번 반복하고 1픽셀 이하로 움직이면 종료
criteria = (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)


# label은 각각의 데이터가 속한 군집 정보, center은 군집의 중심점 좌표
ret, label, center = cv2.kmeans(data, 10, None, criteria, 20, cv2.KMEANS_RANDOM_CENTERS)

# 군집화 결과를 이용하여 출력 영상 생성
center = np.uint8(center)
print(center)
# 중심점 좌표를 받아서 dst에 입력 (307200, 3) 3은 중심 좌표
dst = center[label.flatten()] # 각 픽셀을 K개 군집 중심 색상으로 치환
label = label.reshape((gray.shape))
# 입력 영상과 동일한 형태로 변환 (640,480,3)
dst = dst.reshape((src.shape))

#cv2.imshow('src', src)
cv2.imshow('dst', dst)

# method 1 -> 변환하여 값 확인
# n = 1 # 1 -> blue, 4 -> red
# print(center[n, ::-1])
# print(rgb2hsv(center[n, ::-1]))
# img_bin[label == n] =255
# src_bin[label == n] = center[n]
# cv2.imshow('mask', img_bin)
# cv2.imshow('src_bin', src_bin)
# edge = cv2.Canny(img_bin, 10, 150)
# cv2.imshow('edge', edge)

# method 2 -> 현재는 무식한 방법이고 더 정갈한 코드로 가공해야함[W]
hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
lower_blue = np.array([ 100, 100, 100])
upper_blue = np.array([ 130, 255, 255])

#lower_green = np.array([ 50, 100, 100])
#upper_green = np.array([ 70, 255, 255])

lower_red = np.array([ 165, 100, 100])
upper_red = np.array([ 180, 255, 255])
# need to set new criteria[W]

mask1 = cv2.inRange(hsv, lower_red, upper_red)
mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
#mask3 = cv2.inRange(hsv, lower_green, upper_green)
cv2.imshow('red',mask1)
cv2.imshow('blue',mask2)
# cv2.imshow('green',mask3)

# method 3 -> 2와 같은 방법 bitwise연산 유무만 차이있음[W]
# Rmask = cv2.inRange(hsv, np.array([-10, 100, 100]), np.array([10, 255, 255]))
# R = cv2.bitwise_and(dst, dst, mask=Rmask)
# cv2.imshow('r', R)

# ----------------------------------------------
# Blue, Red, Green 각각 Canny or HoughTransform 메서드를 사용하여 선 정보를 검출할 수 있어야 함 
# 직선 검출

canny = cv2.Canny(mask1, 300, 500)
#cv2.imshow('Canny',canny)

lines = cv2.HoughLinesP(canny, 0.8, np.pi / 180, 90, minLineLength = 100, maxLineGap = 100)
print(lines)


# 라인 그리기
for i in lines:
    cv2.line(img_bin, (int(i[0][0]), int(i[0][1])), (int(i[0][2]), int(i[0][3])), (255, 255, 255), 3)

cv2.imshow('line_detected',img_bin)

# 직선을 유의미한 데이터로 만들기 위한 가공 -> 소스코드 C++로 변환 -> rostopic으로 발행 -> 위치 제어
cv2.waitKey()
cv2.destroyAllWindows()


