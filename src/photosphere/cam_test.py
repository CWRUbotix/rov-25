import cv2


def set_dims(vid: cv2.VideoCapture, width: int, height: int) -> None:
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, height)


# Size: Discrete 800x600
#     Interval: Discrete 0.033s (30.000 fps)
# Size: Discrete 3840x2160
#     Interval: Discrete 0.033s (30.000 fps)
# Size: Discrete 3840x2880
#     Interval: Discrete 0.067s (15.000 fps)
# Size: Discrete 416x312
#     Interval: Discrete 0.033s (30.000 fps)
# Size: Discrete 640x480
#     Interval: Discrete 0.033s (30.000 fps)
# Size: Discrete 1280x720
#     Interval: Discrete 0.033s (30.000 fps)
# Size: Discrete 1920x1080
#     Interval: Discrete 0.033s (30.000 fps)
# Size: Discrete 3840x3032
#     Interval: Discrete 0.067s (15.000 fps)


vid = cv2.VideoCapture('/dev/video0')
# vid.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
# vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 3032)
# print(vid.get(cv2.CAP_PROP_FOURCC))
# vid.set(cv2.CAP_PROP_FOURCC, '')
mjpg = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
yuyv = cv2.VideoWriter.fourcc('Y', 'U', 'Y', 'V')
print('MJPG ==', mjpg)
print('YUYV ==', yuyv)
print('cam ==', vid.get(cv2.CAP_PROP_FOURCC))
vid.set(cv2.CAP_PROP_FOURCC, mjpg)
print('cam ==', vid.get(cv2.CAP_PROP_FOURCC))
set_dims(vid, 3840, 3032)
# vid.set(cv2.CAP_PROP_MODE, 0)

while vid.isOpened():
    img, frame = vid.read()
    frame = cv2.resize(frame, (960, 758), interpolation=cv2.INTER_LINEAR)

    cv2.imshow('TRANSMITTING VIDEO', frame)
    key = cv2.waitKey(1) & 0xFF
