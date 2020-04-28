import cv2
import glob
import os
#
#img_mask = '../images/*.png'
path = '../img_backup'
count = 900
#img_names = glob(img_mask)
#for fn in img_names:
#    print('processing %s...' % fn,)
#    img = cv2.imread(fn, 0)
#    cv2.imwrite(os.path.join(path , 'log4_.jpg'+ str(count + 1)) + '.png', img)
#    count = count + 1
filenames = [img for img in glob.glob("../img_backup/*.png")]

filenames.sort(key=lambda f: int(filter(str.isdigit, f))) # ADD THIS LINE
# saving a video
fourcc = cv2.cv.CV_FOURCC(*'MJPG')
out = cv2.VideoWriter('output_logdata1.avi',fourcc, 20.0, (800,800))

images = []
for img in filenames:
    n= cv2.imread(img, 1)
    #cv2.imwrite(os.path.join(path , 'log4_.jpg'+ str(count + 1)) + '.png', n)
    out.write(n)
    count = count + 1
    images.append(n)
    print (img)

out.release()
