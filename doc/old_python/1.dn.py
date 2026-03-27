import cv2
import numpy as np
import os


def mouse_event(event, x, y, flags, param):
    
    global num
    # 10 x 10
    y1,y2 = int(y-5), int(y+5)
    x1,x2 = int(x-5), int(x+5)
    INTERV = 40  # Replace according to the image size
    interv = 0
    
    if event == cv2.EVENT_FLAG_LBUTTON:    
    
        # Multi-spectral value 
        # average value in 10x10
        for i in range(4):
            
            # x1, x2 = x1+interv, x2+interv
            # x,y = x+interv, y+interv
            
            # roi = param[y1:y2, x1:x2]
            # b,g,r = param[y][x]
            dn = param[y][x]
            # dn = np.mean(roi) # only for single
            
            print('i : ', i)
            print('dn : ', dn)
            # print('b,g,r : ', b,g,r)  
            # print('x,y : ', x,y)  
            
            # cv2.rectangle(param, (x1, y1), (x2, y2), 255, -1)
            cv2.circle(param, (x, y), 5, 255, -1)
            
            # f.write(str(cnt)+ '\t' + filename + '\t' + str(num)  + '\t' + str(r) + '\t' + str(g) + '\t' + str(b) + '\n')
            f.write(str(dn)+ '\t')
            # f.write(str(r) + '\t' + str(g) + '\t' + str(b)+ '\t')
            
            interv = INTERV
            x = x+interv
            
            # num += 1
        
        cv2.imshow("draw", src)
 

# input 
BAND = 'b3_b5'  # 3, 5  
par_dir = 'sample'
in_dir = os.path.join(par_dir, '0.raw')
filelist = os.listdir(in_dir)
filelist = [file for file in filelist if file.endswith('3.TIF') or file.endswith('5.TIF')]
filelist.sort()        
print('filelist:', filelist)

# output 
dn_dir = os.path.join(par_dir, '1-1.dn')
os.makedirs(dn_dir, exist_ok=True)

for filename in filelist:
    
    num = 1
    
    f = open(os.path.join(dn_dir, BAND + '.txt'), 'a')
    
    filename = os.path.join(in_dir, filename)
    fname = os.path.basename(filename)
    print('fname : ', fname)
    
    src = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
    
    f.write('\n')
    f.write(fname + '\t')

    cv2.namedWindow('draw', cv2.WINDOW_NORMAL)
    cv2.imshow("draw", src)
    cv2.setMouseCallback("draw", mouse_event, src)
    cv2.waitKey()
    
    f.close()
        
cv2.destroyAllWindows()   
    
