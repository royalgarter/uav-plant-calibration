from posixpath import pardir
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as img
import pandas as pd
import os

## Radiometric calibration
# log
def empiricalLineCalibration(x, expb, a):
  x = np.double(x)
  x_cali = a*(np.exp(expb*x))  # 엑셀파일에 expb -> b 로 치환해줌
  # x_cali = a*np.exp((np.log(expb))*x)  # 엑셀파일에서 지수 계산되어서 나오기 때문
  
  # x_cali_norm = multi_normalizing(x_cali)   # [error] _src.type() == CV_8UC1 in function 'cv::equalizeHist'
  
  return x_cali

# linear
def Calibration(x, a, b):
  x = np.double(x)
  x_cali = a*x + b    
  
  print('x_cali:', x_cali)
  # x_cali_uint8 = rgb_normalizing(x_cali)  # for rgb
  # print('x_cali_uint8 : ', x_cali_uint8)
  
  return x_cali

## Normalize [0,255] : 
def normalizing(data): 
    data_norm = (data - np.min(data))/np.ptp(data) # normalize the data to 0 - 1
    data_norm2 = 255 * data_norm # Now scale by 255
    data_uint8 = data_norm2.astype(np.uint8)
    
    return data_uint8
  
###Import Data from Excel File  @ https://mizykk.tistory.com/94  @ https://freez2385.github.io/posts/Python-Pandas-read_excel/
def import_data(sheet_name, i, header, usecols):
    
    data = pd.read_excel(dn_file, sheet_name, header=header, usecols=str(usecols)) 
    # print('data : ', data)
    
    linear_list = data.values.tolist()
    # linear_list = sum(linear_list, [])
    # print('linear_list : ', linear_list)
    
    a,b = linear_list[i]
    
    '''
    global row
    row, col = data.shape
    
    key_filename, key_a, key_b = data

    # 파일 순서대로 a,b 값이 들어있는 배열을 각각 생성  # 일대일 매칭이기 때문에 index 일치
    filename = [value_filename for value_filename in data[key_filename]]
    a = [value_a for value_a in data[key_a]]
    b = [value_b for value_b in data[key_b]]
    
    # return a, b
    '''
    
    return a, b 
 

## Path
# input : undistorted images
par_dir = 'sample'  # replace
raw_dir = os.path.join(par_dir, '0.raw')  # replace
filelist = os.listdir(raw_dir)
BAND_list = ['3', '5']
red_list = [file for file in filelist if file.endswith(BAND_list[0]+'.TIF')]; red_list.sort()   # ascending order
nir_list = [file for file in filelist if file.endswith(BAND_list[1]+'.TIF')]; nir_list.sort()   # ascending order
dn_dir = os.path.join(par_dir, '1-1.dn')  # replace

# input : excel file
dn_file = os.path.join(dn_dir, 'dn.xlsx')  # replace
sheet_name_dict = {BAND_list[0]:'3;Red', BAND_list[1]:'5;NIR'}    # excel sheet name
band_dict = {BAND_list[0]:red_list, BAND_list[1]:nir_list}
# band_dict = {BAND_list[1]:nir_list}

# output : calibrated images
rcali_dir = os.path.join(par_dir, '1-2.r-calibration')  # replace
os.makedirs(rcali_dir, exist_ok=True)  # make the directory

# output : calibrated images
align_dir = os.path.join(par_dir, '1-3.aligned')  # replace
os.makedirs(rcali_dir, exist_ok=True)  # make the directory

# set Relative Optical Offsets (RED band)
relative_offset_x = 9.93750  # Replace with your actual value
relative_offset_y = 2.78125  # Replace with your actual value


## 각 행별로 일대일 매핑
for key, filelist in band_dict.items():  
  
  i = 0
      
  for file_name in filelist:
    
    file_path = os.path.join(raw_dir, file_name)
    im = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)
    
    print('file_path :', file_path)
    
    ## 1) radiometric calibration
    if key == '3':  # RED band
      # read from excel file
      a, b = import_data(sheet_name_dict[key], i, header=1, usecols='J,K')
      cali = Calibration(im, a, b)  # Bb <= exp(b)  # 엑셀파일에서 선형 계산되어서 나오기 때문
      norm = normalizing(cali)
      
      # Shift the red band to align with NIR band
      rows, cols = norm.shape
      M = np.float32([[1, 0, relative_offset_x], [0, 1, relative_offset_y]])
      # save the aligned red image
      norm_aligned = cv2.warpAffine(norm, M, (cols, rows))      
      cv2.imwrite(os.path.join(align_dir, file_name), norm_aligned) # replace
    
    elif key == '5':  # NIR band
      a, b = import_data(sheet_name_dict[key], i, header=1, usecols='J,K')
      cali = Calibration(im, a, b)
      norm = normalizing(cali)
      
      # Shift the red band to align with NIR band
      rows, cols = norm.shape
      M = np.float32([[1, 0, 0], [0, 1, 0]])
      # save the aligned red image
      norm_aligned = cv2.warpAffine(norm, M, (cols, rows))      
      cv2.imwrite(os.path.join(align_dir, file_name), norm_aligned) # replace
    
    # os.makedirs(os.path.join(rcali_dir, key), exist_ok=True)
    # out_path = os.path.join(rcali_dir, key, file_name)
    os.makedirs(os.path.join(rcali_dir), exist_ok=True)
    out_path = os.path.join(rcali_dir, file_name)
    print('out_path :', out_path)
    
    cv2.imwrite(out_path, norm)
    
    i+=1
    
  # break
  
  