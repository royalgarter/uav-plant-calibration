import os
import cv2
import numpy as np
from matplotlib import pyplot as plt

def contrast_stretch(im):
    """
    Performs a simple contrast stretch of the given image, from 5-100%.
    """
    in_min = np.percentile(im, 5)
    in_max = np.percentile(im, 100)

    out_min = 0.0
    out_max = 255.0

    out = im - in_min
    out *= ((out_min - out_max) / (in_min - in_max))
    out += in_min

    return out

# Read eacg Band
red = cv2.imread('sample/2.undistorted/DJI_0223.TIF', cv2.IMREAD_UNCHANGED)
nir = cv2.imread('sample/2.undistorted/DJI_0225.TIF', cv2.IMREAD_UNCHANGED)
# print('red shape:', red.shape)

# Output path
ndvi_path = 'sample/3.ndvi/'
os.makedirs('sample/3.ndvi/', exist_ok=True)

# Split 3 channels and get 1 channel
red = cv2.split(red)[0] 
nir = cv2.split(nir)[0] 

# # Change dtype to float
red, nir = red.astype(float), nir.astype(float)                   

""""""""""""""""""""""""""" VIs """""""""""""""""""""""""""""
# NDVI
ndvi_bottom = nir + red
ndvi_bottom[ndvi_bottom == 0] = 0.00001  # Make sunire we don't dividereby zero!
ndvi = (nir - red) / ndvi_bottom
ndvi[ndvi<0] = 0   # Normalize ndvi; min, max = 0, +1; to see and draw bbox
ndvi[ndvi>1] = 1
ndvi_strecth = contrast_stretch(ndvi)  # Stretch 
ndvi_strecth = ndvi_strecth.astype(np.uint8)
""""""""""""""""""""""""""" VIs """""""""""""""""""""""""""""

# Colormap: blue(+1) ~ red(0)
cmap = plt.get_cmap('jet')
fig, ax = plt.subplots(figsize=(8, 8))

plt.title('NDVI')
image_colored = cmap(ndvi_strecth)
im = ax.imshow(image_colored)
ax.axis('off')

# Save the colored image
plt.savefig(ndvi_path+'ndvi_color.TIF', bbox_inches='tight')
plt.show()

# Save image
cv2.imwrite(ndvi_path+'ndvi.TIF', ndvi)  # we can see the result in the in-house macro due to its dtype

