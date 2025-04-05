from dataclasses import dataclass
import cv2
import math
import numpy as np

@dataclass
class Fisheye_Image:
    img: np.ndarray
    img_num: int
    left: int
    top: int
    diameter: int


# The dimensions of the output image in width by height
output_dimension = (2000, 1000)

# The apeture of the fisheyes in radians
apeture = 195 * math.pi / 180

# The fisheye images with their information
fisheye_images = [
    Fisheye_Image(
        img = cv2.imread('src/surface/photosphere/fisheye1.jpg'), 
        img_num = 0, 
        left = 400, 
        top = 28, 
        diameter = 3052), 
    Fisheye_Image(
        img = cv2.imread('src/surface/photosphere/fisheye2.jpg'), 
        img_num = 1, 
        left = 404, 
        top = -25, 
        diameter = 3040)
]

# Converts the coordinates in the projection to coordinates in the fisheye image
# all coordinates are in unit coordinates (-1 to 1 in all dimensions with 0 being center)
def projection_to_fisheye(x: float, y: float, img: int):

    # Equirectangular to latitude/longitude
    theta = math.pi * x + math.pi * img
    phi = math.pi * y / 2

    # Latitude/Longitude to 3D vector
    px = math.cos(phi) * math.sin(theta) 
    py = math.cos(phi) * math.cos(theta)
    pz = math.sin(phi)

    # 3D vector to 2D fisheye
    r = 2 * math.atan2(math.sqrt(px * px + pz * pz), py) / apeture
    theta = math.atan2(pz, px)

    x = r * math.cos(theta)
    y = r * math.sin(theta)

    fisheyeCoord = [y, x]

    # Check that it isn't out of bounds
    if x*x+y*y > 1:
        fisheyeCoord = [2, 2]

    return fisheyeCoord

# calculate the maximum width a projection can cover in unit coordinates
def max_width():
    x = 0.5
    y = 0

    result = [0, 0]

    while result[0] != 2:
        x += 0.005
        result = projection_to_fisheye(x, y, 0)

    return x - 0.005

# Calculate the unit coordinate according to the given normal coordinate and width
def normal_to_unit_grid(x: float, width):
    x = x / width
    x = x - 0.5
    x = x * 2
    return x

# Calculate the normal coordinate according to the given unit coordinate and width
def unit_to_normal_grid(x: float, width):
    x = x + 1
    x = x * width
    x = x / 2
    return math.floor(x)

# The maximum width a single projection covers in unit coordinates
max_width = max_width()

# The portions of the projections that overlap and can be blurred
left_seam = [
    unit_to_normal_grid(-1 * max_width, output_dimension[0]), 
    unit_to_normal_grid(-1 + max_width, output_dimension[0])
]
right_seam =  [
    unit_to_normal_grid(1 - max_width, output_dimension[0]), 
    unit_to_normal_grid(max_width, output_dimension[0])
]

# The output projection image
projection = np.zeros((output_dimension[1], output_dimension[0], 3), dtype = np.uint8)

# Loop through each output pixel and find its color from the fisheye
for row_index, row in enumerate(projection):
    for col_index, pixel in enumerate(row):

        # Calculate the unit coordinates of the current pixel
        unit_x = normal_to_unit_grid(col_index, projection.shape[1])
        unit_y = normal_to_unit_grid(row_index, projection.shape[0])

        # if it is in the outer image fill in the coordinate from image 2
        if col_index < left_seam[0] or col_index > right_seam[1]:
            # Calculate the unit coordinates for the fisheye
            coord = projection_to_fisheye(unit_x, unit_y, 1)

            # Calculate the normal coordinates for the fisheye
            coord[0] = unit_to_normal_grid(coord[0], fisheye_images[1].diameter) + fisheye_images[1].top
            coord[1] = unit_to_normal_grid(coord[1], fisheye_images[1].diameter) + fisheye_images[1].left

            # set the pixel
            row[col_index] = fisheye_images[1].img[coord[0]][coord[1]]

        # if it is in the inner image fill in the pixel from image 1
        elif col_index < right_seam[0] and col_index > left_seam[1]: 
            # Calculate the unit coordinates for the fisheye image
            coord = projection_to_fisheye(unit_x, unit_y, 0)

            # Calculate the normal coordinates for the fisheye image
            coord[0] = unit_to_normal_grid(coord[0], fisheye_images[0].diameter) + fisheye_images[0].top
            coord[1] = unit_to_normal_grid(coord[1], fisheye_images[0].diameter) + fisheye_images[0].left

            # Set the pixel
            row[col_index] = fisheye_images[0].img[coord[0]][coord[1]]

        # if it is in the overlapping area calculate the blur
        else:
            # Calculate the unit coordinates for both fisheye images
            coord1 = projection_to_fisheye(unit_x, unit_y, 0)
            coord2 = projection_to_fisheye(unit_x, unit_y, 1)

            # Calculate the normal coordinates for both fisheye images
            coord1[0] = unit_to_normal_grid(coord1[0], fisheye_images[0].diameter) + fisheye_images[0].top
            coord1[1] = unit_to_normal_grid(coord1[1], fisheye_images[0].diameter) + fisheye_images[0].left
            coord2[0] = unit_to_normal_grid(coord2[0], fisheye_images[1].diameter) + fisheye_images[1].top
            coord2[1] = unit_to_normal_grid(coord2[1], fisheye_images[1].diameter) + fisheye_images[1].left

            # Calculate the alpha for the blur depending on whether it is in the left or right seam
            if unit_x < 0:
                alpha = (col_index - left_seam[0]) / (left_seam[1] - left_seam[0])
            else:
                alpha = 1 - (col_index - right_seam[0]) / (right_seam[1] - right_seam[0])

            # Set the pixel using the alpha
            row[col_index] = fisheye_images[0].img[coord1[0]][coord1[1]] * alpha + fisheye_images[1].img[coord2[0]][coord2[1]] * (1 - alpha)

cv2.imwrite("src/surface/photosphere/projection.png", projection)
