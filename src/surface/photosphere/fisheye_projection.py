import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class FisheyeImage:
    img: np.ndarray
    img_num: int
    left: int
    top: int
    diameter: int


# The dimensions of the output image in width by height
OUTPUT_DIMENSION = (2000, 1000)

# The APERTURE of the fisheyes in radians
APERTURE = 195 * math.pi / 180

# The fisheye images with their information
FISHEYE_IMAGES = [
    FisheyeImage(
        img=cv2.imread('src/surface/photosphere/fisheye1.jpg'),
        img_num=0,
        left=400,
        top=28,
        diameter=3052,
    ),
    FisheyeImage(
        img=cv2.imread('src/surface/photosphere/fisheye2.jpg'),
        img_num=1,
        left=404,
        top=-25,
        diameter=3040,
    ),
]


# Converts the coordinates in the projection to coordinates in the fisheye image
# all coordinates are in unit coordinates (-1 to 1 in all dimensions with 0 being center)
def projection_to_fisheye(projection_coord: list, img: int) -> tuple:
    # Equirectangular to latitude/longitude
    theta = math.pi * projection_coord[0] + math.pi * img
    phi = math.pi * projection_coord[1] / 2

    # Latitude/Longitude to 3D vector
    px = math.cos(phi) * math.sin(theta)
    py = math.cos(phi) * math.cos(theta)
    pz = math.sin(phi)

    # 3D vector to 2D fisheye
    r = 2 * math.atan2(math.sqrt(px * px + pz * pz), py) / APERTURE
    theta = math.atan2(pz, px)

    fisheye_col = r * math.cos(theta)
    fisheye_row = r * math.sin(theta)

    return (fisheye_row, fisheye_col)

# Calculate the unit coordinate according to the given normal coordinate and width
def normal_to_unit_grid(x: int, width: int) -> float:
    return (x / width - 0.5) * 2


# Calculate the normal coordinate according to the given unit coordinate and width
def unit_to_normal_grid(x: float, width: int) -> int:
    return math.floor((x + 1) * width / 2)

def unit_to_fisheye_coord(unit_coord:tuple, fisheye_num:int) -> tuple:
    return (
        unit_to_normal_grid(unit_coord[0], FISHEYE_IMAGES[fisheye_num].diameter) + FISHEYE_IMAGES[fisheye_num].top,
        unit_to_normal_grid(unit_coord[1], FISHEYE_IMAGES[fisheye_num].diameter) + FISHEYE_IMAGES[fisheye_num].left
    )

# The maximum width a single projection covers in unit coordinates
MAX_WIDTH = APERTURE / 2 / math.pi

# The portions of the projections that overlap and can be blurred
LEFT_SEAM = [
    unit_to_normal_grid(-1 * MAX_WIDTH, OUTPUT_DIMENSION[0]),
    unit_to_normal_grid(-1 + MAX_WIDTH, OUTPUT_DIMENSION[0]),
]
RIGHT_SEAM = [
    unit_to_normal_grid(1 - MAX_WIDTH, OUTPUT_DIMENSION[0]),
    unit_to_normal_grid(MAX_WIDTH, OUTPUT_DIMENSION[0]),
]

if __name__ == '__main__':
    # The output projection image
    projection = np.zeros((OUTPUT_DIMENSION[1], OUTPUT_DIMENSION[0], 3), dtype=np.uint8)

    # Loop through each output pixel and find its color from the fisheye
    for row_index, row in enumerate(projection):
        for col_index, _pixel in enumerate(row):
            # Calculate the unit coordinates of the current pixel
            unit_x = normal_to_unit_grid(col_index, projection.shape[1])
            unit_y = normal_to_unit_grid(row_index, projection.shape[0])

            # if it is not in the overlapping section set the pixel
            if not ((LEFT_SEAM[0] <= col_index and col_index <= LEFT_SEAM[1]) or (RIGHT_SEAM[0] <= col_index and col_index <= RIGHT_SEAM[1])):
                fisheye_num = 0
                if col_index < LEFT_SEAM[0] or RIGHT_SEAM[1] < col_index:
                    fisheye_num = 1

                # Calculate the unit coordinates for the fisheye
                unit_coord = projection_to_fisheye([unit_x, unit_y], fisheye_num)

                # Calculate the normal coordinates for the fisheye
                fisheye_coord = unit_to_fisheye_coord(unit_coord, fisheye_num)

                # set the pixel
                row[col_index] = FISHEYE_IMAGES[fisheye_num].img[fisheye_coord[0]][fisheye_coord[1]]

            # if it is in the overlapping area calculate the blur
            else:
                # Calculate the unit coordinates for both fisheye images
                unit_coord1 = projection_to_fisheye([unit_x, unit_y], 0)
                unit_coord2 = projection_to_fisheye([unit_x, unit_y], 1)

                # Calculate the normal coordinates for both fisheye images
                fisheye_coord1 = unit_to_fisheye_coord(unit_coord1, 0)
                fisheye_coord2 = unit_to_fisheye_coord(unit_coord2, 1)

                # Calculate the alpha for the blur depending on whether it is in the left or right seam
                if unit_x < 0:
                    alpha = (col_index - LEFT_SEAM[0]) / (LEFT_SEAM[1] - LEFT_SEAM[0])
                else:
                    alpha = 1 - (col_index - RIGHT_SEAM[0]) / (RIGHT_SEAM[1] - RIGHT_SEAM[0])

                # Set the pixel using the alpha
                fisheye1_pixel = FISHEYE_IMAGES[0].img[fisheye_coord1[0]][fisheye_coord1[1]] * alpha
                fisheye2_pixel = FISHEYE_IMAGES[1].img[fisheye_coord2[0]][fisheye_coord2[1]] * (1 - alpha)
                row[col_index] = fisheye1_pixel + fisheye2_pixel

    cv2.imwrite('src/surface/photosphere/projection.png', projection)
