# To make a new matrix: adjust the meta data to the new values and run the equirectangular_projection function
# To make an image using a matrix: use the convert_with_matrix function and give the fisheye images in the order of the metadata
# To make an image without a matrix: adjust the metadata, then use the equirectangular_projection_original function
# To adjust meta data: find the left side of the circle, diameter of the circle (right - left) then find the top
# To find the top of the circle it may be necessary to find the bottom then subtract the diameter if the top is cut off
# If using this file without ros, the imports will need to be changed to be from projection_matrix import get_matrix

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
from numpy import generic
from numpy.typing import NDArray

from photosphere.projection_matrix import get_matrix

# from projection_matrix import get_matrix

Matlike = NDArray[generic]


@dataclass
class FisheyeMetaData:
    img_num: int
    left: int
    top: int
    diameter: int


# The dimensions of the output image in width by height
# needs to be 2:1 width to height
OUTPUT_DIMENSION = (1000, 500)

# The APERTURE of the fisheyes in radians
APERTURE = 185 * math.pi / 180

# The maximum width a single projection covers in unit coordinates
MAX_WIDTH = APERTURE / 2 / math.pi

# The meta data for the fisheye images
# Meta data for the fisheye images that are resized to 960 x 758
FISHEYE_META_DATA = (
    FisheyeMetaData(
        img_num=0,
        left=105,
        top=13,
        diameter=759,
    ),
    FisheyeMetaData(
        img_num=1,
        left=99,
        top=-4,
        diameter=762,
    ),
)

# The meta data for the full resolution images
# FISHEYE_META_DATA = (
#     FisheyeMetaData(
#         img_num=0,
#         left=448,
#         top=55,
#         diameter=3010,
#     ),
#     FisheyeMetaData(
#         img_num=1,
#         left=390,
#         top=-20,
#         diameter=3060,
#     ),
# )


def projection_to_fisheye(projection_coord: tuple[float, float], img: int) -> tuple[float, float]:
    """
    Convert the coordinates in the projection to coordinates in the fisheye image.
    All coordinates are in unit coordinates (-1 to 1 in all dimensions with 0 as the center).

    Parameters
    ----------
    projection_coord : tuple[float, float]
        The unit coordinates in the projection to calculate the fisheye coordinates of
    img : int
        The number of the image, if in the center = 0, edges = 1

    Returns
    -------
    tuple[float, float]
        The corresponding unit fisheye coordinates for that image
    """
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


def normal_to_unit_grid(x: int, width: int) -> float:
    """
    Calculate the unit coordinate according to the given normal coordinate and width.

    Parameters
    ----------
    x : int
        the normal coordinate
    width : int
        the width

    Returns
    -------
    float
        the unit coordinate that corresponds to the normal coordinate
    """
    return (x / width - 0.5) * 2


def unit_to_normal_grid(x: float, width: int) -> int:
    """
    Calculate the normal coordinate according to the given unit coordinate and width.

    Parameters
    ----------
    x : float
        the unit coordinate
    width : int
        the width

    Returns
    -------
    int
        the normal coordinate that corresponds to the unit coordinate
    """
    return math.floor((x + 1) * width / 2)


def unit_to_fisheye_coord(
    unit_coord: tuple[float, float], fisheye_image: FisheyeMetaData
) -> tuple[int, int]:
    """
    Calculate the normal fisheye coordinate given the unit coordinate and the image.

    Parameters
    ----------
    unit_coord : tuple[int, int]
        the unit coordinate in [row, column]
    fisheye_image : FisheyeMetaData
        the meta data for the image to find the coordinates for

    Returns
    -------
    tuple[int, int]
        the coordinates of the point in the fisheye image
    """
    return (
        unit_to_normal_grid(unit_coord[0], fisheye_image.diameter) + fisheye_image.top,
        unit_to_normal_grid(unit_coord[1], fisheye_image.diameter) + fisheye_image.left,
    )


# The portions of the projections that overlap and can be blurred
# The first item is the left bound, the second is the right bound
LEFT_SEAM = (
    unit_to_normal_grid(-1 * MAX_WIDTH, OUTPUT_DIMENSION[0]),
    unit_to_normal_grid(-1 + MAX_WIDTH, OUTPUT_DIMENSION[0]),
)
RIGHT_SEAM = (
    unit_to_normal_grid(1 - MAX_WIDTH, OUTPUT_DIMENSION[0]),
    unit_to_normal_grid(MAX_WIDTH, OUTPUT_DIMENSION[0]),
)


def equirectangular_projection() -> Matlike:
    """
    Create an equirectangular projection matrix based on two fisheye images.

    Returns
    -------
    Matlike
        the projection image
    """
    # The output projection image
    projection = np.zeros((OUTPUT_DIMENSION[1], OUTPUT_DIMENSION[0], 3), dtype=np.uint8)
    projection_matrix = [
        [[] for col in range(OUTPUT_DIMENSION[0])] for row in range(OUTPUT_DIMENSION[1])
    ]
    print(len(projection_matrix))
    print(len(projection_matrix[0]))
    print('blank prjection created')

    # Loop through each output pixel and find its color from the fisheye
    for row_index, row in enumerate(projection):
        # stored_matrix.append("\t[")
        for col_index, _pixel in enumerate(row):
            # Calculate the unit coordinates of the current pixel
            projection_unit_coord = (
                normal_to_unit_grid(col_index, projection.shape[1]),
                normal_to_unit_grid(row_index, projection.shape[0]),
            )

            # if it is not in the overlapping section set the pixel
            if not (
                (LEFT_SEAM[0] <= col_index <= LEFT_SEAM[1])
                or (RIGHT_SEAM[0] <= col_index <= RIGHT_SEAM[1])
            ):
                fisheye_num = 0
                if col_index < LEFT_SEAM[0] or RIGHT_SEAM[1] < col_index:
                    fisheye_num = 1

                # Calculate the unit coordinates for the fisheye
                fisheye_unit_coord = projection_to_fisheye(
                    (projection_unit_coord[0], projection_unit_coord[1]), fisheye_num
                )

                # Calculate the normal coordinates for the fisheye
                fisheye_normal_coord = unit_to_fisheye_coord(
                    fisheye_unit_coord, FISHEYE_META_DATA[fisheye_num]
                )

                if fisheye_num == 0:
                    projection_matrix[row_index][col_index].extend(fisheye_normal_coord)
                else:
                    projection_matrix[row_index][col_index].extend(fisheye_normal_coord)

            # if it is in the overlapping area calculate the blur
            else:
                # Calculate the unit coordinates for both fisheye images
                fisheye_unit_coord1 = projection_to_fisheye(
                    (projection_unit_coord[0], projection_unit_coord[1]), 0
                )
                fisheye_unit_coord2 = projection_to_fisheye(
                    (projection_unit_coord[0], projection_unit_coord[1]), 1
                )

                # Calculate the normal coordinates for both fisheye images
                fisheye_normal_coord1 = unit_to_fisheye_coord(
                    fisheye_unit_coord1, FISHEYE_META_DATA[0]
                )
                fisheye_normal_coord2 = unit_to_fisheye_coord(
                    fisheye_unit_coord2, FISHEYE_META_DATA[1]
                )

                # Calculate the alpha for the blur depending on which seam it is in
                if projection_unit_coord[0] < 0:
                    alpha = (col_index - LEFT_SEAM[0]) / (LEFT_SEAM[1] - LEFT_SEAM[0])
                else:
                    alpha = 1 - (col_index - RIGHT_SEAM[0]) / (RIGHT_SEAM[1] - RIGHT_SEAM[0])

                projection_pixel = [
                    fisheye_normal_coord1[0],
                    fisheye_normal_coord1[1],
                    fisheye_normal_coord2[0],
                    fisheye_normal_coord2[1],
                    int(alpha * 100),
                ]
                projection_matrix[row_index][col_index] = projection_pixel

    print('projection created, returning')
    store_projection_matrix(projection_matrix)
    return projection


def equirectangular_projection_original(
    fisheye_image1: Matlike, fisheye_image2: Matlike
) -> Matlike:
    """
    Create an equirectangular projection image based on two fisheye images.

    Parameters
    ----------
    fisheye_image1 : Matlike
        the fisheye image for the center of the projection
    fisheye_image2 : Matlike
        the fisheye image for the edges of the projection

    Returns
    -------
    Matlike
        the projection image
    """
    # Create the input fisheye images
    images = (fisheye_image1, fisheye_image2)

    # The output projection image
    projection = np.zeros((OUTPUT_DIMENSION[1], OUTPUT_DIMENSION[0], 3), dtype=np.uint8)

    print('blank prjection created')

    # Loop through each output pixel and find its color from the fisheye
    for row_index, row in enumerate(projection):
        for col_index, _pixel in enumerate(row):
            # Calculate the unit coordinates of the current pixel
            projection_unit_coord = (
                normal_to_unit_grid(col_index, projection.shape[1]),
                normal_to_unit_grid(row_index, projection.shape[0]),
            )

            # if it is not in the overlapping section set the pixel
            if not (
                (LEFT_SEAM[0] <= col_index <= LEFT_SEAM[1])
                or (RIGHT_SEAM[0] <= col_index <= RIGHT_SEAM[1])
            ):
                fisheye_num = 0
                if col_index < LEFT_SEAM[0] or RIGHT_SEAM[1] < col_index:
                    fisheye_num = 1

                # Calculate the unit coordinates for the fisheye
                fisheye_unit_coord = projection_to_fisheye(
                    (projection_unit_coord[0], projection_unit_coord[1]), fisheye_num
                )

                # Calculate the normal coordinates for the fisheye
                fisheye_normal_coord = unit_to_fisheye_coord(
                    fisheye_unit_coord, FISHEYE_META_DATA[fisheye_num]
                )

                # set the pixel
                row[col_index] = images[fisheye_num][fisheye_normal_coord[0]][
                    fisheye_normal_coord[1]
                ]

            # if it is in the overlapping area calculate the blur
            else:
                # Calculate the unit coordinates for both fisheye images
                fisheye_unit_coord1 = projection_to_fisheye(
                    (projection_unit_coord[0], projection_unit_coord[1]), 0
                )
                fisheye_unit_coord2 = projection_to_fisheye(
                    (projection_unit_coord[0], projection_unit_coord[1]), 1
                )

                # Calculate the normal coordinates for both fisheye images
                fisheye_normal_coord1 = unit_to_fisheye_coord(
                    fisheye_unit_coord1, FISHEYE_META_DATA[0]
                )
                fisheye_normal_coord2 = unit_to_fisheye_coord(
                    fisheye_unit_coord2, FISHEYE_META_DATA[1]
                )

                # Calculate the alpha for the blur depending on which seam it is in
                if projection_unit_coord[0] < 0:
                    alpha = (col_index - LEFT_SEAM[0]) / (LEFT_SEAM[1] - LEFT_SEAM[0])
                else:
                    alpha = 1 - (col_index - RIGHT_SEAM[0]) / (RIGHT_SEAM[1] - RIGHT_SEAM[0])

                # Set the pixel using the alpha
                fisheye1_pixel = (
                    images[0][fisheye_normal_coord1[0]][fisheye_normal_coord1[1]] * alpha
                )
                fisheye2_pixel = images[1][fisheye_normal_coord2[0]][fisheye_normal_coord2[1]] * (
                    1 - alpha
                )
                row[col_index] = fisheye1_pixel + fisheye2_pixel

    print('projection created, returning')
    return projection


def store_projection_matrix(projection_matrix: Matlike) -> None:
    """
    Store the given projection matrix in a file.

    Parameters
    ----------
    projection_matrix : Matlike
        the projection matrix to store
    """
    matrix_strings = [
        """def get_matrix() -> list[list[list[int]]]:
    matrix = ["""
    ]
    for row in projection_matrix:
        matrix_strings.append('[')
        for col in row:
            matrix_strings.append('[' + ','.join(str(num) for num in col) + '],')
        matrix_strings.append('],')
    matrix_strings.append(']\n    return matrix')

    with open('src/surface/photosphere/photosphere/projection_matrix.py', 'w') as file:
        file.writelines(matrix_strings)


def convert_with_matrix(fisheye_image1: Matlike, fisheye_image2: Matlike) -> Matlike:
    projection_matrix = get_matrix()
    projection_image = np.zeros((OUTPUT_DIMENSION[1], OUTPUT_DIMENSION[0], 3), dtype=np.uint8)
    images = (fisheye_image1, fisheye_image2)

    for row_index, row in enumerate(projection_matrix):
        for col_index, pixel in enumerate(row):
            if len(pixel) == 5:
                fisheye1_pixel = images[0][pixel[0]][pixel[1]] * (pixel[4] / 100.0)
                fisheye2_pixel = images[1][pixel[2]][pixel[3]] * (1 - (pixel[4] / 100.0))
                projection_image[row_index][col_index] = fisheye1_pixel + fisheye2_pixel

            elif col_index < LEFT_SEAM[0] or RIGHT_SEAM[1] < col_index:
                projection_image[row_index][col_index] = images[1][pixel[0]][pixel[1]]
            else:
                projection_image[row_index][col_index] = images[0][pixel[0]][pixel[1]]

    return projection_image


if __name__ == '__main__':
    fisheye_image1 = cv2.imread('src/surface/photosphere/photosphere/fisheye1.jpg')
    fisheye_image2 = cv2.imread('src/surface/photosphere/photosphere/fisheye2.jpg')

    start_time = time.time()
    projection = equirectangular_projection()

    finish_projection_time = time.time()

    # projection = convert_with_matrix(fisheye_image1, fisheye_image2)
    # cv2.imwrite('src/photosphere/display/projection.jpg', projection)

    finish_map = time.time()
    # projection = equirectangular_projection_original(fisheye_image1, fisheye_image2)
    # cv2.imwrite('src/photosphere/display/projection.jpg', projection)

    finish_original = time.time()

    print('Time started: ', start_time)
    print('Time matrix finished calculating:', finish_projection_time)
    print('Time finished mapping to image:', finish_map)
    print('Time finished original:', finish_original)
    print('Time to make matrix:', finish_projection_time - start_time)
    print('Time to finish conversion:', finish_map - finish_projection_time)
    print('Time to do original:', finish_original - finish_map)
