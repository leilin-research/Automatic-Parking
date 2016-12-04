# -----------------------------------
# utility functions to check if two rotated rectangles intersect
# Author: Tao Chen
# Date: 2016.10.28
# -----------------------------------
import numpy as np



def get_line_coeffi(point1, point2):
    if point1[0] == point2[0]:
        A = 1
        B = 0
        C = -point1[0]
    else:
        x = np.array([point1[0], point2[0]])
        y = np.array([point1[1], point2[1]])
        a = np.vstack([x, np.ones(len(x))]).T
        m, c =np.linalg.lstsq(a, y)[0] # y = mx + c
        A = m
        B = -1
        C = c
    return A, B, C


def two_rects_intersect(rect1_verts, rect2_verts):
    tolerance = 1e-8
    for line1_idx in range(4):
        for line2_idx in range(4):
            line1 = rect1_verts[line1_idx, :]
            line1 = np.vstack((line1, rect1_verts[(line1_idx + 1) % 4, :]))
            line2 = rect2_verts[line2_idx, :]
            line2 = np.vstack((line2, rect2_verts[(line2_idx + 1) % 4, :]))
            a1, b1, c1 = get_line_coeffi(line1[0, :], line1[1, :])
            a2, b2, c2 = get_line_coeffi(line2[0, :], line2[1, :])
            if abs(a1 * b2 - a2 * b1) < tolerance:
                continue
            else:
                A = np.array([[a1, b1], [a2, b2]])
                C = -np.array([[c1], [c2]])
                x, y = np.linalg.lstsq(A, C)[0]
                # print (x - line1[0, 0]) * (x - line1[1, 0])
                # print (y - line1[0, 1]) * (y - line1[1, 1])
                # print (x - line2[0, 0]) * (x - line2[1, 0])
                # print (y - line2[0, 1]) * (y - line2[1, 1])
                if (x - line1[0, 0]) * (x - line1[1, 0]) <= tolerance and \
                   (y - line1[0, 1]) * (y - line1[1, 1]) <= tolerance and \
                   (x - line2[0, 0]) * (x - line2[1, 0]) <= tolerance and \
                   (y - line2[0, 1]) * (y - line2[1, 1]) <= tolerance:
                    return True
                else:
                    continue
    return False