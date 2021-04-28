import shapely.geometry as sg
import math

robot_diameter = 7.4


def convert(orig_set):
    new_set = []
    for i in range(len(orig_set[0].geoms)):
        # Environment boundary obstacle
        line1 = list(orig_set[0].geoms)[i]
        if i == len(orig_set[0].geoms) - 1:
            # Is last segment
            line2 = list(orig_set[0].geoms)[0]
        else:
            line2 = list(orig_set[0].geoms)[i + 1]
    for ob in orig_set[1:]:
        # Inner obstacles
        for seg in ob:
            pass
    return orig_set
