FIELD_CENTER = (8.775, 4.028)  # TODO this is just a guess
BLUE_REEF_CENTER = (4.489, 4.028)  # TODO this is just a guess

import math
import json

POS = tuple[float, float]  # position
VEC = tuple[float, float]  # vector
# yes i know these are the samebut they are used for different things
# so having them different reduces ambiguity in what the function's arguments are for

PLACE_TO_HEADING = {
    "A": 0,
    "B": 0,
    "C": 60,
    "D": 60,
    "E": 120,
    "F": 120,
    "G": 180,
    "H": 180,
    "I": -120,
    "J": -120,
    "K": -60,
    "L": -60,
}


def switch_side(pos: POS) -> POS:
    """rotate pos 180 degrees about the field center"""
    px, py = pos
    px, py = (
        px - FIELD_CENTER[0],
        py - FIELD_CENTER[1],
    )  # shift it so field center is 0,0
    px, py = -px, -py  # invert about the origin (equivelant to rotate 180 degrees)
    px, py = px + FIELD_CENTER[0], py + FIELD_CENTER[1]  # shift it back
    return px, py


def get_rotated_unit_vectors(heading: float) -> tuple[VEC, VEC]:
    """returns the unit vectors for a given heading in degrees"""
    rad = math.radians(heading)
    return (round(math.cos(rad)), math.sin(rad)), (-math.sin(rad), math.cos(rad))


def apply_translation_robot_relative(pos: POS, heading: float, trans: VEC) -> POS:
    """apply a translation to a position relative to that position and its heading"""
    unit_vectors = get_rotated_unit_vectors(heading)
    return (
        pos[0] + trans[0] * unit_vectors[0][0] + trans[1] * unit_vectors[1][0],
        pos[1] + trans[0] * unit_vectors[0][1] + trans[1] * unit_vectors[1][1],
    )


print(apply_translation_robot_relative(BLUE_REEF_CENTER, 0, (1, 1))) # TODO need mor tst
