# tuning things
DISTANCE_FROM_REEF_CENTER = 1.386  # TODO tune
DISTANCE_FROM_CENTER_OF_REEF_SEGMENT = 0.17  # TODO tune

# robot things
HOW_FAR_LEFT_FROM_CENTER_IS_THE_CORAL_SHOOTER = 0.203  # TODO put real number

# field things
FIELD_CENTER = (8.775, 4.028)  # TODO this is just a guess
BLUE_REEF_CENTER = (4.489, 4.028)  # TODO this is just a guess


import math

POS = tuple[float, float]  # position
VEC = tuple[float, float]  # vector
# yes i know these are the same but they are used for different things
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

PLACE_MORE_LEFT = {
    "A": 1,
    "B": -1,
    "C": 1,
    "D": -1,
    "E": 1,
    "F": -1,
    "G": 1,
    "H": -1,
    "I": 1,
    "J": -1,
    "K": 1,
    "L": -1,
}

LETTERS = list("ABCDEFGHIJKL")


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


def normalize_angle(angle: float) -> float:
    """make an angle (-180, 180]"""
    result = (angle + 180) % 360 - 180
    if result == -180:
        result = 180.0
    return result


def switch_side_with_heading(pos: POS, heading: float) -> tuple[POS, float]:
    """rotate pos 180 degrees about the field center"""
    return switch_side(pos), normalize_angle(heading + 180)


def get_rotated_unit_vectors(heading: float) -> tuple[VEC, VEC]:
    """returns the unit vectors for a given heading in degrees"""
    rad = math.radians(heading)
    return (math.cos(rad), math.sin(rad)), (-math.sin(rad), math.cos(rad))


def apply_translation_robot_relative(pos: POS, heading: float, trans: VEC) -> POS:
    """apply a translation to a position relative to that position and its heading"""
    unit_vectors = get_rotated_unit_vectors(heading)
    return (
        pos[0] + trans[0] * unit_vectors[0][0] + trans[1] * unit_vectors[1][0],
        pos[1] + trans[0] * unit_vectors[0][1] + trans[1] * unit_vectors[1][1],
    )


def get_correct_position(letter: str, blue_side: bool) -> tuple[POS, float]:
    """returns the correct positions and headings for a given letter and side of the field"""
    heading = PLACE_TO_HEADING[letter]

    pos = BLUE_REEF_CENTER
    pos = apply_translation_robot_relative(
        pos,
        heading,
        (
            -DISTANCE_FROM_REEF_CENTER,
            DISTANCE_FROM_CENTER_OF_REEF_SEGMENT * PLACE_MORE_LEFT[letter],
        ),
    )
    pos = apply_translation_robot_relative(
        pos,
        heading,
        (
            0,
            -HOW_FAR_LEFT_FROM_CENTER_IS_THE_CORAL_SHOOTER,
        ),
    )

    if not blue_side:
        pos, heading = switch_side_with_heading(pos, heading)

    return pos, heading

print(get_correct_position("D", True))