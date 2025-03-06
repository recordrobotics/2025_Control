# tuning things
DISTANCE_FROM_REEF_CENTER_CORAL = 1.386  # TODO tune
DISTANCE_FROM_REEF_CENTER_ALGAE = 1.386  # TODO tune
DISTANCE_FROM_CENTER_OF_REEF_SEGMENT = 0.17  # TODO tune

# robot things
HOW_FAR_LEFT_FROM_CENTER_IS_THE_CORAL_SHOOTER = 0.203  # TODO put real number (CAD?)
HOW_FAR_LEFT_FROM_CENTER_IS_THE_ALGAE_THING = 0.125  # TODO put real number (CAD?)

# field things
FIELD_CENTER = (8.775, 4.028)  # TODO this is just a guess
BLUE_REEF_CENTER = (4.489, 4.028)  # TODO this is just a guess


import math
import json
from os import listdir
from os.path import abspath

POS = tuple[float, float]  # position
VEC = tuple[float, float]  # vector
# yes i know these are the same but they are used for different things
# so having them different reduces ambiguity in what the function's arguments are for

PLACE_TO_HEADING = {
    "A": 0,
    "AB": 0,
    "B": 0,
    "C": 60,
    "CD": 60,
    "D": 60,
    "E": 120,
    "EF": 120,
    "F": 120,
    "G": 180,
    "GH": 180,
    "H": 180,
    "I": -120,
    "IJ": -120,
    "J": -120,
    "K": -60,
    "KL": -60,
    "L": -60,
}

PLACE_MORE_LEFT = {
    "A": 1,
    "AB": 0,
    "B": -1,
    "C": 1,
    "CD": 0,
    "D": -1,
    "E": 1,
    "EF": 0,
    "F": -1,
    "G": 1,
    "GH": 0,
    "H": -1,
    "I": 1,
    "IJ": 0,
    "J": -1,
    "K": 1,
    "KL": 0,
    "L": -1,
}

CORAL_LETTERS = list("ABCDEFGHIJKL")
ALGAE_LETTERS = ["AB", "CD", "EF", "GH", "IJ", "KL"]
ALL_LETTERS = CORAL_LETTERS + ALGAE_LETTERS


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


def get_correct_end_position(letter: str, blue_side: bool) -> tuple[POS, float]:
    """returns the correct positions and headings for a given letter and side of the field"""
    left_offset = (
        HOW_FAR_LEFT_FROM_CENTER_IS_THE_CORAL_SHOOTER
        if letter in CORAL_LETTERS
        else HOW_FAR_LEFT_FROM_CENTER_IS_THE_ALGAE_THING
    )
    distance_from_reef_center = (
        DISTANCE_FROM_REEF_CENTER_CORAL
        if letter in CORAL_LETTERS
        else DISTANCE_FROM_REEF_CENTER_ALGAE
    )

    heading = PLACE_TO_HEADING[letter]

    pos = BLUE_REEF_CENTER
    pos = apply_translation_robot_relative(
        pos,
        heading,
        (
            -distance_from_reef_center,
            (DISTANCE_FROM_CENTER_OF_REEF_SEGMENT * PLACE_MORE_LEFT[letter]),
        ),
    )
    pos = apply_translation_robot_relative(pos, heading, (0, -left_offset))

    if not blue_side:
        pos, heading = switch_side_with_heading(pos, heading)

    return pos, heading


def read_path_from_letter(letter: str):
    type_of_thing = "Coral" if letter in CORAL_LETTERS else "Algae"
    with open(
        f"src/main/deploy/pathplanner/paths/Approach {type_of_thing} {letter}.path",
        "r",
    ) as f:
        return json.load(f)


def write_path_to_letter(letter: str, path):
    type_of_thing = "Coral" if letter in CORAL_LETTERS else "Algae"
    with open(
        f"src/main/deploy/pathplanner/paths/Approach {type_of_thing} {letter}.path",
        "w",
    ) as f:
        json.dump(path, f, indent=2)


def edit_linked_waypoint(name: str, pos: POS, heading: float): # TODO test heading
    # loop through all paths in ../src/main/deploy/pathplanner/paths
    # and look for waypoints with "linkedName": name
    # and change their position to pos
    if name == None:
        return

    paths = listdir("src/main/deploy/pathplanner/paths")
    for path in paths:
        with open(f"src/main/deploy/pathplanner/paths/{path}", "r") as f:
            data = json.load(f)
        for waypoint in data["waypoints"]:
            if waypoint["linkedName"] == name:
                if waypoint["nextControl"]:
                    if not waypoint["prevControl"]: # first waypoint
                        data["idealStartingState"]["rotation"] = heading

                    control_handle_offset = (
                        waypoint["nextControl"]["x"] - waypoint["anchor"]["x"],
                        waypoint["nextControl"]["y"] - waypoint["anchor"]["y"],
                    )
                    waypoint["nextControl"]["x"] = pos[0] + control_handle_offset[0]
                    waypoint["nextControl"]["y"] = pos[1] + control_handle_offset[1]

                if waypoint["prevControl"]:
                    if not waypoint["nextControl"]: # last waypoint
                        data["goalEndState"]["rotation"] = heading

                    control_handle_offset = (
                        waypoint["prevControl"]["x"] - waypoint["anchor"]["x"],
                        waypoint["prevControl"]["y"] - waypoint["anchor"]["y"],
                    )
                    waypoint["prevControl"]["x"] = pos[0] + control_handle_offset[0]
                    waypoint["prevControl"]["y"] = pos[1] + control_handle_offset[1]

                waypoint["anchor"]["x"] = pos[0]
                waypoint["anchor"]["y"] = pos[1]
        with open(f"src/main/deploy/pathplanner/paths/{path}", "w") as f:
            json.dump(data, f, indent=2)


def main():
    for letter in CORAL_LETTERS:
        # just change the linked ones
        pos, heading = get_correct_end_position(letter, True)
        edit_linked_waypoint(f"Reef{letter}", pos, heading)


if __name__ == "__main__":
    main()
