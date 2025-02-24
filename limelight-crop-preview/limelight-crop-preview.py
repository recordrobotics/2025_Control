import pygame
from networktables import NetworkTables
import sys
import ctypes


def set_window_transparency(hwnd):
    """Enable per-pixel transparency for Windows OS."""
    style = ctypes.windll.user32.GetWindowLongW(hwnd, -20)  # GWL_EXSTYLE
    ctypes.windll.user32.SetWindowLongW(hwnd, -20, style | 0x00080000)  # WS_EX_LAYERED
    ctypes.windll.user32.SetLayeredWindowAttributes(hwnd, 0, 255, 1)  # LWA_COLORKEY


left, top, right, bottom, downscale = -1, -1, 1, 1, 1

NetworkTables.initialize(server="127.0.0.1")


def valueChanged(table, key, value, isNew):
    global left, top, right, bottom, downscale
    if key == "crop":
        left = value[0]
        right = value[1]
        top = value[2]
        bottom = value[3]
    elif key == "fiducial_downscale_set":
        downscale = value


def connectionListener(connected, info):
    print(f"{info}; Connected={connected}")


NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

sd = NetworkTables.getTable("limelight")
sd.addEntryListener(valueChanged)


def transform_coordinates(value, size):
    """Convert normalized (-1 to 1) coordinates to pixel coordinates."""
    return int((value + 1) / 2 * size)


def main(aspect_ratio=(1280, 800), transparent=False):
    global left, top, right, bottom, downscale

    pygame.init()

    # Set initial window size based on aspect ratio
    base_width = 800
    base_height = int(base_width * aspect_ratio[1] / aspect_ratio[0])

    flags = pygame.RESIZABLE | pygame.DOUBLEBUF

    if transparent:
        flags |= pygame.SRCALPHA

    screen = pygame.display.set_mode((base_width, base_height), flags)

    pygame.display.set_caption("Limelight Crop Preview")

    if transparent and sys.platform == "win32":
        hwnd = pygame.display.get_wm_info()["window"]
        set_window_transparency(hwnd)

    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                # Maintain aspect ratio when resizing
                width = event.w
                height = int(width * aspect_ratio[1] / aspect_ratio[0])
                screen = pygame.display.set_mode((width, height), flags)

        screen.fill(
            (0, 0, 0, 0) if transparent else (0, 0, 0)
        )  # Transparent or black background

        # Get current window size
        width, height = screen.get_size()

        # Convert normalized coordinates to pixel coordinates
        rect_left = transform_coordinates(left, width)
        rect_top = transform_coordinates(top, height)
        rect_right = transform_coordinates(right, width)
        rect_bottom = transform_coordinates(bottom, height)

        # Draw the green rectangle
        if downscale == 1:
            pygame.draw.rect(
                screen,
                (0, 255, 0),
                (rect_left, rect_top, rect_right - rect_left, rect_bottom - rect_top),
                2,
            )
        else:
            pygame.draw.rect(
                screen,
                (255, 0, 255),
                (rect_left, rect_top, rect_right - rect_left, rect_bottom - rect_top),
                2,
            )

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main(transparent=True)
