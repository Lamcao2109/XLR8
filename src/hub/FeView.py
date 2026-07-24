from pybricks.tools import wait
from pybricks.parameters import Button

# Reuse the already-initialised hub and sensor objects from FeFunctions.
# Importing this module runs FeFunctions' setup (hub, motors, sensors).
import FeFunctions as fe


def read_ultra(dev):
    """Return distance from a PUPDevice ultrasonic sensor, -1 means no echo."""
    try:
        val = dev.read(0)
        return val[0]
    except Exception:
        return None


def main():
    fe.hub.display.off()
    print("Sensor monitor running. Press CENTER button to stop.")
    print("cam(x, y, colour, black) | RGBW | L | R")

    while True:
        # Stop when the center button is pressed.
        if Button.CENTER in fe.hub.buttons.pressed():
            break

        # --- Camera ---
        x, y, colour, black = fe.ReadCam()
        if colour == fe.RED:
            colour_str = "RED"
        elif colour == fe.GREEN:
            colour_str = "GREEN"
        else:
            colour_str = "NONE"

        # --- Color sensor: raw RGBW, before any conversion ---
        try:
            rgbw = fe.ColorSens.read(5)
            r, g, b, w = rgbw[0], rgbw[1], rgbw[2], rgbw[3]
        except Exception:
            r = g = b = w = None

        # --- Ultrasonic sensors ---
        left = read_ultra(fe.UltraL)
        right = read_ultra(fe.UltraR)

        print(
            "CAM x={:>4} y={:>4} col={:<5} blk={:>4} | "
            "RGBW=({:>4},{:>4},{:>4},{:>4}) | L={:>5} | R={:>5}".format(
                x, y, colour_str, black, r, g, b, w, left, right
            )
        )

        wait(100)  # ~10 readings per second

    print("Stopped.")


main()