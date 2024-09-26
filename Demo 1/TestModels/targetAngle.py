import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Function to calculate the angle to the target point
def calculate_angle(current, target):
    delta_x = target.x - current.x
    delta_y = target.y - current.y

    # atan2 returns the angle in radians
    angle_radians = math.atan2(delta_y, delta_x)

    # Convert to degrees
    angle_degrees = math.degrees(angle_radians)

    # Normalize the angle to be within [0, 360)
    if angle_degrees < 0:
        angle_degrees += 360.0

    return angle_degrees

# Example usage
if __name__ == "__main__":
    current = Point(1.0, 1.0)
    target = Point(1.0, -1.0)

    angle = calculate_angle(current, target)

    print(f"Angle to target: {angle:.2f} degrees")
