import math

def calculate_distance(x1, y1, x2, y2):
    """
    Calculate the Euclidean distance between two points (x1, y1) and (x2, y2).
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def normal_error(x1, y1, x2, y2, x3, y3):
    """
    Calculate the normal error, i.e., the perpendicular distance from point (x3, y3) 
    to the line defined by points (x1, y1) and (x2, y2).
    """
    # Line equation: Ax + By + C = 0
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2
    
    # Perpendicular distance from (x3, y3) to the line Ax + By + C = 0
    distance = abs(A * x3 + B * y3 + C) / math.sqrt(A**2 + B**2)
    
    return distance

def main():    
    # Start (x1, y1), Current (x2, y2), Target (x3, y3)

    # Calculate the normal error (perpendicular distance to the line from start to target)
    perpendicular_distance = normal_error(x1, y1, x3, y3, x2, y2)

    # Calculate the distance error (Euclidean distance to the target)
    distance_to_target = calculate_distance(x2, y2, x3, y3)

    inline_distance = math.sqrt((distance_to_target)**2 - (perpendicular_distance)**2)
    

if __name__ == "__main__":
    main()
