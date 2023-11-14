import numpy as np

def trilaterate(anchor1, anchor2, anchor3):
    # Unpack anchor data
    aoa1, distance1, pdoa1, rssi1 = anchor1
    aoa2, distance2, pdoa2, rssi2 = anchor2
    aoa3, distance3, pdoa3, rssi3 = anchor3

    # Convert angles to radians
    aoa1_rad = np.radians(aoa1)
    aoa2_rad = np.radians(aoa2)
    aoa3_rad = np.radians(aoa3)

    # Coordinates of anchors (you need to replace these with actual anchor coordinates)
    x1, y1 = 0, 0
    x2, y2 = 1, 0
    x3, y3 = 0, 1

    # Trilateration equations
    A = np.array([
        [2 * (x2 - x1), 2 * (y2 - y1)],
        [2 * (x3 - x1), 2 * (y3 - y1)],
    ])

    b = np.array([
        distance1**2 - distance2**2 - x1**2 + x2**2 - y1**2 + y2**2,
        distance1**2 - distance3**2 - x1**2 + x3**2 - y1**2 + y3**2,
    ])

    # Solve the system of equations
    result = np.linalg.lstsq(A, b, rcond=None)

    if result[2] == 2:  # Check if the system is consistent
        x, y = result[0]
        return x, y
    else:
        return None

# Example usage
anchor_data = [
    (45, 1, 30, -50),  # Replace with actual data from anchor 1 (AoA, Distance, PDoA, RSSI)
    (60, 1.5, 45, -55),  # Replace with actual data from anchor 2 (AoA, Distance, PDoA, RSSI)
    (30, 1, 60, -60),  # Replace with actual data from anchor 3 (AoA, Distance, PDoA, RSSI)
]

tag_coordinates = trilaterate(*anchor_data)

if tag_coordinates is not None:
    print(f"Tag Coordinates: {tag_coordinates}")
else:
    print("System is inconsistent. Unable to determine tag coordinates.")
