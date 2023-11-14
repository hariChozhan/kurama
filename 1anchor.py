import numpy as np

def estimate_position(anchor):
    # Unpack anchor data
    aoa, distance, pdoa, rssi = anchor

    # Convert angle to radians
    aoa_rad = np.radians(aoa)

    # Coordinates of the anchor (replace with actual anchor coordinates)
    anchor_x, anchor_y = 0, 0

    # Estimate x and y coordinates based on distance and AoA
    x = anchor_x + distance * np.cos(aoa_rad)
    y = anchor_y + distance * np.sin(aoa_rad)

    return x, y

# Example usage
anchor_data = (45, 1, 30, -50)  # Replace with actual data from your anchor (AoA, Distance, PDoA, RSSI)

tag_coordinates = estimate_position(anchor_data)

print(f"Tag Coordinates: {tag_coordinates}")
