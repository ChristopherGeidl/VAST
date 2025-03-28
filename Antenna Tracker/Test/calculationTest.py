import numpy as np

a = 6378137 # m Earth's semi-major axis
e = 0.08181919 # Earth's eccentricity

def getAngles(B_LAT, B_LONG, B_ALT, A_LAT, A_LONG, A_ALT):
    # Convert degrees to radians
    B_LAT = np.radians(B_LAT)
    B_LONG = np.radians(B_LONG)
    A_LAT = np.radians(A_LAT)
    A_LONG = np.radians(A_LONG)

    # Using Earth-Centered, Earth-Fixed (ECEF) coordinate system
    Nb = a / np.sqrt(1 - e*e*np.sin(B_LAT)**2)
    Xb = (Nb + B_ALT) * np.cos(B_LAT) * np.cos(B_LONG)
    Yb = (Nb + B_ALT) * np.cos(B_LAT) * np.sin(B_LONG)
    Zb = (Nb * (1 - e*e) + B_ALT) * np.sin(B_LAT)
    rb = np.array([Xb, Yb, Zb])

    Na = a / np.sqrt(1 - e*e*np.sin(A_LAT)**2)
    Xa = (Na + A_ALT) * np.cos(A_LAT) * np.cos(A_LONG)
    Ya = (Na + A_ALT) * np.cos(A_LAT) * np.sin(A_LONG)
    Za = (Na * (1 - e*e) + A_ALT) * np.sin(A_LAT)
    ra = np.array([Xa, Ya, Za])

    # Compute relative position in ECEF frame (antenna to balloon)
    r = rb - ra

    # Transform to ENU (East, North, Up) frame
    lat = A_LAT
    lon = A_LONG
    transform_matrix = np.array([[-np.sin(lon), np.cos(lon), 0],
                                  [-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)],
                                  [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]])
    enu = transform_matrix @ r

    # Calculate elevation and azimuth
    x, y, z = enu
    distance = np.sqrt(x**2 + y**2 + z**2)

    # Line-of-sight geometric check for curvature correction
    earth_radius = a
    geometric_horizon_angle = np.degrees(np.arctan2(z, np.sqrt(x**2 + y**2)))
    horizon_angle_correction = np.degrees(np.arccos(earth_radius / (earth_radius + A_ALT)))
    elevation = geometric_horizon_angle - horizon_angle_correction

    azimuth = np.degrees(np.arctan2(x, y))

    # Ensure azimuth is within 0-360 degrees
    azimuth = (azimuth + 360) % 360

    # Round to two decimals
    elevation = np.round(elevation, 2)
    azimuth = np.round(azimuth, 2)

    return azimuth, elevation

# Run test cases
def test(test_title, blat, blong, balt, alat, along, aalt, expected_elevation, expected_azimuth):
    print(test_title)
    az, el = getAngles(blat, blong, balt, alat, along, aalt)

    if np.isclose(el, expected_elevation, atol=0.5) and np.isclose(az, expected_azimuth, atol=0.5):
        print("PASS")
    else:
        print(f"Expected (el, az): {expected_elevation}, {expected_azimuth}")
        print(f"Actual elevation: {el}")
        print(f"Actual azimuth: {az}")
    print("")

test("Same Location", 90, 0, 0, 90, 0, 0, 0, 0)
test("Balloon Directly Above", 90, 0, 1000, 90, 0, 0, 90, 0)
test("Balloon Directly Below (other side of earth)", -90, 0, 1000, 90, 0, 0, -90, 0)
test("Balloon at Equator Prime Meridian, Antenna at North", 0, 0, 0, 90, 0, 0, -45, 0)
test("Balloon at Equator 90E, Antenna at North", 0, 90, 0, 90, 0, 0, -45, 90)
test("Balloon at Equator 90W, Antenna at North", 0, -90, 0, 90, 0, 0, -45, 270)
test("Balloon 100m away 1000m up", 43.610385, -116.340367, 1000, 43.611345, -116.340319, 0, 84.29, 0) 
test("Balloon 100m away 0m up", 43.610385, -116.340367, 0, 43.611345, -116.340319, 0, 0, 0) 
test("Balloon 1000m away 1000m up", 43.575709, -116.334336, 1000, 43.583792, -116.334293, 0, 45, 0) 
test("Balloon 1000m away 0m up", 43.575709, -116.334336, 0, 43.583792, -116.334293, 0, 0, 0)
test("Balloon 25000m away, 25000m up", 43.575709, -116.334336, 25000, 43.775270, -116.331695, 0, 45, 0)
test("Balloon 25000m away, 25000m up, antenna 25000m up", 43.575709, -116.334336, 25000, 43.775270, -116.331695, 25000, 0, 0)
test("Balloon 25000m away, 25000m up, antenna 12500m up", 43.575709, -116.334336, 25000, 43.775270, -116.331695, 12500, 22.5, 0)