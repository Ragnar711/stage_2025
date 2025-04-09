import math

class Position3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = round(x, 3)
        self.y = round(y, 3)
        self.z = round(z, 3)

    @staticmethod
    def fromGPScoordinates(latitude, longitude, altitude):
        # Constants for WGS84
        a = 6378137.0  
        f = 1 / 298.257223563  
        e2 = 2 * f - f ** 2  

        # Convert latitude and longitude from degrees to radians
        lat_rad = math.radians(latitude)
        lon_rad = math.radians(longitude)

        # Calculate prime vertical radius of curvature
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)

        # Calculate ECEF coordinates
        x = (N + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e2) + altitude) * math.sin(lat_rad)

        return Position3D(x, y, z)

    def toString(self):
        return f"Position3D : Latitude = {self.x}, Longitude = {self.y}, Altitude = {self.z}"        