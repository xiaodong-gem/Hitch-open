import csv
import math

# ✅ TTL 轨迹原点
ORIGIN = {
    'lat': 39.79476,
    'lon': -86.234915,
    'alt': 0.87
}

def gps_to_enu(lat, lon, alt):
    lat0, lon0, alt0 = ORIGIN['lat'], ORIGIN['lon'], ORIGIN['alt']
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f ** 2

    def geodetic_to_ecef(lat, lon, alt):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)

        N = a / math.sqrt(1 - e2 * sin_lat ** 2)
        x = (N + alt) * cos_lat * cos_lon
        y = (N + alt) * cos_lat * sin_lon
        z = (N * (1 - e2) + alt) * sin_lat
        return x, y, z

    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, alt0)
    x, y, z = geodetic_to_ecef(lat, lon, alt)

    dx = x - x0
    dy = y - y0
    dz = z - z0

    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    enu_e = -math.sin(lon0_rad) * dx + math.cos(lon0_rad) * dy
    enu_n = (-math.sin(lat0_rad) * math.cos(lon0_rad) * dx
             - math.sin(lat0_rad) * math.sin(lon0_rad) * dy
             + math.cos(lat0_rad) * dz)
    enu_u = (math.cos(lat0_rad) * math.cos(lon0_rad) * dx
             + math.cos(lat0_rad) * math.sin(lon0_rad) * dy
             + math.sin(lat0_rad) * dz)

    return enu_e, enu_n, enu_u

def convert_csv(input_file, output_file):
    with open(input_file, newline='') as infile, open(output_file, 'w', newline='') as outfile:
        reader = csv.DictReader(infile)
        writer = csv.writer(outfile)
        writer.writerow(['x', 'y', 'z', 'target_yaw', 'target_speed'])  # 表头

        for row in reader:
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            alt = float(row['altitude'])
            x, y, z = gps_to_enu(lat, lon, alt)
            writer.writerow([f"{x:.15f}", f"{y:.15f}", f"{z:.15f}", "0.0", "0.0"])

if __name__ == "__main__":
    convert_csv("your_gps_data.csv", "output_ttl_format.csv")