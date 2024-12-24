from PIL import Image, ImageFilter
from .config import SCALE, FOV, HEIGHT, ASPECT_RATIO, EARTH_CIRCUMFERENCE_METERS
import numpy as np
import os, json, math

def get_image_arr(name = "data.json", id = 0, cam = "left", greyscale=False):
    data = load_json(name)

    img = Image.open(f"{data[id][cam]}")
    width, height = img.size
    img = img.resize((int(width/SCALE), int(height/SCALE)))
    if greyscale:
        img = img.convert(mode="L")
    else:
        if img.mode != "RGB":
            img = img.convert(mode="RGB")
    return np.asarray(img)

def load_json(json_file_path):

    if not os.path.exists(json_file_path):
        raise FileNotFoundError(f"JSON file not found: {json_file_path}")

    with open(json_file_path, 'r') as file:
        data = json.load(file)

    result = []

    for entry in data:
        left = entry.get('left')
        right = entry.get('right')
        geo_coords = entry.get('geo_coordinates')
        result.append({"left": left, "right": right, "geo_coordinates": geo_coords})

    return result

def image_dimensions():
    half_width = HEIGHT * math.tan(math.radians(FOV / 2))
    width_meters = 2 * half_width
    height_meters= width_meters / ASPECT_RATIO
    return width_meters, height_meters

def get_geo_coordinates(name = "data.json", id = 0):
    data = load_json(name)
    return data[id]['geo_coordinates']

def geo_coordinates_map(name = "data.json", id = 0, cam = "left", x = 0, y = 0):
    width, height = image_dimensions()

    data = load_json(name)
    img = Image.open(f"{data[id][cam]}")
    px_w, px_h = img.size

    meters_per_pixel_h = width / px_w
    meters_per_pixel_v = height / px_h

    # Degrees per pixel
    lat_per_pixel = meters_per_pixel_v / EARTH_CIRCUMFERENCE_METERS
    lon_per_pixel = meters_per_pixel_h / (EARTH_CIRCUMFERENCE_METERS * math.cos(math.radians(data[id]['geo_coordinates']['latitude'])))

    # Compute offsets from the center of the image
    center_x = px_w // 2
    center_y = px_h // 2

    x_offset = x - center_x
    y_offset = y - center_y

    # Calculate new latitude and longitude
    target_lat = data[id]['geo_coordinates']['latitude'] - (y_offset * lat_per_pixel)
    target_lon = data[id]['geo_coordinates']['longitude'] + (x_offset * lon_per_pixel)

    return target_lat, target_lon