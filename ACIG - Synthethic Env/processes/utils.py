from PIL import Image, ImageFilter
from .config import SCALE, FOV, HEIGHT, ASPECT_RATIO, ID, JSON, METERS_PER_DEGREE_LAT, H_FOV, V_FOV
import numpy as np
import os, json, math

def get_image_arr(cam = "left", greyscale=False):
    data = load_json(JSON)

    img = Image.open(f"{data[ID][cam]}")
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
    H = HEIGHT  
    # половин ширина и половин височина (катет в правоъгълен триъгълник)
    half_w = H * math.tan(math.radians(H_FOV  / 2))
    half_h = H * math.tan(math.radians(V_FOV  / 2))
    return 2 * half_w, 2 * half_h


def get_geo_coordinates(id = 0):
    data = load_json(JSON)
    return data[id]['geo_coordinates']

def geo_coordinates_map(x, y, idx=ID):
    """
    Преобразува пикселен (x,y) → (latitude, longitude),
    без изкривяване на съотношението 4:3.
    """
    data = load_json(JSON)[idx]['geo_coordinates']
    img = Image.open(load_json(JSON)[idx]['right'])
    px_w, px_h = (s/SCALE for s in img.size)

    width_m, height_m = image_dimensions()
    mpp_x = width_m  / px_w
    mpp_y = height_m / px_h

    # център на изображението
    cx, cy = px_w/2, px_h/2
    dx, dy = x - cx, y - cy

    lat_per_px = mpp_y / METERS_PER_DEGREE_LAT
    lon_per_px = mpp_x / (
        METERS_PER_DEGREE_LAT * math.cos(math.radians(data['latitude']))
    )

    lat = data['latitude'] - dy * lat_per_px
    lon = data['longitude'] + dx * lon_per_px
    return lat, lon


def geo_coordinates_map_2(x, y):
    #Dimensipns of the image in meters
    width_m, height_m = image_dimensions()

    data = load_json(JSON)
    img = Image.open(f"{data[ID]['right']}")
    width_px, height_px = img.size
    #@TODO: описание защо се дели на SCALE
    width_px, height_px = width_px / SCALE, height_px / SCALE

    meters_per_pixel_h = width_m / width_px
    meters_per_pixel_v = height_m / height_px
    # Calculate the displacement from the image center in pixels
    displacement_x_px = x - (width_px / 2)
    displacement_y_px = y - (height_px / 2)

    # Convert pixel displacement to meters
    displacement_x_m = (displacement_x_px / width_px) * width_m
    displacement_y_m = (displacement_y_px / height_px) * height_m

    earth_radius = 6378137.0  # Earth's radius in meters

    # Calculate the change in latitude and longitude
    delta_lat = (displacement_y_m / earth_radius) * (180 / np.pi)
    delta_lon = (displacement_x_m / (earth_radius * np.cos(np.pi * data[ID]['geo_coordinates']['latitude'] / 180))) * (180 / np.pi)

    # Calculate the estimated geographic coordinates of the bounding box center
    lat = data[ID]['geo_coordinates']['latitude'] + delta_lat
    lon = data[ID]['geo_coordinates']['longitude'] + delta_lon

    return lat, lon


