from PIL import Image, ImageFilter
from .config import SCALE, FOV, HEIGHT, ASPECT_RATIO, ID, JSON, METERS_PER_DEGREE_LAT
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
    half_width = HEIGHT * math.tan(math.radians(FOV / 2))
    width_meters = 2 * half_width
    height_meters= width_meters / ASPECT_RATIO
    return width_meters, height_meters

def get_geo_coordinates(id = 0):
    data = load_json(JSON)
    return data[id]['geo_coordinates']

def geo_coordinates_map(x = 0, y = 0):
    width, height = image_dimensions() #Dimensipns of the image in meters

    data = load_json(JSON)
    img = Image.open(f"{data[ID]['right']}")
    
    px_w, px_h = img.size
    px_w, px_h = px_w / SCALE, px_h / SCALE

    meters_per_pixel_h = width / px_w
    meters_per_pixel_v = height / px_h

    # Degrees per pixel
    lat_per_pixel = meters_per_pixel_v / METERS_PER_DEGREE_LAT
    lon_per_pixel = meters_per_pixel_h / (METERS_PER_DEGREE_LAT * math.cos(math.radians(data[ID]['geo_coordinates']['latitude'])))

    # Compute offsets from the center of the image
    center_x = px_w // 2
    center_y = px_h // 2
    x_offset = x - center_x
    y_offset = y - center_y

    # Calculate true geographical latitude and longitude
    target_lat = data[ID]['geo_coordinates']['latitude'] - (y_offset * lat_per_pixel)
    target_lon = data[ID]['geo_coordinates']['longitude'] + (x_offset * lon_per_pixel)

    return target_lat, target_lon

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


