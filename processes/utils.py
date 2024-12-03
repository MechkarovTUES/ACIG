from PIL import Image, ImageFilter
from .config import SCALE, SET
import numpy as np

def get_image_arr(name = "im0", greyscale=False):
    img = Image.open(f"stereoData/{SET}/{name}.png")
    width, height = img.size
    img = img.resize((int(width/SCALE), int(height/SCALE)))
    if greyscale:
        img = img.convert(mode="L")
    else:
        if img.mode != "RGB":
            img = img.convert(mode="RGB")
    return np.asarray(img)
