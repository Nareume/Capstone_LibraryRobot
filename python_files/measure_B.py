from PIL import Image
from PIL import ImageEnhance

def measure_brightness(image_path):
    
    image = Image.open(image_path)
    image = image.convert("RGB")
    pixels = image.getdata()
    
    brightness = sum(sum(pixel) for pixel in pixels) / (len(pixels) * 3)

    return brightness

def adjust_brightness(image_path, brightness_factor):

    image = Image.open(image_path)
    enhancer = ImageEnhance.Brightness(image)
    adjusted_image = enhancer.enhance(brightness_factor)
    
    return adjusted_image

image_path = "test (80).jpg"
brightness = measure_brightness(image_path)
print("밝기:", brightness)
brightness_factor = 1.5
adjusted_image = adjust_brightness(image_path, brightness_factor)
adjusted_image.show()