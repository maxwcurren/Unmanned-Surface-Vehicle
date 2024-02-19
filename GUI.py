import tkinter as tk
from PIL import Image, ImageTk

MAX_LONGITUDE = 33.87543
MIN_LATITUDE = -117.20027
MIN_LONGITUDE = 33.83772
MAX_LATITUDE = -117.14987

IMAGE_WIDTH = 830
IMAGE_HEIGHT = 749

def convert_pixel_to_latitude(pixel_y):
    scaling_factor = IMAGE_HEIGHT / (MAX_LATITUDE - MIN_LATITUDE)
    return MAX_LATITUDE - (pixel_y / scaling_factor)

def convert_pixel_to_longitude(pixel_x):
    scaling_factor = IMAGE_WIDTH / (MAX_LONGITUDE - MIN_LONGITUDE)
    return MIN_LONGITUDE + (pixel_x / scaling_factor)

class WaypointApp:
    def __init__(self, root, image_path):
        self.root = root
        self.root.title("Waypoint Selector")

        # Load the image
        try:
            self.image = Image.open(image_path)
        except FileNotFoundError:
            raise FileNotFoundError(f"Image not found: {image_path}")

        self.photo = ImageTk.PhotoImage(self.image)

        # ... (rest of your initialization code)

    # ... (rest of your class methods)

def getWaypoints(image_path):
    root = tk.Tk()
    app = WaypointApp(root, image_path)

    # Replace root.mainloop() with the waypoint selection logic
    root.update_idletasks()
    root.deiconify()
    root.withdraw()

    root.wait_window(root)

    final_longitude, final_latitude = app.get_final_coordinates()
    return final_longitude, final_latitude

# Check if this script is the main script
if __name__ == "__main__":
    image_path = 'map.png'
    final_longitude, final_latitude = getWaypoints(image_path)
    print("Final Longitude:", final_longitude)
    print("Final Latitude:", final_latitude)
