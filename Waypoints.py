import tkinter as tk
from PIL import Image, ImageTk

max_longitude = 33.87543
min_latitude = -117.20027
min_longitude = 33.83772
max_latitude = -117.14987

image_width = 830
image_height = 749

def convert_pixel_to_latitude(pixel_y, min_latitude, max_latitude, image_height):
    scaling_factor = image_height / (max_latitude - min_latitude)
    return max_latitude - (pixel_y / scaling_factor)

def convert_pixel_to_longitude(pixel_x, min_longitude, max_longitude, image_width):
    scaling_factor = image_width / (max_longitude - min_longitude)
    return min_longitude + (pixel_x / scaling_factor)

class WaypointApp:
    def __init__(self, root, image_path):
        self.root = root
        self.root.title("Waypoint Selector")

        # Load the image
        self.image = Image.open(image_path)
        self.photo = ImageTk.PhotoImage(self.image)

        # Create Canvas for image
        self.canvas = tk.Canvas(root, width=self.image.width, height=self.image.height)
        self.canvas.pack(side=tk.RIGHT)  # Adjusted to place canvas on the right

        # Display the image on the canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.photo)

        # Bind mouse click event to handle waypoint selection
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        # Waypoint arrays
        self.x_array = []
        self.y_array = []
        self.latitude_array = []
        self.longitude_array = []

        # Create "Finished" and "Undo" buttons
        self.finish_button = tk.Button(root, text="Finished", command=self.on_finish_click)
        self.finish_button.pack()

        self.undo_button = tk.Button(root, text="Undo", command=self.on_undo_click)
        self.undo_button.pack()

        # Variables to keep track of waypoint number and counter
        self.waypoint_number = 1
        self.counter = 0

        # Text widget to display coordinates
        self.coordinate_text = tk.Text(root, height=20, width=30)
        self.coordinate_text.pack(side=tk.LEFT, padx=10)

    def on_canvas_click(self, event):
        # Get coordinates of the click
        x, y = event.x, event.y

        # Assuming some conversion factor to get latitude and longitude from pixel coordinates
        latitude = convert_pixel_to_latitude(y, min_latitude, max_latitude, image_height)
        longitude = convert_pixel_to_longitude(x, min_longitude, max_longitude, image_width)

        # Append coordinates to arrays
        self.x_array.append(x)
        self.y_array.append(y)
        self.latitude_array.append(latitude)
        self.longitude_array.append(longitude)

        # Display the waypoint number on the canvas at the clicked point
        waypoint_number_text = f"{self.waypoint_number}"
        self.canvas.create_text(x, y, text=waypoint_number_text, fill="red", font=("Arial", 10, "bold"))

        # Increment the waypoint number counter
        self.waypoint_number += 1
        self.counter += 1

        # Update the Text widget with current coordinates
        self.update_coordinate_text()

        print("X Array:", self.x_array)
        print("Y Array:", self.y_array)
        print("Latitude Array:", self.latitude_array)
        print("Longitude Array:", self.longitude_array)

    def on_finish_click(self):
        # Finalize arrays or perform other actions
        print("Final X Array:", self.x_array)
        print("Final Y Array:", self.y_array)
        print("Final Latitude Array:", self.latitude_array)
        print("Final Longitude Array:", self.longitude_array)
        self.root.destroy()  # Close the GUI

    def on_undo_click(self):
        # Undo the last waypoint
        if self.x_array and self.y_array and self.latitude_array and self.longitude_array:
            last_x = self.x_array.pop()
            last_y = self.y_array.pop()

            # Find the corresponding canvas item (text) and remove it
            for item in self.canvas.find_all():
                item_coords = self.canvas.coords(item)
                if item_coords and int(item_coords[0]) == last_x and int(item_coords[1]) == last_y:
                    self.canvas.delete(item)

            # Pop the corresponding latitude and longitude
            last_latitude = self.latitude_array.pop()
            last_longitude = self.longitude_array.pop()

            # Decrement the waypoint number counter
            self.waypoint_number -= 1
            self.counter -= 1

            # Update the Text widget with current coordinates after undo
            self.update_coordinate_text()

            # Print x, y, latitude, and longitude arrays after undo for debugging
            print("After Undo - X Array:", self.x_array)
            print("After Undo - Y Array:", self.y_array)
            print("After Undo - Latitude Array:", self.latitude_array)
            print("After Undo - Longitude Array:", self.longitude_array)

    def update_coordinate_text(self):
        # Clear the Text widget
        self.coordinate_text.delete(1.0, tk.END)

        # Iterate through coordinates and update the Text widget
        for lon, lat in zip(self.longitude_array, self.latitude_array):
            self.coordinate_text.insert(tk.END, f"{lon},{lat}\n")

    def get_final_coordinates(self):
        return self.longitude_array, self.latitude_array

if __name__ == "__main__":
    image_path = 'map.png'

    root = tk.Tk()
    app = WaypointApp(root, image_path)
    root.mainloop()

    final_longitude, final_latitude = app.get_final_coordinates()
