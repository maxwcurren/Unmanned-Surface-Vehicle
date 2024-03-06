from datetime import datetime


test_lon = 300.2345
test_lat = -117.6452
test_or = 67

file_path = r"C:\Users\maxcu\OneDrive\Desktop\SeniorProject\Boat\USV_Locations.txt"
file = open(file_path, "a")
file.write(f"Longitude: {test_lon} Latitude: {test_lat} Orientation: {test_or}")
file.write(datetime.today().strftime('%Y-%m-%d %H:%M:%S')+"\n")
file.close()
