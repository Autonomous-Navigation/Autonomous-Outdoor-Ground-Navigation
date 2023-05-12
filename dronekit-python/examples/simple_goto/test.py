from coordinates import fun
import time

start_lat =33.64514403550909 
start_lng =  -117.84273063954905
end_lat = 33.64585607700809
end_lng = -117.84198898453141

arr = fun(start_lat, start_lng, end_lat, end_lng)

for pts in arr:
    print(pts[0])
    time.sleep(1)
