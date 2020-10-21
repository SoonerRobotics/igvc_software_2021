from math import radians, degrees, atan2, sin, cos, asin, fmod, pi

R = 6356    #6378.1 #Radius of the Earth
d = 12.37 / 1000 #Distance in km

lat1 = radians(35.210467)
lon1 = radians(-97.441811)
brng = radians(0)#radians(270.95)

# lat2 = asin( sin(lat1)*cos(d/R) + cos(lat1)*sin(d/R)*cos(brng))
#
# lon2 = lon1 + atan2(sin(brng)*sin(d/R)*cos(lat1), cos(d/R)-sin(lat1)*sin(lat2))
#
# lat2 = degrees(lat2)
# lon2 = degrees(lon2)

# Sketchy way
R = 6378137
v = 3.19023999999999 # velocity of 1 m/s
t = 1 # seconds since last update
lat2 = lat1 + (v * t) * cos(brng) / R
lon2 = lon1 + (v * t) * sin(brng) / (R * cos(lat1))

print(degrees(lat2), ",", degrees(lon2))

# import geopy
# from geopy.distance import VincentyDistance
#
# # given: lat1, lon1, b = bearing in degrees, d = distance in kilometers
# d = 12.37 / 1000
# lat1 = 35.210356  #Current lat point converted to radians
# lon1 = -97.442451 #Current long point converted to radians
# b = 0
#
# origin = geopy.Point(lat1, lon1)
# destination = VincentyDistance(kilometers=d).destination(origin, b)
#
# lat2, lon2 = destination.latitude, destination.longitude