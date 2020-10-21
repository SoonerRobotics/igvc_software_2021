from numpy import genfromtxt, savetxt, vectorize
from math import degrees


# Load data
gps_data_radians = genfromtxt("gps_log.csv", delimiter=",")

# vectorize the conversion between degrees and radians
rad_to_deg = vectorize(degrees)

# Convert to degrees and store in a numpy array
gps_data_degrees = rad_to_deg(gps_data_radians)

# Save the degrees version to a CSV
savetxt("gps_data_degrees.csv", gps_data_degrees, delimiter=",")