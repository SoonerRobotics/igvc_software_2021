gps = read.csv("GPSData.csv")

# Latitude variance
lat.var = sd(gps$Latitude)
lat.var = lat.var * 111111
lat.var^2

# Longitude Variance
lon.var = sd(gps$Longitude)
lon.var = lon.var * 111111
lon.var^2


# Latitude test for normality
shapiro.test(gps$Latitude)


# Longitude test for normality
shapiro.test(gps$Longitude)


violin <- function(data, x_val, y_val, fill_col, title)
{
  library(ggplot2)
  ggplot(data, aes(x = x_val, y = y_val, fill = fill_col)) + geom_point() + ggtitle(title)
}

violin(gps, x_val = gps$Latitude, y_val = gps$Longitude, fill_col = "cyan", "yeet")

hist(gps$Latitude)
hist(gps$Longitude)

length(gps$Latitude)
