
import matplotlib.pyplot as plt
import csv
import numpy as np

x = []
y = []
easting=[]
northing=[]

with open("stationaryGpsTopic.csv","r") as csvfile:
    plots = csv.reader(csvfile,delimiter=",")
    next(plots)
    for row in plots:
        x.append(float(row[1]))
        y.append(float(row[7]))
        easting.append(float(row[8]))
        northing.append(float(row[9]))

altmean = np.mean(y)
altstd = np.std(y)
eastingmean = np.mean(easting)
northingmean = np.mean(northing)
eastingstd = np.std(easting)
northingstd = np.std(northing)
print ("Easting Mean is :", eastingmean )
print ("Northing Mean is :", northingmean )
print ("Easting Standard Deviation is :",eastingstd )
print("Northing Standard Deciation is :",northingstd)
print("Altitude Mean is :",altmean)
print("Altitude Standard Deviation is :",altstd)
m,c = np.polyfit(x,y,1)

# m,c = np.polyfit(easting,northing,1)
d = [element*m +c for element in easting]
mline = [altmean for element in x]
plt.scatter(x,y ,label = "Easting and Northing", color = "green")
plt.plot(x,mline, linestyle= 'dashed', label = " Mean Altitude", color = "red")

error = 0

for i,j in northing,d:
    error += abs(i - j)
error = error / len(northing)
print(error)

# plt.scatter(x,y, color = 'g',)
# plt.scatter(a,b,color="r")
# plt.xticks(rotation=25)
plt.xlabel('Time')
plt.ylabel('Altitude')
plt.title('Altitude vs Time')
plt.legend()
plt.show()

