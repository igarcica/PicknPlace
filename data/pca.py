import csv
import pandas as pd
import plotly
import plotly.graph_objs as go
from matplotlib import pyplot as plt
import numpy as np
from scipy.spatial import distance

from sklearn.decomposition import PCA


## Plot data
def plot(data):
    fig1 = go.Scatter3d(x=data[:,x], y=data[:,y], z=data[:,z], marker=dict(color=data[:,d],symbol=data[:,obj], opacity=1, reversescale=True, colorscale='Blues', size=5), mode='markers')
    mylayout = go.Layout(scene=dict(xaxis=dict(title="1"), yaxis=dict(title="2"), zaxis=dict(title="3")))
    plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)

def plot_class(data, centr):
    print(data)
    print(centr)
#    data = data.append(max_intra)
#    data = data.append(min_inter)
    data = np.append(data, [centr,centr], axis=0)
    print(data)
    fig1 = go.Scatter3d(x=data[:,x], y=data[:,y], z=data[:,z], marker=dict(color=data[:,d],symbol=data[:,obj], opacity=1, reversescale=True, colorscale='Blues', size=5), mode='markers')
    mylayout = go.Layout(scene=dict(xaxis=dict(title="1"), yaxis=dict(title="2"), zaxis=dict(title="3")))
    plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)

def clusters(points):
    print("Clustering...")
    class_A = []
    class_B = []
    class_C = []
    class_D = []
    class_E = []
    class_F = []
    class_G = []
    for row in points:
        if row[3] == "A":
            class_A.append(row)
        if row[3] == "B":
            class_B.append(row)
        if row[3] == "C":
            class_C.append(row)
        if row[3] == "D":
            class_D.append(row)
        if row[3] == "E":
            class_E.append(row)
        if row[3] == "F":
            class_F.append(row)
        if row[3] == "G":
            class_G.append(row)
    class_A = np.array(class_A)
    class_B = np.array(class_B)
    class_C = np.array(class_C)
    class_D = np.array(class_D)
    class_E = np.array(class_E)
    class_F = np.array(class_F)
    class_G = np.array(class_G)
    return class_A, class_B, class_C, class_D, class_E, class_F, class_G

## Centroid
def centroid(points):
    print("Computing centroid of cluster...")
    length = len(points[:,0])
    cent_x = np.sum(points[:,x])/length
    cent_y = np.sum(points[:,y])/length
    cent_z = np.sum(points[:,z])/length
    cent_d = np.sum(points[:,d])/length
    centroid = [cent_x, cent_y, cent_z, cent_d]
#    centroid = ["diamond",0,0,0,cent_x, cent_y, cent_z, cent_d]
#    print("HOLA:. .", centroid)
#    centroid = np.array(centroid)
#    print("HOLA:. .", centroid)
    return centroid

#Inter / Intra
def inter_intra(centroid, intra_class, inter_class_1, inter_class_2, inter_class_3, inter_class_4, inter_class_5, inter_class_6):
    print("Computing intra distance...")
    print("Centroid: ", centroid)
    max_intra_dist = 0
    min_inter_dist = 1000
    # Compute intra distance (same cluster)
    for row in intra_class:
        p = [row[x], row[y], row[z], row[d]]
        dist = distance.euclidean(p,centroid)
        if dist > max_intra_dist:
            max_intra_dist = dist
            print("Point with max intra distance ", max_intra_dist, " -> Obj: ", row[0], " type: ", row[1], " exp: ", row[2], " class: ", row[3])

    print("Computing inter distance...")
    #Compute inter distance (between other clusters)
    for row in inter_class_1:
        p = [row[x], row[y], row[z], row[d]]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print("Point with min inter distance ", min_inter_dist, " -> Obj: ", row[0], " type: ", row[1], " exp: ", row[2], " class: ", row[3])
    for row in inter_class_2:
        p = [row[x], row[y], row[z], row[d]]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print("Point with min inter distance ", min_inter_dist, " -> Obj: ", row[0], " type: ", row[1], " exp: ", row[2], " class: ", row[3])
    for row in inter_class_3:
        p = [row[x], row[y], row[z], row[d]]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print("Point with min inter distance ", min_inter_dist, " -> Obj: ", row[0], " type: ", row[1], " exp: ", row[2], " class: ", row[3])
    for row in inter_class_4:
        p = [row[x], row[y], row[z], row[d]]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print("Point with min inter distance ", min_inter_dist, " -> Obj: ", row[0], " type: ", row[1], " exp: ", row[2], " class: ", row[3])
    for row in inter_class_5:
        p = [row[x], row[y], row[z], row[d]]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print("Point with min inter distance ", min_inter_dist, " -> Obj: ", row[0], " type: ", row[1], " exp: ", row[2], " class: ", row[3])
    for row in inter_class_6:
        p = [row[x], row[y], row[z], row[d]]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print("Point with min inter distance ", min_inter_dist, " -> Obj: ", row[0], " type: ", row[1], " exp: ", row[2], " class: ", row[3])
    

    print("Max INTRA distance: ", max_intra_dist)
    print("Min INTER distance: ", min_inter_dist)

#    with open(file_name) as csv_file:
#        csv_reader = csv.reader(csv_file, delimiter=',')
#        for row in csv_reader:
#            print(row)
#

#################

## Load data
file_name = "./z_class.csv"
obj = 0
clas = 3
x = 4
y = 5
z = 6
d = 7

#Pandas
data = pd.read_csv(file_name)
#data=data[:,0].replace("o1","square").replace("o2","circle")
data = data.to_numpy()
#Numpy
#data = np.genfromtxt(file_name,delimiter=",")

## Plot points
plot(data)

## Separate clusters
class_A, class_B, class_C, class_D, class_E, class_F, class_G = clusters(data)


## Compute centroid of points
#cent_x, cent_y, cent_z, cent_d = centroid(class_A)
centr_class_A = centroid(class_A)
centr_class_B = centroid(class_B)
centr_class_C = centroid(class_C)
centr_class_D = centroid(class_D)
centr_class_E = centroid(class_E)
centr_class_F = centroid(class_F)
centr_class_G = centroid(class_G)

#plot_class(class_A, centr_class_A)
#plot_class(class_B, centr_class_B)
#plot_class(class_C, centr_class_C)
#plot_class(class_D, centr_class_D)

### Copute inter distance
print("Class A: ")
inter_intra(centr_class_A, class_A, class_B, class_C, class_D, class_E, class_F, class_G)
print("Class B: ")
inter_intra(centr_class_B, class_B, class_A, class_C, class_D, class_E, class_F, class_G)
print("Class C: ")
inter_intra(centr_class_C, class_C, class_B, class_A, class_D, class_E, class_F, class_G)
print("Class D: ")
inter_intra(centr_class_D, class_D, class_B, class_C, class_A, class_E, class_F, class_G)
print("Class E: ")
inter_intra(centr_class_E, class_E, class_B, class_C, class_D, class_A, class_F, class_G)
print("Class F: ")
inter_intra(centr_class_F, class_F, class_B, class_C, class_D, class_E, class_A, class_G)
print("Class G: ")
inter_intra(centr_class_G, class_G, class_B, class_C, class_D, class_E, class_F, class_A)


### Classification
#classes_filename = "./classes.csv"
#my_file = open(classes_filename, "wb")
#wr = csv.writer(my_file, quoting=csv.QUOTE_NONNUMERIC)
#
#with open(file_name) as csv_file:
#    csv_reader = csv.reader(csv_file, delimiter=',')
#    for row in csv_reader:
#        print(row[4])
#        print(row[4]<1)
#        if row[4] < 0.203:
#            print(row[4])
#            wr.writerow((row,'class1'))
