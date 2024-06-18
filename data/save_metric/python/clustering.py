import csv
import pandas as pd
import plotly
import plotly.graph_objs as go
from matplotlib import pyplot as plt
import numpy as np
from scipy.spatial import distance

from sklearn.decomposition import PCA

max_n_div = 2
metrics_directory = "./metrics/not_filling_peso/"
results_directory = "./results/not_filling_peso/"

activate_print = False

##################################################################################################

def print_info(activate, arg1, arg2=""):
    if(activate):
        #print(arg1)
        #print(arg2)
        print(str(arg1) + str(arg2))

## Plot data
def plot(data):
    #fig1 = go.Scatter3d(x=data[:,x], y=data[:,y], z=data[:,z],text=data[:,0], marker=dict(color=data[:,d],symbol=data[:,obj], opacity=1, reversescale=True, colorscale='Blues', size=5), mode='markers')
    fig1 = go.Scatter3d(x=data[:,x], y=data[:,y], z=data[:,z],text=data[:,0], marker=dict(color=data[:,d], opacity=1, reversescale=True, colorscale='Blues', size=5), mode='markers')
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

def classes(dataframe_array):
    print("\033[96m Getting samples of each class \033[0m")

    class_A = []
    class_B = []
    class_C = []
    class_D = []
    class_E = []
    class_F = []
    class_G = []
    for row in dataframe_array:
        if row[1] == "A":
            class_A.append(row)
        if row[1] == "B":
            class_B.append(row)
        if row[1] == "C":
            class_C.append(row)
        if row[1] == "D":
            class_D.append(row)
        if row[1] == "E":
            class_E.append(row)
        if row[1] == "F":
            class_F.append(row)
        if row[1] == "G":
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
def centroid(class_samples, n_div):
    print("\033[96m Computing centroid of cluster \033[0m")

    centroid = []
    length = len(class_samples[:,0])

    ## Compute centroid of each dimension
    for i in range(3,n_div*n_div+3): ## M1 is in the third column of the CSV
        centroid.append(np.sum(class_samples[:,i])/length)

    print_info(activate_print, "Class centroid: "+str(centroid))

    return centroid

#Inter / Intra
def inter_intra(tot_inter_intra, n_div, centroid, intra_class, inter_class_1, inter_class_2, inter_class_3="", inter_class_4="", inter_class_5="", inter_class_6=""):
    print("\033[96m Computing intra distance \033[0m")
    activate_print=True
    print_info(activate_print, "Centroid: "+str(centroid))
    max_intra_dist = 0    ## Max distance to samples of same cluster
    min_inter_dist = 1000 ## Min distance to another cluster

    # Compute intra distance (same cluster)
    for row in intra_class:
        print_info(activate_print, "File: "+row[0])
        p = row[3:n_div*n_div+3]
        dist = distance.euclidean(p,centroid)
        if dist > max_intra_dist:
            max_intra_dist = dist
            print_info(activate_print, "Point with max intra distance " + str(max_intra_dist), " -> File: "+row[0])
    #inter_intra_wr.writerow(["A-Intra_dist",max_intra_dist])
    #inter_intra_wr.writerow(["A-Inter_dist",max_inter_dist])

    min_inter_dist_1 = 1000
    min_inter_dist_2 = 1000
    min_inter_dist_3 = 1000
    print("\033[96m Computing inter distance \033[0m")
    #Compute inter distance (between other clusters)
    for row in inter_class_1:
        #print("Class 1")
        #p = [row[x], row[y], row[z], row[d]]
        p = row[3:n_div*n_div+3]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist_1:
            min_inter_dist_1 = dist
            print_info(activate_print, "Class 1 point with min inter distance "+ str(min_inter_dist_1), " -> File: "+ row[0])
    for row in inter_class_2:
        #print("Class 2")
        p = row[3:n_div*n_div+3]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist_2:
            min_inter_dist_2 = dist
            print_info(activate_print, "Class 2 point with min inter distance "+ str(min_inter_dist_2), " -> File: "+ row[0])
    for row in inter_class_3:
        #print("Class 3")
        p = row[3:n_div*n_div+3]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist_3:
            min_inter_dist_3 = dist
            print_info(activate_print, "Class 3 point with min inter distance "+ str(min_inter_dist_3), " -> File: "+ row[0])
    for row in inter_class_4:
        #print("Class 4")
        p = row[3:n_div*n_div+3]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print_info(activate_print, "Class 4 point with min inter distance "+ str(min_inter_dist), " -> File: "+ row[0])
    for row in inter_class_5:
        p = row[3:n_div*n_div+3]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print_info(activate_print, "Class 5 point with min inter distance "+ str(min_inter_dist), " -> File: "+ row[0])
    for row in inter_class_6:
        p = row[3:n_div*n_div+3]
        dist = distance.euclidean(p,centroid)
        if dist < min_inter_dist:
            min_inter_dist = dist
            print_info(activate_print, "Class 6 point with min inter distance "+ str(min_inter_dist), " -> File: "+ row[0])


    print("Max INTRA distance: ", max_intra_dist)
    #print("Min INTER distance: ", min_inter_dist)
    print("Min INTER distance 1: ", min_inter_dist_1)
    print("Min INTER distance 2: ", min_inter_dist_2)
    print("Min INTER distance 3: ", min_inter_dist_3)

    return max_intra_dist, min_inter_dist_1, min_inter_dist_2, min_inter_dist_3


##################################################################################################

## Save CSV with inter and intra distances
# inter_intra_file = metrics_directory + "inter_intra" + ".csv"
# my_file = open(inter_intra_file, "wb")
# inter_intra_wr = csv.writer(my_file, delimiter=",")
##inter_intra_wr.writerow(["","2x2"])

#tot_inter_intra = [[""],["A-Intra_dist"],["A-Inter_dist"],["D-Intra_dist"],["D-Inter_dist"]]#,["A-Inter_dist"]]
tot_inter_intra = [["4x4"],["Class_A"],["Class_B"],["Class_C"],["Class_D"]]#,["A-Inter_dist"]]
## Load data
for i in range(2,max_n_div+1): ## Use all the def metric files with different n_buckets
    print("\033[92m ----- Clustering for N Buckets: \033[96m "+str(i)+"x"+str(i)+"\033[92m ----- \033[0m")

    inter_intra_file = metrics_directory + str(i) + "x" + str(i) + "/inter_intra" + str(i) + "x" + str(i) +".csv"
    my_file = open(inter_intra_file, "wb")
    inter_intra_wr = csv.writer(my_file, delimiter=",")

    metrics_csv_file = str(i) + "x" + str(i) + "/" + str(i) + "x" + str(i) + ".csv"
    metrics_csv_dir = metrics_directory+metrics_csv_file

    data = pd.read_csv(metrics_csv_dir)
    data = data.to_numpy()

    ## Separate clusters
    class_A, class_B, class_C, class_D, class_E, class_F, class_G = classes(data)

    centr_class_A = centroid(class_A, i)
    centr_class_B = centroid(class_B, i)
    centr_class_C = centroid(class_C, i)
    centr_class_D = centroid(class_D, i)

    ## Copute inter distance
    print("Class A: ")
    #max_intra_dist_A, min_inter_dist_A = inter_intra(tot_inter_intra, i, centr_class_A, class_A, class_B, class_C, class_D)
    max_intra_dist_A, min_inter_dist_AB, min_inter_dist_AC, min_inter_dist_AD = inter_intra(tot_inter_intra, i, centr_class_A, class_A, class_B, class_C, class_D)
    max_intra_dist_B, min_inter_dist_BA, min_inter_dist_BC, min_inter_dist_BD = inter_intra(tot_inter_intra, i, centr_class_B, class_B, class_A, class_C, class_D)
    max_intra_dist_C, min_inter_dist_CA, min_inter_dist_CB, min_inter_dist_CD = inter_intra(tot_inter_intra, i, centr_class_C, class_C, class_A, class_B, class_D)
    max_intra_dist_D, min_inter_dist_DA, min_inter_dist_DB, min_inter_dist_DC = inter_intra(tot_inter_intra, i, centr_class_D, class_D, class_A, class_B, class_C)

    ## Add new data to matrix containing previous inter and intra distances
    #current_inter_intra = [[str(i) + "x" + str(i)],[max_intra_dist_A],[min_inter_dist_A],[max_intra_dist_A],[min_inter_dist_D]]
    current_inter_intra = [["Class A"],[max_intra_dist_A],[min_inter_dist_AB],[min_inter_dist_AC],[min_inter_dist_AD]]
    tot_inter_intra = np.hstack([tot_inter_intra, current_inter_intra])
    current_inter_intra = [["Class B"],[min_inter_dist_BA],[max_intra_dist_B],[min_inter_dist_BC],[min_inter_dist_BD]]
    tot_inter_intra = np.hstack([tot_inter_intra, current_inter_intra])
    current_inter_intra = [["Class C"],[min_inter_dist_CA],[min_inter_dist_CB],[max_intra_dist_C],[min_inter_dist_CD]]
    tot_inter_intra = np.hstack([tot_inter_intra, current_inter_intra])
    current_inter_intra = [["Class D"],[min_inter_dist_DA],[min_inter_dist_DB],[min_inter_dist_DC],[max_intra_dist_D]]
    tot_inter_intra = np.hstack([tot_inter_intra, current_inter_intra])
    #print(tot_inter_intra)
    np.savetxt(inter_intra_file, tot_inter_intra, fmt='%s', delimiter=',')
    tot_inter_intra = [["4x4"],["Class_A"],["Class_B"],["Class_C"],["Class_D"]]


#################

# #    with open(file_name) as csv_file:
# #        csv_reader = csv.reader(csv_file, delimiter=',')
# #        for row in csv_reader:
# #            print(row)
# #
#
# ## Load data
# file_name = "./all.csv"
# obj = 0
# clas = 1
# x = 2
# y = 3
# z = 4
# d = 5
#
# #Pandas
# data = pd.read_csv(file_name)
# #data=data[:,0].replace("o1","square").replace("o2","circle")
# data = data.to_numpy()
# #Numpy
# #data = np.genfromtxt(file_name,delimiter=",")
#
# ## Plot points
# plot(data)
#
# ## Separate clusters
# #class_A, class_B, class_C, class_D, class_E, class_F, class_G = classes(data)
#
#
# ## Compute centroid of points
# #cent_x, cent_y, cent_z, cent_d = centroid(class_A)
# #centr_class_A = centroid(class_A)
# #centr_class_B = centroid(class_B)
# #centr_class_C = centroid(class_C)
# #centr_class_D = centroid(class_D)
# #centr_class_E = centroid(class_E)
# #centr_class_F = centroid(class_F)
# #centr_class_G = centroid(class_G)
#
# #plot_class(class_A, centr_class_A)
# #plot_class(class_B, centr_class_B)
# #plot_class(class_C, centr_class_C)
# #plot_class(class_D, centr_class_D)
#
# ### Copute inter distance
# #print("Class A: ")
# #inter_intra(centr_class_A, class_A, class_B, class_C, class_D, class_E, class_F, class_G)
# #print("Class B: ")
# #inter_intra(centr_class_B, class_B, class_A, class_C, class_D, class_E, class_F, class_G)
# #print("Class C: ")
# #inter_intra(centr_class_C, class_C, class_B, class_A, class_D, class_E, class_F, class_G)
# #print("Class D: ")
# #inter_intra(centr_class_D, class_D, class_B, class_C, class_A, class_E, class_F, class_G)
# #print("Class E: ")
# #inter_intra(centr_class_E, class_E, class_B, class_C, class_D, class_A, class_F, class_G)
# #print("Class F: ")
# #inter_intra(centr_class_F, class_F, class_B, class_C, class_D, class_E, class_A, class_G)
# #print("Class G: ")
# #inter_intra(centr_class_G, class_G, class_B, class_C, class_D, class_E, class_F, class_A)
#
#
# ### Classification
# #classes_filename = "./classes.csv"
# #my_file = open(classes_filename, "wb")
# #wr = csv.writer(my_file, quoting=csv.QUOTE_NONNUMERIC)
# #
# #with open(file_name) as csv_file:
# #    csv_reader = csv.reader(csv_file, delimiter=',')
# #    for row in csv_reader:
# #        print(row[4])
# #        print(row[4]<1)
# #        if row[4] < 0.203:
# #            print(row[4])
# #            wr.writerow((row,'class1'))
