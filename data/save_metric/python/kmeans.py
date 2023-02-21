import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sb
from sklearn.cluster import KMeans
from sklearn.metrics import pairwise_distances_argmin_min
from mpl_toolkits.mplot3d import Axes3D

n_div = 2
n_grids = n_div*n_div

save_classification = True

write_directory = "./results/" + str(n_div) + "x" + str(n_div) + "/"

metrics_csv_file = str(n_div) + "x" + str(n_div) + ".csv" ##o1_2x2.csv
metrics_csv_dir = write_directory+metrics_csv_file

class_directory = "./results/"# + str(n_div) + "x" + str(n_div) + "/"
class_file = "test2.csv"
class_dir = class_directory+class_file

plt.rcParams['figure.figsize'] = (16, 9)
plt.style.use('ggplot')

GT_name_classes = ['A','B','C','D','E']

##################################################################################################

def info_dataframe(dataframe):
    print("--------Head dataframe--------")
    print(dataframe.head())

    print("--------Describe"--------"")
    print(dataframe.describe())
    print(dataframe.groupby('Class_GT').size()) ##Number of experiments in each grount truth class
    #dataframe.drop(['Class_GT'],1).hist() ## Dispersion
    #plt.show()
    #sb.pairplot(dataframe.dropna(), hue='Class_GT',size=4,vars=["M1","M2","M4"],kind='scatter') ##Cruzar 3 dimensiones para ver agrupaciones

def info_clusters(pred_classes):
    ## Getting the cluster centers
    # centroids = kmeans.cluster_centers_
    # print("----Centroids----")
    # print(centroids)

    #Number of trials in each predicted class
    cantidadGrupo =  pd.DataFrame()
    #cantidadGrupo['label']=pred_classes['label'].values
    cantidadGrupo['cantidad']=pred_classes.groupby('label').size()
    print("----Cantidad grupos----")
    print(cantidadGrupo)

    ## Diversidad de GT classes en cada predicted class
    predicted_classes = [0,1,2,3,4]
    related_GT_classes = []
    pred_name_classes = []

    for i in range(0,len(predicted_classes)):
        group_referrer_index = pred_classes['label']==i
        group_referrals = pred_classes[group_referrer_index]
        print(group_referrals)

        diversidadGrupo =  pd.DataFrame()
        diversidadGrupo['Class_GT_n']=[0,1,2,3,4]
        diversidadGrupo['cantidad']=group_referrals.groupby('Class_GT_n').size()
        print("----Diversidad de grupos----")
        print(diversidadGrupo)

        hola=diversidadGrupo["cantidad"]
        hola[np.isnan(hola)]=0
        print(hola)
        # Buscamos la clase de GT mayoritaria en la predicted class
        indice = diversidadGrupo['cantidad']==max(diversidadGrupo['cantidad'])
        test = diversidadGrupo[indice]
        test2 = test["Class_GT_n"]
        # print("Cantidad: ", max(diversidadGrupo['cantidad']))
        # print(indice)
        # print(test2)
        related_GT_classes.append(float(test2))
        pred_name_classes.append(GT_name_classes[int(test2)])


    print("Predicted classes: ", predicted_classes)
    print("Related GT classes: ", related_GT_classes)
    print("Class Names: ", pred_name_classes)

# def success_class():
#     s
##################################################################################################

print("Reading data file: ", metrics_csv_dir)
dataframe = pd.read_csv(metrics_csv_dir) #Load data
#info_dataframe(dataframe)

## Define training data: Deformation metrics (according to grid division)
metrics_name = ["M1","M2","M3","M4","M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]
metrics = []
for i in range(0,n_grids):
    metrics.append(metrics_name[i])
    train_data = np.array(dataframe[metrics])
#y = np.array(dataframe['Class_GT'])
print("Training with metrics ", metrics)
#print(train_data)
#print("Number trials and Metrics: ", train_data.shape)

## Fit cluster with deformation metric data
kmeans = KMeans(n_clusters=5).fit(train_data)

# Predicting the clusters
labels = kmeans.predict(train_data)
print("Labels: ", labels)

##Create new CSV with GT and predicted classes
pred_classes =  pd.DataFrame()
pred_classes['File']=dataframe['File'].values
pred_classes['Class_GT_n']=dataframe['Class_GT_n'].values
pred_classes['label']=labels
if(save_classification):
    pred_classes.to_csv(class_dir, index=False)

## vemos el representante del grupo, el usuario cercano a su centroid
closest, _ = pairwise_distances_argmin_min(kmeans.cluster_centers_, train_data)
#print(closest)
print("Closest exps to centroids: ")
users=dataframe['File'].values
for row in closest:
    print(users[row])

## Get prediction success rate
print("Reading data file: ", class_dir)
pred_classes_df = pd.read_csv(class_dir) #Load data

info_clusters(pred_classes_df)

# new_lab = kmeans.predict()
# print(new_lab)



###########################

#from sklearn.cluster import KMeans
#import numpy as np
#
#X = np.array
#
### Parameters
#n_div = 5 # Division
#print_info = False
#all_files = True
#
### Canonical object
#can_obj_file = 'o1_gr_can_seg.pcd'
##can_obj_file = 'o1_gr_e06_seg.pcd'
##filename = "o1_gr_e03_seg.pcd"
#save_img = "./plots/"
#
### CSV file to save def metric
#data_filename = "./classes.csv"
#my_file = open(data_filename, "wb")
#wr = csv.writer(my_file, delimiter=",")


#######  REFS
# https://www.aprendemachinelearning.com/k-means-en-python-paso-a-paso/
