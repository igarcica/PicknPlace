import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sb
from sklearn.cluster import KMeans
from sklearn.metrics import pairwise_distances_argmin_min
 
#matplotlib inline
from mpl_toolkits.mplot3d import Axes3D

plt.rcParams['figure.figsize'] = (16, 9)
plt.style.use('ggplot')

dataframe = pd.read_csv("./plots/o1o2_5x5.csv")
dataframe.head()

dataframe.describe()
print(dataframe.groupby('Class_GT').size())
#dataframe.drop(['Class_GT'],1).hist()
#plt.show()

#sb.pairplot(dataframe.dropna(), hue='Class_GT',size=4,vars=["M1","M2","M4"],kind='scatter')

X = np.array(dataframe[["M1","M2","M3","M4", "M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]])
#y = np.array(dataframe['Class_GT'])
print(X.shape)

#fig = plt.figure()
#ax = Axes3D(fig)
#colores=['blue','red','green','yellow']#,'cyan','yellow','orange','black','pink','brown','purple']
#asignar=[]
#for row in y:
#    asignar.append(colores[row])
#ax.scatter(X[:, 0], X[:, 1], c=asignar,s=60)
#plt.show()

##Obtener valor k
#Nc = range(1, 20)
#kmeans = [KMeans(n_clusters=i) for i in Nc]
#print(kmeans)
#score = [kmeans[i].fit(X).score(X) for i in range(len(kmeans))]
#print(score)
#plt.plot(Nc,score)
#plt.xlabel('Number of Clusters')
#plt.ylabel('Score')
#plt.title('Elbow Curve')
#plt.show()


kmeans = KMeans(n_clusters=7).fit(X)
centroids = kmeans.cluster_centers_
print("----Centroids----")
print(centroids)

# Predicting the clusters
labels = kmeans.predict(X)
# Getting the cluster centers
C = kmeans.cluster_centers_

colores=['red','green','blue','yellow']
#asignar=[]
#for row in labels:
#    asignar.append(colores[row])
#fig = plt.figure()
#ax = Axes3D(fig)
#ax.scatter(X[:, 0], X[:, 1], c=asignar,s=60)
#ax.scatter(C[:, 0], C[:, 1], marker='*', c=colores, s=1000)
#plt.show()

print("----Labels----")
print(labels)
pred_classes =  pd.DataFrame()
pred_classes['File']=dataframe['File'].values
pred_classes['Class_GT']=dataframe['Class_GT'].values
pred_classes['label'] = labels


cantidadGrupo =  pd.DataFrame()
cantidadGrupo['color']=colores
cantidadGrupo['cantidad']=pred_classes.groupby('label').size()
print("----Cantidad grupos----")
print(cantidadGrupo)


group_referrer_index = pred_classes['label'] ==0
group_referrals = pred_classes[group_referrer_index]
 
diversidadGrupo =  pd.DataFrame()
diversidadGrupo['Class_GT']=[0,1,2,3]
diversidadGrupo['cantidad']=group_referrals.groupby('Class_GT').size()
print("----Diversidad de grupos----")
print(diversidadGrupo)


#vemos el representante del grupo, el usuario cercano a su centroid
closest, _ = pairwise_distances_argmin_min(kmeans.cluster_centers_, X)
#print(closest)
users=dataframe['File'].values
for row in closest:
    print(users[row])



pred_classes.to_csv("./pred_classes.csv", index=False)






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
