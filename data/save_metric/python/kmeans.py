import pandas as pd
import numpy as np
import csv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import seaborn as sb
import cv2
from sklearn.cluster import KMeans
from sklearn.metrics import pairwise_distances_argmin_min
from mpl_toolkits.mplot3d import Axes3D
import math

n_div = 2
n_grids = n_div*n_div

get_total_success = True
save_classification = True

#dataset_directory = "/home/pal/Desktop/more_data/dataset/RGB/"
dataset_directory = "/home/pal/Desktop/all/RGB/"
#write_directory = "./new_results/filling/" #+ str(n_div) + "x" + str(n_div) + "/"
#write_directory = "./new_folds/filling/" + str(n_div) + "x" + str(n_div) + "/"
write_directory = "./results/filling/"

metrics_csv_file = str(n_div) + "x" + str(n_div) + ".csv" ##o1_2x2.csv
metrics_csv_dir = write_directory+metrics_csv_file

#class_directory = "./new_folds/filling/" + str(n_div) + "x" + str(n_div) + "/"
class_directory = "./results/filling/"# + str(n_div) + "x" + str(n_div) + "/"
class_file = "clusters.csv"
class_dir = class_directory+class_file


plt.rcParams['figure.figsize'] = (16, 9)
plt.style.use('ggplot')

#GT_name_classes = ['Z','A','B','C','D','E']
#GT_name_classes_unk = ['Z','A','B','C','D','E','X','Y','Z','M','N','J']
#classes = ["Z","A","A","B","B","B","B","C","C","C","A","A","B","Z","E","E","E","D","D","D","D","D","D","D","D","D","Z","D","D","D","E","D","D","E","D","E","D","D","D","Z","A","A","A","C","C","C","A","A","A","B","B","B","Z","D","D","D","D","D","D","E","D","D","D","D","E"]

GT_name_classes = ['A','B','C','D','E']
GT_name_classes_unk = ['A','B','C','D','E','X','Y','Z','M','N','J']
pred_classes2 = [0,1,2,3,4]

##################################################################################################

def info_dataframe(dataframe):
    print("--------Head dataframe--------")
    print(dataframe.head())

    print("--------Describe--------")
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
    predicted_classes = pred_classes2#[0,1,2]#,3]#,4,5]#,6,7]
    related_GT_classes = []
    pred_name_classes = []
    combined = []
    combined_class_name = []
    clusters_files = []
    l=8
    n=1

    for i in range(0,len(predicted_classes)):
        print("----------------------------------------------------------------------------------")
        group_referrer_index = pred_classes['label']==i
        group_referrals = pred_classes[group_referrer_index]
        print(group_referrals)
        ## Group file names that each cluster is composed
        cluster_files = group_referrals['File'].values
        clusters_files.append(np.array(cluster_files))

        diversidadGrupo =  pd.DataFrame()
        diversidadGrupo['Class_GT_n']=pred_classes2#[0,1,2]#,3]#,4,5]#,6,7]
        diversidadGrupo['cantidad']=group_referrals.groupby('Class_GT_n').size()
        print("----Diversidad de grupos----")
        print(diversidadGrupo)

        put_zero=diversidadGrupo["cantidad"]
        put_zero[np.isnan(put_zero)]=0

        divGrupo = diversidadGrupo.to_numpy()
        #print(divGrupo)

        # Buscamos la clase de GT mayoritaria en la predicted class
        indices = diversidadGrupo['cantidad']==max(diversidadGrupo['cantidad'])
        max_class = diversidadGrupo[indices]
        repr_class = max_class["Class_GT_n"]
        print("Max class: ", max_class)
        print("Repr. class: ", repr_class)

        if(len(max_class)>1):
            print("\033[96m Empate \033[0m")
            for i in range(len(indices)):
                if(indices[i]):
                    print(indices[i])
                    indice = indices[i]
                    repr_class = int(divGrupo[i,0])
                    print("Max class: ", max_class)
                    combined.append(int(repr_class))
                    combined_class_name.append(GT_name_classes_unk[int(repr_class)])
            related_GT_classes.append(combined)
            pred_name_classes.append(combined_class_name)

        # else:
        #     max_class = diversidadGrupo[indices]
        #     repr_class = max_class["Class_GT_n"]
        #     related_GT_classes.append(int(repr_class))
        #     pred_name_classes.append(GT_name_classes_unk[int(repr_class)])

    ## Si la clase ya ha sido asignada

        print("\033[96m CHECK \033[0m")
        # max_class = diversidadGrupo[indices]
        # repr_class = float(max_class["Class_GT_n"])
        print(related_GT_classes)
        if(float(repr_class) in related_GT_classes):
            print("\033[96m Class already given \033[0m")
            print(repr_class)
            print("New class: ")
            print(str(GT_name_classes[int(repr_class)]+str(n)))
            new_class = GT_name_classes[int(repr_class)]+str(n)
            related_GT_classes.append(l)
            pred_name_classes.append(new_class)
            n+=1
            l+=1
        else:
            print("\033[96m Repr. class: \033[0m", repr_class)
            related_GT_classes.append(int(repr_class))
            pred_name_classes.append(GT_name_classes[int(repr_class)])


    print("Predicted classes: ", predicted_classes)
    print("Related GT classes: ", related_GT_classes)
    print("Class Names: ", pred_name_classes)

    return clusters_files, related_GT_classes, pred_name_classes

def plot_results(clusters_files):
    print("\033[96m Plotting files \033[0m")
    # #read img
    # #creat subplot sizes
    # # _, axs = plt.subplots(n_row, n_col, figsize=(12,12))
    # # axs = axs.flatten()
    # print(clusters_files) #Size: n clusters, n samples in each clusters
    # print(len(clusters_files))
    # #plt.figure(figsize=(10,10))
    # fig, axs = plt.subplots(len(clusters_files), len(clusters_files[0]))
    # for i in range(0,len(clusters_files)): # number of clusters_files
    #     for n in range(0,len(clusters_files[i])):#len(clusters_files[i])): #number of samples in cluster
    #         image_file = dataset_directory + clusters_files[i][n] + ".png"
    #         #print(image_file)
    #         img = mpimg.imread(image_file)
    #         #plt.subplot(1,len(clusters_files[i]),i+1)
    #         #plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    #         #plt.show()
    #         axs[i, n].imshow(img)
    #         #axs[i, n].set_title('Axis [0, 0]')
    # plt.show()

    file = 0
    for i in range(0,len(clusters_files)): # number of clusters_files -3
        size = np.sqrt(len(clusters_files[i])) # images grid
        print(size)
        size = int(math.ceil(size))
        fig, axs = plt.subplots(size, size)
        print(size)
        for n in range(0,size): #len(clusters_files[i])): #number of samples in cluster
            for m in range(0,size):
                #print("n: ", n, " / m: ", m, " / file: ", file, " / size: ", size)
                image_file = dataset_directory + clusters_files[i][file] + ".png"
                #print(image_file)
                img = mpimg.imread(image_file)
                axs[n, m].imshow(img)
                axs[n, m].set_title(clusters_files[i][file])
                if(file < len(clusters_files[i])-1):
                    file +=1
                else:
                    file = 0
        file = 0

        plt.show()

def success_ratio(class_dir, labels, file_n, tot_success_wr):
    print("Computing success ratio...")

    ## Get prediction success rate
    print("Reading data file: ", class_dir)
    pred_classes_df = pd.read_csv(class_dir) #Load data

    clusters_files, related_GT_classes, pred_name_classes = info_clusters(pred_classes_df)

    my_file = open(class_dir, "wb")
    class_wr = csv.writer(my_file, delimiter=",")
    copy =  pd.DataFrame()
    copy['File']=pred_classes_df['File'].values
    copy['Class_GT']=pred_classes_df['Class_GT_n'].values
    copy['label_name']=labels
    copy_n = copy.to_numpy()
    class_d = np.empty([1,3])
    for row in (copy_n):
        #print(pred_name_classes[int(row[2])])
        new_row = [row[0], GT_name_classes[int(row[1])], pred_name_classes[int(row[2])]]
        class_d = np.vstack([class_d, new_row])
        class_wr.writerow(new_row)
    #print(class_d)

    success_file = "success_" + str(file_n) + "x" + str(file_n) + ".csv"
    my_file = open(class_directory+success_file, "wb")
    #my_file = open(class_directory+"success.csv", "wb")
    success_wr = csv.writer(my_file, delimiter=",")
    sum = 0
    start=True
    Z_succ = 0
    A_succ = 0
    B_succ = 0
    C_succ = 0
    D_succ = 0
    E_succ = 0
    for row in class_d:
        if start:
            start=False
        else:
            if(row[1]==row[2]):
                new_row = [row[0],row[1],row[2], 1]
                sum +=1
                if(row[2]=="Z"):
                    Z_succ +=1
                if(row[2]=="A"):
                    A_succ +=1
                if(row[2]=="B"):
                    B_succ +=1
                if(row[2]=="C"):
                    C_succ +=1
                if(row[2]=="D"):
                    D_succ +=1
                if(row[2]=="E"):
                    E_succ +=1
            else:
                new_row = [row[0],row[1],row[2], 0]
            success_wr.writerow(new_row)
    total_success = (float(sum)/float(90))*100
    print("Success: ", total_success)
    # success_wr.writerow(["Total", "-", "-",total_success])
    # success_wr.writerow(["Success Z", "-","-",float(Z_succ)/float(5)])
    # success_wr.writerow(["Success A", "-","-",float(A_succ)/float(24)])#10
    # #success_wr.writerow(["Success B", "-","-",float(B_succ)/float(8)])
    # #success_wr.writerow(["Success C", "-","-",float(C_succ)/float(6)])
    # success_wr.writerow(["Success D", "-","-",float(D_succ)/float(29)])
    # success_wr.writerow(["Success E", "-","-",float(E_succ)/float(8)])

    success_wr.writerow(["Total", "-", "-",total_success])
    success_wr.writerow(["Success A", "-","-",(float(A_succ)/float(28))*100])#24)])
    success_wr.writerow(["Success B", "-","-",(float(B_succ)/float(45))*100])#25)])
    success_wr.writerow(["Success C", "-","-",(float(C_succ)/float(11))*100])
    success_wr.writerow(["Success D", "-","-",(float(D_succ)/float(6))*100])

#    for row in (tot_success_file):
#        print(row)
        # new_row = [row[0], (float(A_succ)/float(28))*100]
        # class_wr.writerow(new_row)
        # new_row = [row[0], (float(B_succ)/float(28))*100]
        # class_wr.writerow(new_row)
        # new_row = [row[0], (float(C_succ)/float(28))*100]
        # class_wr.writerow(new_row)
        # new_row = [row[0], (float(D_succ)/float(28))*100]
        # class_wr.writerow(new_row)


    #n_buckets = str(file_n) + "x" + str(file_n)
    #tot_success_wr.writerow([n_buckets,n_buckets])
    tot_success_wr.writerow([(float(A_succ)/float(28))*100])#24)])
    tot_success_wr.writerow([(float(B_succ)/float(45))*100])#25)])
    tot_success_wr.writerow([(float(C_succ)/float(11))*100])
    tot_success_wr.writerow([(float(D_succ)/float(6))*100])
    tot_success_wr.writerow([total_success])

    # for row in hola:
    #     writer.writerow(row+["hola",2,3,4,5])


    return success_file

def tot_success():

    # hola = []
    # hola = ["Success_A"]
    # hola = np.vstack([hola, "Success_B"])
    # hola = np.vstack([hola, "Success_C"])
    # hola = np.vstack([hola, "Success_D"])
    # hola = np.vstack([hola, "Total"])
    # print(hola)


    dir = write_directory+"all_success_column.csv"
    print(dir)
    tot_success_content = pd.read_csv(dir)
    # for row in range(0,6):
    #     new_wr.writerow(tot_success_content[0]+tot_success_content[6])
    # tot_success_content["new_col"] = [1,2,3,4,5]

    print(tot_success_content.values[0])

    success_file = "total_success.csv"
    my_file = open(class_directory+success_file, "wb")
    #my_file = open(class_directory+"success.csv", "wb")
    test_wr = csv.writer(my_file, delimiter=",")

    label = ["Success_A", "Success_B", "Success_C", "Success_D", "TOTAL"]
    test_wr.writerow(["","2x2","3x3","4x4","5x5"])
    for n in range(0,5):
        new_row = label[n]
        # new_row = np.append(new_row, tot_success_content.values[n])
        print(n)
        for i in range(n,n+20,5):
            print(i)
            new_row = np.append(new_row, tot_success_content.values[i])
        print(new_row)
        test_wr.writerow(new_row)
        new_row = []

def train_kmeans(metrics_csv_dir):

    print("Reading data file: ", metrics_csv_dir)
    dataframe = pd.read_csv(metrics_csv_dir) #Load data
    info_dataframe(dataframe)

    ## Define training data: Deformation metrics (according to grid division)
    metrics_name = ["M1","M2","M3","M4","M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]
    metrics = []
    for i in range(0,n_grids):
        text = "M"+str(i+1)
        metrics.append(text)
        train_data = np.array(dataframe[metrics])
        #metrics.append(metrics_name[i])
        #train_data = np.array(dataframe[metrics])
    #y = np.array(dataframe['Class_GT'])
    print("Training with metrics ", metrics)
    print(dataframe[metrics])
    #print("Number trials and Metrics: ", train_data.shape)

    ## Fit cluster with deformation metric data
    kmeans = KMeans(n_clusters=4, init='random').fit(train_data)
    #kmeans = KMeans(n_clusters=4).fit(train_data)

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

    return labels

##################################################################################################

if not get_total_success:
    success_file = "all_success_column.csv"
    my_file = open(class_directory+success_file, "wb")
    #my_file = open(class_directory+"success.csv", "wb")
    tot_success_wr = csv.writer(my_file, delimiter=",")


    tot_success_wr.writerow(["Success_rates"])
    for i in range(2,6):
        metrics_csv_file = str(i) + "x" + str(i) + ".csv" ##2x2.csv
        metrics_csv_dir = write_directory+metrics_csv_file

        ## Train kmeans with deformation data
        labels = train_kmeans(metrics_csv_dir)

        ## Get prediction success rate
        success_file = success_ratio(class_dir, labels, i, tot_success_wr)

else:
    tot_success()




#related_GT_classes.append(int(repr_class))
#pred_name_classes.append(GT_name_classes_unk[int(repr_class)])

#plot_results(clusters_files)

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
