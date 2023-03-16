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

get_total_success = False

save_classification = True
save_kmeans_clusters = True
save_GT_classes = True

activate_print = False

#prectory = "/home/pal/Desktop/more_data/dataset/RGB/"
#dataset_directory = "/home/pal/Desktop/all/RGB/"
metrics_directory = "./metrics/filling/" #+ str(n_div) + "x" + str(n_div) + "/"
#write_directory = "./new_results/filling/" #+ str(n_div) + "x" + str(n_div) + "/"
#write_directory = "./new_folds/filling/" + str(n_div) + "x" + str(n_div) + "/"
write_directory = "./results/filling/1/"

metrics_csv_file = str(n_div) + "x" + str(n_div) + ".csv" ##o1_2x2.csv
metrics_csv_dir = write_directory+metrics_csv_file

#class_directory = "./new_folds/filling/" + str(n_div) + "x" + str(n_div) + "/"
class_directory = "./results/filling/1/"# + str(n_div) + "x" + str(n_div) + "/"
class_file = "clusters.csv"
class_dir = class_directory+class_file


plt.rcParams['figure.figsize'] = (16, 9)
plt.style.use('ggplot')

#GT_name_classes = ['Z','A','B','C','D','E']
#GT_name_classes_unk = ['Z','A','B','C','D','E','X','Y','Z','M','N','J']
#classes = ["Z","A","A","B","B","B","B","C","C","C","A","A","B","Z","E","E","E","D","D","D","D","D","D","D","D","D","Z","D","D","D","E","D","D","E","D","E","D","D","D","Z","A","A","A","C","C","C","A","A","A","B","B","B","Z","D","D","D","D","D","D","E","D","D","D","D","E"]

GT_name_classes = ['A','B','C','D']#,'E']
GT_name_classes_unk = ['A','B','C','D','E','X','Y','Z','M','N','J']
pred_classes2 = [0,1,2,3]#,4]

##################################################################################################

def print_info(activate, arg1, arg2=""):
    if(activate):
        #print(arg1)
        #print(arg2)
        print(str(arg1) + str(arg2))

def info_dataframe(dataframe, n_div):
    print("\033[96m Info Dataframe \033[0m")

    print_info(activate_print, "--------Head dataframe--------")
    print_info(activate_print, dataframe.head())

    print_info(activate_print, "--------Describe--------")
    print_info(activate_print, dataframe.describe())
    print_info(activate_print, dataframe.groupby('Class_GT').size()) ##Number of experiments in each grount truth class
    #dataframe.drop(['Class_GT'],1).hist() ## Dispersion
    #plt.show()
    #sb.pairplot(dataframe.dropna(), hue='Class_GT',size=4,vars=["M1","M2","M4"],kind='scatter') ##Cruzar 3 dimensiones para ver agrupaciones

    clusters_files = []
    for i in range(0,len(GT_name_classes)):
        group_referrer_index = dataframe['Class_GT']==GT_name_classes[i]
        group_referrals = dataframe[group_referrer_index]

        ## Group file names that each cluster is composed
        cluster_files = group_referrals['File'].values
        clusters_files.append(np.array(cluster_files))

    if(save_GT_classes):
        plot_GT_class(clusters_files, n_div, "")
        plot_GT_class(clusters_files, n_div, "_pat")

def plot_GT_class(GT_class_files, n_div, rgb_pat):
    print("\033[96m Plotting GT files \033[0m")
    file = 0
    for i in range(0,len(GT_class_files)): # number of classes
        size = np.sqrt(len(GT_class_files[i])) # images grid
        # size = int(math.ceil(size))
        # fig, axs = plt.subplots(size, size)
        size1 = int(math.ceil(size))
        size2 = int(round(size))
        fig, axs = plt.subplots(size2,size1)
        for n in range(0,size2): # number of samples in class
            for m in range(0,size1):
                directory = metrics_directory + str(n_div) + "x" + str(n_div) + "/"
                image_file = directory + GT_class_files[i][file] + str(rgb_pat) + ".png"
                img = mpimg.imread(image_file)
                axs[n, m].imshow(img)
                axs[n, m].set_title(GT_class_files[i][file])
                axs[n, m].set_xticks([]) ## Remove ticks in axis
                axs[n, m].set_yticks([])
                if(file < len(GT_class_files[i])-1):
                    file +=1
                else:
                    break;
        file = 0

        #plt.show()
        file_name = write_directory + str(n_div) + "x" + str(n_div) + "/GT_class_" + str(i) + str(rgb_pat) + ".png"
        print_info(activate_print, file_name)
        plt.savefig(file_name)
        plt.close()

def assign_label_clusters(pred_classes):
    print("\033[96m Assign label Clusters \033[0m")
    ## Getting the cluster centers
    # centroids = kmeans.cluster_centers_
    # print("----Centroids----")
    # print(centroids)

    #Number of trials in each predicted class
    cantidadGrupo =  pd.DataFrame()
    #cantidadGrupo['label']=pred_classes['label'].values
    cantidadGrupo['cantidad']=pred_classes.groupby('label').size()
    print_info(activate_print, "----Cantidad grupos----")
    print_info(activate_print, cantidadGrupo)

    ## Diversidad de GT classes en cada predicted class
    predicted_classes = pred_classes2
    related_GT_classes = []
    pred_name_classes = []
    combined = []
    combined_class_name = []
    clusters_files = []
    l=8
    n=1

    for i in range(0,len(predicted_classes)):
        print_info(activate_print, "----------------------------------------------------------------------------------")
        group_referrer_index = pred_classes['label']==i
        group_referrals = pred_classes[group_referrer_index]
        print_info(activate_print, group_referrals)
        ## Group file names that each cluster is composed
        cluster_files = group_referrals['File'].values
        clusters_files.append(np.array(cluster_files))

        diversidadGrupo =  pd.DataFrame()
        diversidadGrupo['Class_GT_n']=pred_classes2#[0,1,2]#,3]#,4,5]#,6,7]
        diversidadGrupo['cantidad']=group_referrals.groupby('Class_GT_n').size()
        print_info(activate_print, "----Diversidad de grupos----")
        print_info(activate_print, diversidadGrupo)

        put_zero=diversidadGrupo["cantidad"]
        put_zero[np.isnan(put_zero)]=0

        divGrupo = diversidadGrupo.to_numpy()
        #print(divGrupo)

        # Buscamos la clase de GT mayoritaria en la predicted class
        indices = diversidadGrupo['cantidad']==max(diversidadGrupo['cantidad'])
        max_class = diversidadGrupo[indices]
        repr_class = max_class["Class_GT_n"]
        print_info(activate_print, "Max class: ", max_class)
        print_info(activate_print, "Repr. class: ", repr_class)

        if(len(max_class)>1):
            print_info(activate_print, "\033[96m Empate \033[0m")
            for i in range(len(indices)):
                if(indices[i]):
                    print_info(activate_print, indices[i])
                    indice = indices[i]
                    repr_class = int(divGrupo[i,0])
                    print_info(activate_print, "Max class: ", max_class)
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


        # max_class = diversidadGrupo[indices]
        # repr_class = float(max_class["Class_GT_n"])
        print_info(activate_print, related_GT_classes)
        if(float(repr_class) in related_GT_classes):
            print_info(activate_print, "\033[96m Class label already assigned \033[0m")
            print_info(activate_print, repr_class)
            print_info(activate_print, "New class: ")
            print_info(activate_print, str(GT_name_classes[int(repr_class)]+str(n)))
            new_class = GT_name_classes[int(repr_class)]+str(n)
            related_GT_classes.append(l)
            pred_name_classes.append(new_class)
            n+=1
            l+=1
        else:
            print_info(activate_print, "\033[96m Repr. class: \033[0m", repr_class)
            related_GT_classes.append(int(repr_class))
            pred_name_classes.append(GT_name_classes[int(repr_class)])


    print_info(activate_print, "Predicted classes: ", predicted_classes)
    print_info(activate_print, "Related GT classes: ", related_GT_classes)
    print_info(activate_print, "Class Names: ", pred_name_classes)

    return clusters_files, related_GT_classes, pred_name_classes

## Plots the images that compose each cluster
def plot_results(clusters_files, n_div, rgb_pat):
    print("\033[96m Plotting clusters files \033[0m")
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
        #size = int(math.ceil(size))
        #fig, axs = plt.subplots(size, size)
        size1 = int(math.ceil(size))
        size2 = int(round(size))
        print(clusters_files[i])
        print(len(clusters_files[i]))
        print(size1, " /", size2)
        if(size1==1):
            print("1 sample in cluster")
            fig, ax = plt.subplots()
            directory = metrics_directory + str(n_div) + "x" + str(n_div) + "/"
            image_file = directory + clusters_files[i][0] + str(rgb_pat) + ".png"
            img = mpimg.imread(image_file)
            ax.imshow(img)
            ax.set_title(clusters_files[i][0])
            ax.set_xticks([]) ## Remove ticks in axis
            ax.set_yticks([])

            file_name = write_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + str(i) + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()
        if(size1 >1 and size2==1):
            print("2 samples in cluster")
        #     fig, asx = plt.subplots(size2,size1)
        #     directory = metrics_directory + str(n_div) + "x" + str(n_div) + "/"
        #     image_file = directory + clusters_files[i][0] + str(rgb_pat) + ".png"
        #     img = mpimg.imread(image_file)
        #     ax.imshow(img)
        #     ax.set_title(clusters_files[i][0])
        #     ax.set_xticks([]) ## Remove ticks in axis
        #     ax.set_yticks([])
            fig, axs = plt.subplots(1,size1) #(1,size1)
            #print(len(axs[0]))
            for m in range(0,size1):
                directory = metrics_directory + str(n_div) + "x" + str(n_div) + "/"
                image_file = directory + clusters_files[i][file] + str(rgb_pat) + ".png"
                print(image_file)
                img = mpimg.imread(image_file)
                axs[m].imshow(img) #axs[m]
                axs[m].set_title(clusters_files[i][file])
                axs[m].set_xticks([]) ## Remove ticks in axis
                axs[m].set_yticks([])
                if(file < len(clusters_files[i])-1):
                    file +=1
                else:
                    break;
            file = 0

            #plt.show()
            file_name = write_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + str(i) + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()
        else:
            print("more than 2 samples in cluster")
            fig, axs = plt.subplots(size2,size1)
            for n in range(0,size2): #len(clusters_files[i])): #number of samples in cluster
                for m in range(0,size1):
                    #print("n: ", n, " / m: ", m, " / file: ", file, " / size: ", size)
                    ## Plot all images of each cluster
                    directory = metrics_directory + str(n_div) + "x" + str(n_div) + "/"
                    image_file = directory + clusters_files[i][file] + str(rgb_pat) + ".png"
                    print(image_file)
                    img = mpimg.imread(image_file)
                    axs[n, m].imshow(img)
                    axs[n, m].set_title(clusters_files[i][file])
                    axs[n, m].set_xticks([]) ## Remove ticks in axis
                    axs[n, m].set_yticks([])
                    if(file < len(clusters_files[i])-1):
                        file +=1
                    else:
                        break;
                        #file = 0
            file = 0

            #plt.show()
            file_name = write_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + str(i) + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()

def success_ratio(pred_classes_df, labels, file_n, tot_success_wr):
    print("\033[96mComputing success ratio...\033[0m")

    # ## Get prediction success rate
    # print_info(activate_print, "Reading data file: ", class_dir)
    # print_info(activate_print, "\033[96m DSJFKSJDGSKDJGSDJ \033[0m")
    # print_info(activate_print, class_dir)
    # pred_classes_df = pd.read_csv(class_dir) #Load data
    # print(pred_class_df)
    #
    # clusters_files, related_GT_classes, pred_name_classes = assign_label_clusters(pred_classes_df)

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
        new_row = [row[0], GT_name_classes[int(row[1])], pred_name_classes[int(row[2])]] ##file_name, GT classes name, pred classes name
        class_d = np.vstack([class_d, new_row])
        class_wr.writerow(new_row)
    #print(class_d)

    success_file = write_directory + str(file_n) + "x" + str(file_n) + "/success_" + str(file_n) + "x" + str(file_n) + ".csv"
    my_file = open(success_file, "wb")
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
    print_info(activate_print, "Success: ", total_success)
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


    return clusters_files, success_file

def tot_success():
    print("\033[96m Total Success \033[0m")
    # hola = []
    # hola = ["Success_A"]
    # hola = np.vstack([hola, "Success_B"])
    # hola = np.vstack([hola, "Success_C"])
    # hola = np.vstack([hola, "Success_D"])
    # hola = np.vstack([hola, "Total"])
    # print(hola)


    dir = write_directory+"all_success_column.csv"
    print_info(activate_print, dir)
    tot_success_content = pd.read_csv(dir)
    # for row in range(0,6):
    #     new_wr.writerow(tot_success_content[0]+tot_success_content[6])
    # tot_success_content["new_col"] = [1,2,3,4,5]

    print_info(activate_print, tot_success_content.values[0])

    success_file = "total_success.csv"
    my_file = open(class_directory+success_file, "wb")
    #my_file = open(class_directory+"success.csv", "wb")
    test_wr = csv.writer(my_file, delimiter=",")

    label = ["Success_A", "Success_B", "Success_C", "Success_D", "TOTAL"]
    test_wr.writerow(["","2x2","3x3","4x4","5x5", "6x6", "7x7", "8x8", "9x9", "10x10"])
    for n in range(0,10):
        new_row = label[n]
        # new_row = np.append(new_row, tot_success_content.values[n])
        print_info(activate_print, n)
        for i in range(n,n+45,5):
            print_info(activate_print, i)
            new_row = np.append(new_row, tot_success_content.values[i])
        print_info(activate_print, new_row)
        test_wr.writerow(new_row)
        new_row = []

    print_info(activate_print, "Writing total success table in ", success_file)

## Trains kmeans with the computed metrics from a csv file
def train_kmeans(metrics_csv_dir, n_div):
    print("\033[96m Train KMEANS \033[0m")

    print_info(activate_print, "Reading data file: ", metrics_csv_dir)
    dataframe = pd.read_csv(metrics_csv_dir) ## Load data with deformation metrics
    info_dataframe(dataframe, n_div) ## Print info of deformation metrics dataframe and plot GT files

    ## Define training data: Deformation metrics (according to grid division)
    metrics = []
    for i in range(0,n_grids):
        text = "M"+str(i+1)
        metrics.append(text)
        train_data = np.array(dataframe[metrics])
        #metrics.append(metrics_name[i])
        #train_data = np.array(dataframe[metrics])
    #y = np.array(dataframe['Class_GT'])
    print_info(activate_print, "Training with metrics ", metrics)
    #print(dataframe[metrics])
    #print("Number trials and Metrics: ", train_data.shape)

    ## Fit cluster with deformation metric data
    kmeans = KMeans(n_clusters=4, init='random', n_init=20, max_iter=500, tol=1e-8, random_state=0).fit(train_data)

    # Predicting the clusters
    labels = kmeans.predict(train_data)
    print_info(activate_print, "Labels: ", labels) ## List of predicted cluster label to each sample

    ## Create new CSV with GT and predicted classes
    pred_classes =  pd.DataFrame()
    pred_classes['File']=dataframe['File'].values
    pred_classes['Class_GT_n']=dataframe['Class_GT_n'].values
    pred_classes['label']=labels
    if(save_classification):
        pred_classes.to_csv(class_dir, index=False)

    ## vemos el representante del grupo, el usuario cercano a su centroid
    closest, _ = pairwise_distances_argmin_min(kmeans.cluster_centers_, train_data)
    #print(closest)
    print_info(activate_print, "Closest exps to centroids: ")
    users=dataframe['File'].values
    for row in closest:
        print_info(activate_print, users[row])

    return labels, pred_classes

##################################################################################################

## Trains kmeans with the given metrics, writes a new csv file with the clusters and success rate based on GT classes
if not get_total_success:
    success_file = "all_success_column.csv"
    my_file = open(class_directory+success_file, "wb")
    tot_success_wr = csv.writer(my_file, delimiter=",")
    tot_success_wr.writerow(["Success_rates"])

    ## Use all the def metric files with different n_buckets
    for i in range(2,11):
        metrics_csv_file = str(i) + "x" + str(i) + "/" + str(i) + "x" + str(i) + ".csv"
        metrics_csv_dir = metrics_directory+metrics_csv_file

        ## Train kmeans with deformation data
        labels, pred_classes_df = train_kmeans(metrics_csv_dir, i)
        clusters_files, related_GT_classes, pred_name_classes = assign_label_clusters(pred_classes_df)

        ## Get prediction success rate
        clusters_files, success_file = success_ratio(pred_classes_df, labels, i, tot_success_wr)

        if(save_kmeans_clusters):
            print_info(activate_print, "\033[96m Plotting RGB files \033[0m")
            plot_results(clusters_files, i, "")
            print_info(activate_print, "\033[96m Plotting pattern files \033[0m")
            plot_results(clusters_files, i, "_pat")

## Writes a CSV file with the total success for several number of buckets
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
