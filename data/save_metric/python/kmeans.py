import pandas as pd
import numpy as np
import csv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import seaborn as sb
import cv2
from sklearn.cluster import KMeans
from sklearn.metrics import pairwise_distances_argmin_min
from sklearn.metrics import confusion_matrix
from mpl_toolkits.mplot3d import Axes3D
import math

max_n_div = 10

save_GT_classes = False          ## Save images with sample images of each GT class
save_predicted_clusters = False  ## Save images with sample images of each cluster
save_confusion_matrix = True     ## Save confusion matrix

activate_print = False ## Print info in terminal

metrics_directory = "./metrics_D/not_filling/"
results_directory = "./results_D/not_filling/"

plt.rcParams['figure.figsize'] = (16, 9)
plt.style.use('ggplot')

GT_name_classes = ['A','B','C','D']#,'E']
GT_name_classes_unk = ['A','B','C','D','E','X','Y','Z','M','N','J']
possible_classes_n = [0,1,2,3]#,4]

##################################################################################################

def print_info(activate, arg1, arg2=""):
    if(activate):
        print(str(arg1) + str(arg2))

## Plots the images that compose each GT class
def plot_GT_class(GT_class_files, n_div, rgb_pat):
    print("\033[96m Plotting GT files... \033[0m")
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
        #file_name = results_directory + str(n_div) + "x" + str(n_div) + "/GT_class_" + str(i) + str(rgb_pat) + ".png"
        file_name = results_directory + str(n_div) + "x" + str(n_div) + "/GT_class_" + GT_name_classes[i] +"_"+ str(rgb_pat) + ".png"
        print_info(activate_print, file_name)
        plt.savefig(file_name)
        plt.close()

## Plots the images that compose each cluster
def plot_results(clusters_files, n_div, pred_class_label, rgb_pat):
    print("\033[96m Plotting clusters files... \033[0m")

    file = 0
    for i in range(0,len(clusters_files)): # number of clusters_files -3
        size = np.sqrt(len(clusters_files[i])) # images grid
        #size = int(math.ceil(size))
        #fig, axs = plt.subplots(size, size)
        size1 = int(math.ceil(size))
        size2 = int(round(size))
        print_info(activate_print, clusters_files[i])
        if(size1==1):
            print_info(activate_print, "1 sample in cluster")
            fig, ax = plt.subplots()
            directory = metrics_directory + str(n_div) + "x" + str(n_div) + "/"
            image_file = directory + clusters_files[i][0] + str(rgb_pat) + ".png"
            img = mpimg.imread(image_file)
            ax.imshow(img)
            ax.set_title(clusters_files[i][0])
            ax.set_xticks([]) ## Remove ticks in axis
            ax.set_yticks([])

            #file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + str(i) + str(rgb_pat) + ".png"
            file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()
        if(size1 >1 and size2==1):
            print_info(activate_print, "2 samples in cluster")
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
                print_info(activate_print, image_file)
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
            #file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + str(i) + str(rgb_pat) + ".png"
            file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()
        else:
            print_info(activate_print, "more than 2 samples in cluster")
            fig, axs = plt.subplots(size2,size1)
            for n in range(0,size2): #len(clusters_files[i])): #number of samples in cluster
                for m in range(0,size1):
                    #print("n: ", n, " / m: ", m, " / file: ", file, " / size: ", size)
                    ## Plot all images of each cluster
                    directory = metrics_directory + str(n_div) + "x" + str(n_div) + "/"
                    image_file = directory + clusters_files[i][file] + str(rgb_pat) + ".png"
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
            #file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + str(i) + str(rgb_pat) + ".png"
            file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()

def success_ratio(pred_classes_df, n_div, assigned_class_labels, tot_success_array):
    print("\033[96m Computing success ratio \033[0m")

    ## Convert dataframe to array
    print_info(activate_print, pred_classes_df)
    pred_classes_array = pred_classes_df.to_numpy()
    pred_classes_name = np.empty([1,3])

    ## Create list of samples changing class number to the assigned class name
    for row in (pred_classes_array):
        #print(pred_name_classes[int(row[2])])
        new_row = [row[0], GT_name_classes[int(row[1])], assigned_class_labels[int(row[2])]] ##file_name, GT classes name, pred classes name
        pred_classes_name = np.vstack([pred_classes_name, new_row])
    print_info(activate_print, pred_classes_name)

    ## Get success rates of each cluster comparing assigned label with GT label
    success_file = results_directory + str(n_div) + "x" + str(n_div) + "/success_" + str(n_div) + "x" + str(n_div) + ".csv"
    my_file = open(success_file, "wb")
    success_wr = csv.writer(my_file, delimiter=",")
    sum = 0
    start=True
    Z_succ = 0
    A_succ = 0
    B_succ = 0
    C_succ = 0
    D_succ = 0
    E_succ = 0
    ## Sum success of each class and total
    for row in pred_classes_name:
        if start: ## Skip first row (column titles)
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

    tot_success_array = np.vstack([tot_success_array, (float(A_succ)/float(28))*100])
    tot_success_array = np.vstack([tot_success_array, (float(B_succ)/float(45))*100])
    tot_success_array = np.vstack([tot_success_array, (float(C_succ)/float(11))*100])
    tot_success_array = np.vstack([tot_success_array, (float(D_succ)/float(6))*100])
    tot_success_array = np.vstack([tot_success_array, total_success])

    return success_file, tot_success_array

def success_ratio_combine_GT_classes(pred_classes_df, n_div, GT_class_sizes, tot_success_array):
    print("\033[96m Computing success ratio \033[0m")

    ## All possible combinations of classes (for 4 GT classes)
    comb_classes = [["A","B","C","D"],["B","A","C","D"],["C","B","A","D"],["D","B","C","A"],["A","B","D","C"],["A","D","C","B"],["A","C","B","D"]]
    comb_classes_n = [[0,1,2,3],[1,0,2,3],[2,1,0,3],[3,1,2,0],[0,1,3,2],[0,3,2,1],[0,2,1,3]]

    ## Convert dataframe to array
    print_info(activate_print, pred_classes_df)
    pred_classes_array = pred_classes_df.to_numpy()
    #pred_classes_name = np.empty([1,3])
    prev_total_success = 0
    combinations_success_matrix = []

    ## Create list of samples changing class number to the assigned class name
    for i in range(0,len(comb_classes)):
        pred_classes_name = np.empty([1,3])
        #current_success_ratios = np.zeros([1,1])
        current_success_ratios = []
        for row in pred_classes_array:
            new_row = [row[0], GT_name_classes[int(row[1])], comb_classes[i][int(row[2])]] ##file_name, GT classes name, pred classes name
            pred_classes_name = np.vstack([pred_classes_name, new_row])
        pred_classes_name = np.delete(pred_classes_name,0,0) ## Delete first row (initialization)
        print_info(activate_print, pred_classes_name)

        ## Get success rates of each cluster comparing assigned label with GT label
        success_file = results_directory + str(n_div) + "x" + str(n_div) + "/success_" + str(n_div) + "x" + str(n_div) + "_"+str(i)+".csv"
        my_file = open(success_file, "wb")
        success_wr = csv.writer(my_file, delimiter=",")
        sum = 0
        Z_succ = 0
        A_succ = 0
        B_succ = 0
        C_succ = 0
        D_succ = 0
        E_succ = 0
        ## Sum success of each class and total
        for row in pred_classes_name:
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
            ## Write GT class, assigned label, and 0/1 if success or not
            success_wr.writerow(new_row)

        total_success = (float(sum)/float(GT_class_sizes[4]))*100
        print_info(activate_print, "Success: ", total_success)

        ## Write class success and total in the corresponding n_bucket file
        success_wr.writerow(["Total", "-", "-",total_success])
        success_wr.writerow(["Success A", "-","-",(float(A_succ)/float(GT_class_sizes[0]))*100])
        success_wr.writerow(["Success B", "-","-",(float(B_succ)/float(GT_class_sizes[1]))*100])
        success_wr.writerow(["Success C", "-","-",(float(C_succ)/float(GT_class_sizes[2]))*100])
        success_wr.writerow(["Success D", "-","-",(float(D_succ)/float(GT_class_sizes[3]))*100])

        ## Save class and total success in array
        current_success_ratios = (float(A_succ)/float(GT_class_sizes[0]))*100
        current_success_ratios = np.vstack([current_success_ratios, (float(B_succ)/float(GT_class_sizes[1]))*100])
        current_success_ratios = np.vstack([current_success_ratios, (float(C_succ)/float(GT_class_sizes[2]))*100])
        current_success_ratios = np.vstack([current_success_ratios, (float(D_succ)/float(GT_class_sizes[3]))*100])
        current_success_ratios = np.vstack([current_success_ratios, total_success])

        ## Save urrent class labels combination if success is higher
        if total_success > prev_total_success:
            #print("Max success: ", i)
            max_success_combination = i
            prev_total_success = total_success

        ## All class and total success for all GT combinations assignments
        combinations_success_matrix.append(current_success_ratios)
        print_info(activate_print, combinations_success_matrix)

    ## Maximum class and total success
    #tot_success_array = combinations_success_matrix[max_success_combination]
    tot_success_array = np.vstack([tot_success_array, combinations_success_matrix[max_success_combination]])
    print_info(activate_print, "Best success rates: ", tot_success_array)
    print_info(activate_print, "Best combo: ", comb_classes[max_success_combination])

    return success_file, tot_success_array, comb_classes[max_success_combination], comb_classes_n[max_success_combination]

def tot_success(success_ratios):
    print("\033[96m Total Success \033[0m")

    success_file = "total_success.csv"
    my_file = open(results_directory+success_file, "wb")
    test_wr = csv.writer(my_file, delimiter=",")

    success_title = ["Success_A", "Success_B", "Success_C", "Success_D", "TOTAL"]
    test_wr.writerow(["","2x2","3x3","4x4","5x5", "6x6", "7x7", "8x8", "9x9", "10x10"])
    for n in range(0,5):
        new_row = success_title[n]
        print_info(activate_print, "n"+str(n))
        for i in range(n,(max_n_div-1)*5,5):
            print_info(activate_print, i)
            new_row = np.append(new_row, success_ratios[i])
        print_info(activate_print, new_row)
        test_wr.writerow(new_row)
        new_row = []

    print_info(activate_print, "Writing total success table in ", success_file)

def info_dataframe(dataframe, n_div):
    print("\033[96m Info Dataframe \033[0m")

    print_info(activate_print, "--------Head dataframe--------")
    print_info(activate_print, dataframe.head())

    print_info(activate_print, "--------Describe--------")
    print_info(activate_print, dataframe.describe())
    print_info(activate_print, dataframe.groupby('Class_GT').size()) ##Number of experiments in each grount truth class
    GT_class_sizes = dataframe.groupby('Class_GT').size().values
    tot_class = 0
    for i_class in range(0,len(GT_class_sizes)):
        tot_class += GT_class_sizes[i_class]
    GT_class_sizes = np.append(GT_class_sizes, tot_class)
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

    return GT_class_sizes

def assign_label_clusters(pred_classes):
    print("\033[96m Assign label Clusters \033[0m")
    ## Getting the cluster centers
    # centroids = kmeans.cluster_centers_
    # print("----Centroids----")
    # print(centroids)

    ##Number of trials in each predicted class
    cantidadGrupo =  pd.DataFrame()
    #cantidadGrupo['label']=pred_classes['label'].values
    cantidadGrupo['cantidad']=pred_classes.groupby('label').size()
    print_info(activate_print, "----Cantidad grupos----")
    print_info(activate_print, cantidadGrupo)

    ## Diversidad de GT classes en cada predicted class
    predicted_classes = possible_classes_n
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
        diversidadGrupo['Class_GT_n']=possible_classes_n#[0,1,2]#,3]#,4,5]#,6,7]
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

def assign_label_clusters2(pred_classes):
    print("\033[96m Assign label Clusters \033[0m")

    ## Number of samples in each predicted class
    cantidadGrupo =  pd.DataFrame()
    #cantidadGrupo['label']=pred_classes['label'].values
    cantidadGrupo['cantidad']=pred_classes.groupby('label').size()
    print_info(activate_print, "----Cantidad grupos----")
    print_info(activate_print, cantidadGrupo)

    ## Diversidad de GT classes en cada predicted class
    predicted_classes = possible_classes_n
    assigned_class_numbers = []
    assigned_class_labels = []
    combined = []
    combined_class_name = []
    l=8
    n=1

    ## For each predicted cluster
    for i in range(0,len(predicted_classes)):
        print_info(activate_print, "----------------------------------------------------------------------------------")
        group_referrer_index = pred_classes['label']==i
        group_referrals = pred_classes[group_referrer_index]
        print_info(activate_print, group_referrals)

        diversidadGrupo =  pd.DataFrame()
        diversidadGrupo['Class_GT_n']=possible_classes_n
        diversidadGrupo['cantidad']=group_referrals.groupby('Class_GT_n').size()
        print_info(activate_print, "----Diversidad de grupos----")
        print_info(activate_print, diversidadGrupo)

        ## Replace empty (NaN) for 0
        put_zero=diversidadGrupo["cantidad"]
        put_zero[np.isnan(put_zero)]=0

        divGrupo = diversidadGrupo.to_numpy()
        #print(divGrupo)

        # Buscamos la clase de GT mayoritaria en la predicted class
        indices = diversidadGrupo['cantidad']==max(diversidadGrupo['cantidad'])
        max_class = diversidadGrupo[indices]
        repr_class = max_class["Class_GT_n"] #.values ## Dominant class

        ## If there are same number of samples of different classes -> create a combined class label
        if(len(max_class)>1):
            print_info(activate_print, "\033[96m Empate \033[0m")
            for i in range(len(indices)):
                if(indices[i]):
                    print_info(activate_print, indices[i])
                    indice = indices[i]
                    repr_class = int(divGrupo[i,0])
                    combined.append(int(repr_class))
                    combined_class_name.append(GT_name_classes_unk[int(repr_class)])
                    print_info(activate_print, "----Max class----")
                    print_info(activate_print, max_class)
                    print_info(activate_print, "\033[92m Assigning combined class: \033[0m", GT_name_classes_unk[int(repr_class)])
            assigned_class_numbers.append(combined)
            assigned_class_labels.append(combined_class_name)

        # else:
        #     max_class = diversidadGrupo[indices]
        #     repr_class = max_class["Class_GT_n"]
        #     assigned_class_numbers.append(int(repr_class))
        #     assigned_class_labels.append(GT_name_classes_unk[int(repr_class)])

        # max_class = diversidadGrupo[indices]
        # repr_class = float(max_class["Class_GT_n"])

        ## If the class name has already been assigned --> Add a number
        print_info(activate_print, assigned_class_numbers)
        if(float(repr_class) in assigned_class_numbers):
            print_info(activate_print, "\033[96m Class label already assigned \033[0m")
            print_info(activate_print, "\033[92m Repr. class: \033[0m", repr_class.values)
            print_info(activate_print, "\033[92m Assigning new class: \033[0m", str(GT_name_classes[int(repr_class)]+str(n)))
            new_class = GT_name_classes[int(repr_class)]+str(n)
            assigned_class_numbers.append(l)
            assigned_class_labels.append(new_class)
            n+=1
            l+=1
        ## Assign a GT label to the predicted cluster
        else:
            assigned_class_numbers.append(int(repr_class))
            assigned_class_labels.append(GT_name_classes[int(repr_class)])
            print_info(activate_print, "\033[92m Repr. class: \033[0m", repr_class.values)
            print_info(activate_print, "\033[92m Assigned class: \033[0m", GT_name_classes[int(repr_class)])

    print_info(activate_print, "----------------------------------------------------------------------------------")
    print_info(activate_print, "Predicted classes: ", predicted_classes)
    print_info(activate_print, "Assigned class numbers: ", assigned_class_numbers)
    print_info(activate_print, "Assigned class labels: ", assigned_class_labels)

    return assigned_class_numbers, assigned_class_labels

def get_clusters_files(pred_classes):
    print("\033[96m Getting clusters samples \033[0m")

    clusters_files = []
    predicted_classes = possible_classes_n
    for i in range(0,len(predicted_classes)):
        group_referrer_index = pred_classes['label']==i
        group_referrals = pred_classes[group_referrer_index]
        print_info(activate_print, group_referrals)
        ## Group file names that each cluster is composed
        cluster_files = group_referrals['File'].values
        clusters_files.append(np.array(cluster_files))
        print_info(activate_print, "CLUSTER FILES")

    return clusters_files

## Creates the confusion matrix
def compute_confusion_matrix(assigned_class_labels_n, pred_classes_df, n_div):
    print("\033[96m Plotting confusion matrix \033[0m")

    GT_pred_data = pred_classes_df.to_numpy() ## Dataframe containing: file name, GT class (int), predicted class (int)
    GT_classes = np.vstack(GT_pred_data[:, 1]).astype(np.int)   ## Get the GT classes as an array (int format)
    pred_classes = np.vstack(GT_pred_data[:, 2]).astype(np.int) ## Get the predicted classes as an array (int format)

    ## Create new array with the predicted classes changing the labels (int format) to the assigned ones
    correct_pred_classes_n = np.empty([1,1])
    for n in pred_classes:
        new_pred_label_n = assigned_class_labels_n[n[0]]
        correct_pred_classes_n = np.vstack([correct_pred_classes_n, new_pred_label_n])
    correct_pred_classes_n = np.delete(correct_pred_classes_n,0,0) ## Delete first row (initialization)

    ## Compute confusion matrix
    cm = confusion_matrix(GT_classes, correct_pred_classes_n)

    ## Plot (and save) confusion matrix
    if(save_confusion_matrix):
        plt.imshow(cm,interpolation='none',cmap='Greens')
        for (i, j), z in np.ndenumerate(cm):
            plt.text(j, i, z, ha='center', va='center', color='black') ##Write in plot cm values
            plt.grid(False)
            plt.xticks([0,1,2,3],['A', 'B', 'C', 'D'])
            plt.yticks([0,1,2,3],['A', 'B', 'C', 'D'])
            plt.xlabel("kmeans label")
            plt.ylabel("truth label")
            #plt.show()
            file_name = results_directory + str(n_div) + "x" + str(n_div) + "/confusion_mat.png"
            plt.savefig(file_name)
        plt.close()

## Trains kmeans with the computed metrics from a csv file
def train_kmeans(metrics_csv_dir, n_div):
    print("\033[96m Train KMEANS \033[0m")

    print_info(activate_print, "Reading data file: ", metrics_csv_dir)
    dataframe = pd.read_csv(metrics_csv_dir) ## Load data with deformation metrics
    GT_class_sizes = info_dataframe(dataframe, n_div) ## Print info of deformation metrics dataframe and plot GT files

    ## Define training data: Deformation metrics (according to grid division)
    metrics = []
    for i in range(0,n_div*n_div):
        text = "M"+str(i+1)
        metrics.append(text)
        train_data = np.array(dataframe[metrics])
    print_info(activate_print, "Training with metrics ", metrics)
    print_info(activate_print, "Number trials and Metrics: ", train_data.shape)

    # model = KMeans()
    # visualizer = KElbowVisualizer(model, k=(1,11))

    ## Fit cluster with deformation metric data
    #kmeans = KMeans(n_clusters=4, init='k-means++', n_init=20, max_iter=500, tol=1e-8, random_state=0).fit(train_data) #init kmeans ++ (starts selecting 1 centroid randomly)
    #kmeans = KMeans(n_clusters=4, init='random', n_init=20, max_iter=500, tol=1e-8, random_state=0).fit(train_data) ## init random (starts selecting k centroids randomly)

    ## Kmeans setting the initial centroids with samples from each class in the GT
    #init_centroids = [dataframe.iloc[0,3],dataframe.iloc[0,3],dataframe.iloc[0,3],dataframe.iloc[0,3]]
    init_centroids = np.empty([1,n_div*n_div])
    ## sample for class A
    sample = dataframe.iloc[7].values
    sample_point = sample[3:n_div*n_div+3]
    init_centroids = np.vstack([init_centroids, sample_point])
    print_info(True, "Sample used as centroid of class A: "+sample[0])
    ## sample for class B
    sample = dataframe.iloc[4].values
    sample_point = sample[3:n_div*n_div+3]
    init_centroids = np.vstack([init_centroids, sample_point])
    print_info(True, "Sample used as centroid of class B: "+sample[0])
    ## sample for class C
    sample = dataframe.iloc[31].values
    sample_point = sample[3:n_div*n_div+3]
    init_centroids = np.vstack([init_centroids, sample_point])
    print_info(True, "Sample used as centroid of class C: "+sample[0])
    ## sample for class D
    sample = dataframe.iloc[22].values #iloc[70].values
    sample_point = sample[3:n_div*n_div+3]
    init_centroids = np.vstack([init_centroids, sample_point])
    print_info(True, "Sample used as centroid of class D: "+sample[0])
    ## Create initial centroid points
    init_centroids = np.delete(init_centroids,0,0)
    #print(init_centroids)
    kmeans = KMeans(n_clusters=4, init=init_centroids, n_init=20, max_iter=500, tol=1e-8, random_state=None).fit(train_data) ## init random (starts selecting k centroids randomly)

    # Predicting the clusters
    labels = kmeans.predict(train_data)
    print_info(activate_print, "Labels: ", labels) ## List of predicted cluster to each sample

    ## Create new CSV with GT and predicted classes
    pred_classes =  pd.DataFrame()
    pred_classes['File']=dataframe['File'].values
    pred_classes['Class_GT_n']=dataframe['Class_GT_n'].values
    pred_classes['label']=labels

    ## Cluster representants (samples closest to the centroid)
    closest, _ = pairwise_distances_argmin_min(kmeans.cluster_centers_, train_data)
    print_info(activate_print, "Closest exps to centroids: ")
    users=dataframe['File'].values
    for row in closest:
        print_info(activate_print, users[row])

    return pred_classes, GT_class_sizes

##################################################################################################


## Trains kmeans with the given metrics, writes a new csv file with the clusters and success rate based on GT classes
tot_success_array = np.zeros([1,1]) #Initialize array for saving total success for all n buckets
for i in range(2,max_n_div+1): ## Use all the def metric files with different n_buckets
    print("\033[92m ----- Clustering for N Buckets: \033[96m "+str(i)+"x"+str(i)+"\033[92m ----- \033[0m")

    metrics_csv_file = str(i) + "x" + str(i) + "/" + str(i) + "x" + str(i) + ".csv"
    metrics_csv_dir = metrics_directory+metrics_csv_file

    ## Train kmeans with deformation data
    pred_classes_df, GT_class_sizes = train_kmeans(metrics_csv_dir, i)

    ## Get samples of each cluster (png files are saved with assigned class label)
    clusters_files = get_clusters_files(pred_classes_df)

    ## Assign GT labels to the predicted clusters based on the majority class present + Compute success based on this criterion
    #assigned_class_numbers, assigned_class_labels = assign_label_clusters2(pred_classes_df)
    #success_file, tot_success_array = success_ratio(pred_classes_df, i, assigned_class_labels, tot_success_array)

    ## Get prediction success rate selecting GT class label combination that maximises total success
    success_file, tot_success_array, assigned_class_labels, assigned_class_labels_n = success_ratio_combine_GT_classes(pred_classes_df, i, GT_class_sizes, tot_success_array)

    compute_confusion_matrix(assigned_class_labels_n, pred_classes_df, i)
    ## Plot sample images that compose each cluster (RGB and deformation pattern)
    if(save_predicted_clusters):
        print_info(activate_print, "\033[96m Plotting RGB files \033[0m")
        plot_results(clusters_files, i, assigned_class_labels, "")
        print_info(activate_print, "\033[96m Plotting pattern files \033[0m")
        plot_results(clusters_files, i, assigned_class_labels, "_pat")

## Writes a CSV file with the total success for all number of buckets
tot_success_ratios = np.delete(tot_success_array,0,0)
tot_success(tot_success_ratios)




#######  REFS
# https://www.aprendemachinelearning.com/k-means-en-python-paso-a-paso/
