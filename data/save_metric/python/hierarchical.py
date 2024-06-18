# Tratamiento de datos
# ==============================================================================
import numpy as np
import pandas as pd
import csv
import math
from sklearn.datasets import make_blobs

# Graficos
# ==============================================================================
import matplotlib.pyplot as plt
from matplotlib import style
import matplotlib.image as mpimg
style.use('ggplot') or plt.style.use('ggplot')

# Preprocesado y modelado
# ==============================================================================
from sklearn.cluster import AgglomerativeClustering
from scipy.cluster.hierarchy import dendrogram
from sklearn.preprocessing import scale
from sklearn.metrics import silhouette_score

# Configuracion warnings
# ==============================================================================
import warnings
warnings.filterwarnings('ignore')

import sklearn
#print(sklearn.__version__)
from sklearn.datasets import load_iris

##################################################################################################

max_n_div = 10

metrics_directory = "./metrics/not_filling_peso/"
results_directory = "./"#results/filling/"

activate_print=False
save_predicted_clusters = True

##################################################################################################
### EJEMPLOS

def example1():
    print("Executing Example 1...")
    iris = load_iris()
    X = iris.data

    # setting distance_threshold=0 ensures we compute the full tree.
    model = AgglomerativeClustering(distance_threshold=0, n_clusters=None)

    model = model.fit(X)
    plt.title("Hierarchical Clustering Dendrogram")
    # plot the top three levels of the dendrogram
    plot_dendrogram(model, truncate_mode="level", p=3)
    plt.xlabel("Number of points in node (or index of point if no parenthesis).")
    plt.show()

def example2():
    print("Executing Example 2...")

    # Simulacion de datos
    # ==============================================================================
    X, y = make_blobs(
            n_samples    = 200,
            n_features   = 2,
            centers      = 4,
            cluster_std  = 0.60,
            shuffle      = True,
            random_state = 0
           )
    # Escalado de datos
    # ==============================================================================
    X_scaled = scale(X)

    # Modelos
    # ==============================================================================
    modelo_hclust_complete = AgglomerativeClustering(
                                n_clusters         = None,   ## None para utilizar criterio distance_threshold
                                affinity = 'euclidean',     ## metrica utilizada como distancia
                                linkage  = 'complete',      ## Tpo de linkage (que puntos se cojen para mirar distancia?) - minimizes the maximum distance between observations of pairs of clusters.
                                distance_threshold = 0.0     ## 0 para crecer todo el dendograma
                            )
    modelo_hclust_complete.fit(X=X_scaled)

    modelo_hclust_average = AgglomerativeClustering(
                                affinity = 'euclidean',
                                linkage  = 'average',       ## minimizes the average of the distances between all observations of pairs of clusters.
                                distance_threshold = 0,
                                n_clusters         = None
                            )
    modelo_hclust_average.fit(X=X_scaled)

    modelo_hclust_ward = AgglomerativeClustering(
                                affinity = 'euclidean',
                                linkage  = 'ward',          ## minimizes the sum of squared differences within all clusters. (igual kmeans)
                                distance_threshold = 0,
                                n_clusters         = None
                         )
    modelo_hclust_ward.fit(X=X_scaled)



    # Dendrogramas
    # ==============================================================================
    fig, axs = plt.subplots(3, 1, figsize=(8, 8))
    plot_dendrogram(modelo_hclust_average, color_threshold=0, ax=axs[0])
    axs[0].set_title("Distancia euclidea, Linkage average")
    plot_dendrogram(modelo_hclust_complete, color_threshold=0, ax=axs[1])
    axs[1].set_title("Distancia euclidea, Linkage complete")
    plot_dendrogram(modelo_hclust_ward, color_threshold=0, ax=axs[2])
    axs[2].set_title("Distancia euclidea, Linkage ward")
    plt.tight_layout();

    ##Corte
    fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    altura_corte = 6
    plot_dendrogram(modelo_hclust_ward, color_threshold=altura_corte, ax=ax)
    ax.set_title("Distancia euclidea, Linkage ward")
    ax.axhline(y=altura_corte, c = 'black', linestyle='--', label='altura corte')
    ax.legend();
    plt.show()

    # Metodo silhouette para identificar el numero optimo de clusters
    # ==============================================================================
    range_n_clusters = range(2, 15)
    valores_medios_silhouette = []

    for n_clusters in range_n_clusters:
        modelo = AgglomerativeClustering(
                        affinity   = 'euclidean',
                        linkage    = 'ward',
                        n_clusters = n_clusters
                 )

        cluster_labels = modelo.fit_predict(X_scaled)
        silhouette_avg = silhouette_score(X_scaled, cluster_labels)
        valores_medios_silhouette.append(silhouette_avg)

    fig, ax = plt.subplots(1, 1, figsize=(6, 3.84))
    ax.plot(range_n_clusters, valores_medios_silhouette, marker='o')
    ax.set_title("Evolucion de media de los indices silhouette")
    ax.set_xlabel('Numero clusters')
    ax.set_ylabel('Media indices silhouette');
    plt.show()

    # Modelo
    # ==============================================================================
    modelo_hclust_ward = AgglomerativeClustering(
                                affinity = 'euclidean',
                                linkage  = 'ward',
                                n_clusters = 4
                         )
    modelo_hclust_ward.fit(X=X_scaled)

def test_dif_models(X, n_div):
    ## Euclidean distance, diferent linkage
    modelo_hclust_complete = train_models(X, 'euclidean', 'complete', None, 0.0)
    #modelo_hclust_single = train_models(X, 'euclidean', 'single', None, 0.0)
    modelo_hclust_average = train_models(X, 'euclidean', 'average', None, 0.0)     ## minimizes the average of the distances between all observations of pairs of clusters.
    modelo_hclust_ward = train_models(X, 'euclidean', 'ward', None, 0.0)           ## minimizes the sum of squared differences within all clusters. (igual kmeans) - ward minimiza una funcion (suma total de varianza intra-cluster)
    models = [modelo_hclust_complete, modelo_hclust_average, modelo_hclust_ward]
    distances = ["euclidean", "euclidean", "euclidean"]
    linkages = ["complete", "average", "ward"]
    thresholds = [0.25, 0.17, 0.4]
    thresholds = [0,0,0]
    show_model_dendograms(n_div, models, distances, linkages, thresholds)


    ## Complete linkage, diferents distances
    modelo_hclust_euclidean = train_models(X, 'euclidean', 'complete', None, 0.0)
    modelo_hclust_manhattan = train_models(X, 'manhattan', 'complete', None, 0.0)
    modelo_hclust_cosine = train_models(X, 'cosine', 'complete', None, 0.0)
    #modelo_hclust_precomp = train_models(X, 'precomputed', 'complete', None, 0.0)
    modelo_hclust_l1 = train_models(X, 'l1', 'complete', None, 0.0)
    modelo_hclust_l2 = train_models(X, 'l2', 'complete', None, 0.0)
    models = [modelo_hclust_euclidean, modelo_hclust_manhattan, modelo_hclust_cosine, modelo_hclust_l1, modelo_hclust_l2]
    distances = ["euclidean", "manhattan", "cosine", "l1", "l2"]
    linkages = ["complete", "complete", "complete", "complete", "complete"]
    thresholds = [0.25, 0.49, 0.17, 0.47, 0.25]
    thresholds = [0,0,0,0,0]
    show_model_dendograms(n_div, models, distances, linkages, thresholds)

    ## Average linkage, diferents distances
    modelo_hclust_euclidean = train_models(X, 'euclidean', 'average', None, 0.0)
    modelo_hclust_manhattan = train_models(X, 'manhattan', 'average', None, 0.0)
    modelo_hclust_cosine = train_models(X, 'cosine', 'average', None, 0.0)
    #modelo_hclust_precomp = train_models(X, 'precomputed', 'average', None, 0.0)
    modelo_hclust_l1 = train_models(X, 'l1', 'average', None, 0.0)
    modelo_hclust_l2 = train_models(X, 'l2', 'average', None, 0.0)
    models = [modelo_hclust_euclidean, modelo_hclust_manhattan, modelo_hclust_cosine, modelo_hclust_l1, modelo_hclust_l2]
    distances = ["euclidean", "manhattan", "cosine", "l1", "l2"]
    linkages = ["average", "average", "average", "average", "average"]
    thresholds = [0.25, 0.49, 0.17, 0.47, 0.25]
    thresholds = [0,0,0,0,0]
    show_model_dendograms(n_div, models, distances, linkages, thresholds)

    ## Optimal nnumber of clusters according to silhouette
    # optimal_n_cluster_silhouette(X)

##################################################################################################
## UTIL FUNCTIONS

def print_info(activate, arg1, arg2=""):
    if(activate):
        #print(arg1)
        #print(arg2)
        print(str(arg1) + str(arg2))

def plot_dendrogram(model, **kwargs):
    '''
    Esta funcion extrae la informacion de un modelo AgglomerativeClustering
    y representa su dendograma con la funcion dendogram de scipy.cluster.hierarchy
    '''

    counts = np.zeros(model.children_.shape[0])
    n_samples = len(model.labels_)
    for i, merge in enumerate(model.children_):
        current_count = 0
        for child_idx in merge:
            if child_idx < n_samples:
                current_count += 1  # leaf node
            else:
                current_count += counts[child_idx - n_samples]
        counts[i] = current_count

    #linkage_matrix = np.column_stack([model.children_, model.n_clusters_, counts]).astype(float)
    linkage_matrix = np.column_stack([model.children_, model.distances_, counts]).astype(float)

    # Plot
    dendrogram(linkage_matrix, **kwargs)

def show_model_dendograms(n_div, models, distances_type, linkages_type, thresholds=0):
    print("Plotting dendograms")
    fig, axs = plt.subplots(len(models), 1, figsize=(8, 8))
    for i in range(0,len(models)):
        plot_dendrogram(models[i], color_threshold=thresholds[i], ax=axs[i])
        title="Distancia "+distances_type[i]+", Linkage "+linkages_type[i]
        axs[i].set_title(title)
        if(thresholds[i]!=0):
            axs[i].axhline(y=thresholds[i], c = 'black', linestyle='--', label='altura corte')
            axs[i].legend();

    plt.tight_layout();
    # plt.show()
    file_name = "dendogram_"+distances_type[i]+"_"+linkages_type[i]+"_"+str(n_div)+".png"
    print_info(True, file_name)
    plt.savefig(file_name)
    plt.close()


def show_dendogram(model, distance_type, linkage_type, threshold=0):
    ## Plot only one dendogram
    fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    plot_dendrogram(model, color_threshold=threshold, ax=ax)
    title="Distancia "+distance_type+", Linkage "+linkage_type
    ax.set_title(title)
    ax.axhline(y=threshold, c = 'black', linestyle='--', label='altura corte')
    ax.legend();
    plt.show()

def optimal_n_cluster_silhouette(data, n_div):
    # Metodo silhouette para identificar el numero optimo de clusters
    range_n_clusters = range(2, 15)
    valores_medios_silhouette = []

    for n_clusters in range_n_clusters:
        modelo = AgglomerativeClustering(
                        affinity   = 'euclidean',
                        linkage    = 'ward',
                        n_clusters = n_clusters
                 )

        cluster_labels = modelo.fit_predict(data)
        silhouette_avg = silhouette_score(data, cluster_labels)
        valores_medios_silhouette.append(silhouette_avg)

    fig, ax = plt.subplots(1, 1, figsize=(6, 3.84))
    ax.plot(range_n_clusters, valores_medios_silhouette, marker='o')
    ax.set_title("Evolucion de media de los indices silhouette")
    ax.set_xlabel('Numero clusters')
    ax.set_ylabel('Media indices silhouette');

    # plt.show()
    file_name = "silhouette_"+str(n_div)+".png"
    print_info(True, file_name)
    plt.savefig(file_name)
    plt.close()

## Plots the images that compose each cluster
def plot_results(clusters_files, n_div, pred_class_label, rgb_pat):
    print("\033[96m Plotting clusters files... \033[0m")

    activate_print=True
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
            #file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            file_name = "./"+pred_class_label[i] +"_" + str(rgb_pat) + ".png"
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
            fig, axs = plt.subplots(1,size1, figsize=(16, 9)) #(1,size1)
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
            #file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            file_name = "./"+pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()
        else:
            print_info(activate_print, "more than 2 samples in cluster")
            fig, axs = plt.subplots(size2,size1, figsize=(16, 9))
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
            #file_name = results_directory + str(n_div) + "x" + str(n_div) + "/cluster_" + pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            file_name = "./"+pred_class_label[i] +"_" + str(rgb_pat) + ".png"
            print_info(activate_print, file_name)
            plt.savefig(file_name)
            plt.close()

##################################################################################################

## CLUSTERING FUNCTIONS
def tot_success(success_ratios):
    print("\033[96m Total Success \033[0m")

    success_file = "./total_success.csv"
    my_file = open(success_file, "w")
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

def success_ratio_combine_GT_classes(pred_classes_df, n_div, GT_class_sizes, tot_success_array):
    print("\033[96m Computing success ratio \033[0m")


    GT_name_classes = ['A','B','C','D']
    ## All possible combinations of classes (for 4 GT classes)
    comb_classes = [["A","B","C","D"],["B","A","C","D"],["C","B","A","D"],["D","B","C","A"],["A","B","D","C"],["A","D","C","B"],["A","C","B","D"]]

    ## Convert dataframe to array
    print_info(activate_print, pred_classes_df)
    pred_classes_array = pred_classes_df.to_numpy()

    prev_total_success = 0
    combinations_success_matrix = []

    ## Create list of samples changing class number to the assigned class name
    for i in range(0,len(comb_classes)):
        pred_classes_name = np.empty([1,3])
        current_success_ratios = []
        for row in pred_classes_array:
            new_row = [row[0], GT_name_classes[int(row[1])], comb_classes[i][int(row[2])]] ##file_name, GT classes name, pred classes name
            pred_classes_name = np.vstack([pred_classes_name, new_row])
        pred_classes_name = np.delete(pred_classes_name,0,0) ## Delete first row (initialization)
        print_info(activate_print, pred_classes_name)

        ## Get success rates of each cluster comparing assigned label with GT label
        #success_file = results_directory + str(n_div) + "x" + str(n_div) + "/success_" + str(n_div) + "x" + str(n_div) + "_"+str(i)+".csv"
        success_file = "./test_"+str(i)+".csv"
        my_file = open(success_file, "w")
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

        total_success = (float(sum)/float(90))*100
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

    return success_file, tot_success_array, comb_classes[max_success_combination]

def get_clusters_files(pred_classes):
    print("\033[96m Getting clusters samples \033[0m")

    possible_classes_n = [0,1,2,3]
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

def train_models(data, distance_type, linkage_type, n_clusters, dist_thr):
    model = AgglomerativeClustering(
                        n_clusters         = n_clusters,   ## None para utilizar criterio distance_threshold
                        affinity = distance_type,     ## metrica utilizada como distancia
                        linkage  = linkage_type,      ## Tpo de linkage (que puntos se cojen para mirar distancia?) - minimizes the maximum distance between observations of pairs of clusters.
                        distance_threshold = dist_thr     ## 0 para crecer todo el dendograma
                        )
    model.fit(X=data)

    return model

# def success_ratio():

##################################################################################################

#example2()
tot_success_array = np.zeros([1,1]) #Initialize array for saving total success for all n buckets
for i in range(8,8+1): ## Use all the def metric files with different n_buckets
    print("\033[92m ----- Clustering for N Buckets: \033[96m "+str(i)+"x"+str(i)+"\033[92m ----- \033[0m")

    ## Reading data
    metrics_csv_file = str(i) + "x" + str(i) + "/" + str(i) + "x" + str(i) + ".csv"
    metrics_csv_dir = metrics_directory+metrics_csv_file

    data_df = pd.read_csv(metrics_csv_dir)

    X = np.genfromtxt(metrics_csv_dir, delimiter=',')
    X = X[1:100,:] #Remove header
    #sX = X[X[:,2].argsort()]    ## Sort samples based on class
    X = X[:,3:9] ## Slice feature columns


## TEST MODELS
    # optimal_n_cluster_silhouette(X,i)
    # test_dif_models(X,i)
    # modelo_hclust_ward = train_models(X, 'euclidean', 'ward', None, 0.0)
    # show_dendogram(modelo_hclust_ward, "euclidean", "ward",0.56)


# ## TRAIN MODEL
    n_classes = 4
    ## Train with optmal number of clusters according to silhouette
    modelo_hclust_complete_opt = train_models(X, 'euclidean', 'complete', 4, None)
    modelo_hclust_average_opt = train_models(X, 'euclidean', 'average', 3, None)
    modelo_hclust_ward_opt = train_models(X, 'euclidean', 'ward', 4, None)

    modelo_hclust_euclidean = train_models(X, 'euclidean', 'complete', 3, None)
    modelo_hclust_manhattan = train_models(X, 'manhattan', 'complete', 4, None)
    modelo_hclust_cosine = train_models(X, 'cosine', 'complete', 3, None)
    modelo_hclust_l1 = train_models(X, 'l1', 'complete', 3, None)
    modelo_hclust_l2 = train_models(X, 'l2', 'complete', 3, None)

    modelo_hclust_av_euclidean = train_models(X, 'euclidean', 'average', 3, None)
    modelo_hclust_av_manhattan = train_models(X, 'manhattan', 'average', 3, None)
    modelo_hclust_av_cosine = train_models(X, 'cosine', 'average', 3, None)
    modelo_hclust_av_l1 = train_models(X, 'l1', 'average', 3, None)
    modelo_hclust_av_l2 = train_models(X, 'l2', 'average', 3, None)
    # print(modelo_hclust_complete_opt.labels_)

## COMPUTE SUCCESS
    GT_class_sizes = data_df.groupby('Class_GT').size().values
    pred_classes_df =  pd.DataFrame()
    pred_classes_df['File']=data_df['File'].values
    pred_classes_df['Class_GT_n']=data_df['Class_GT_n'].values
    pred_classes_df['label']=modelo_hclust_manhattan.labels_

    ## Get samples of each cluster (png files are saved with assigned class label)
    clusters_files = get_clusters_files(pred_classes_df)

    success_file, tot_success_array, assigned_class_labels = success_ratio_combine_GT_classes(pred_classes_df, i, GT_class_sizes, tot_success_array)

    # Plot sample images that compose each cluster (RGB and deformation pattern)
    if(save_predicted_clusters):
        print_info(activate_print, "\033[96m Plotting RGB files \033[0m")
        plot_results(clusters_files, i, assigned_class_labels, "")
        print_info(activate_print, "\033[96m Plotting pattern files \033[0m")
        plot_results(clusters_files, i, assigned_class_labels, "_pat")

# ## Writes a CSV file with the total success for all number of buckets
# tot_success_ratios = np.delete(tot_success_array,0,0)
# tot_success(tot_success_ratios)

##################################################################################################

##### REFS
## https://www.cienciadedatos.net/documentos/py20-clustering-con-python.html
## https://scikit-learn.org/stable/auto_examples/cluster/plot_agglomerative_dendrogram.html#sphx-glr-auto-examples-cluster-plot-agglomerative-dendrogram-py
