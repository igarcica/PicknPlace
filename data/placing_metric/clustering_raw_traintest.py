### Same code as classifier.py but kmeans model (and the validation functions) is obtained with raw data instead of with the pairwise distance matrix

import numpy as np
import pandas as pd
import os
from sklearn.cluster import KMeans
from sklearn.metrics.pairwise import pairwise_distances
import matplotlib.pyplot as plt
from sklearn.metrics import silhouette_score
from sklearn.metrics import davies_bouldin_score
# from scipy.spatial.distance import pdist, squareform
from scipy.spatial.distance import cdist
import plotly.express as px

from sklearn.metrics import confusion_matrix, accuracy_score
import seaborn as sns #To plot confusion matrix with values


directory="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/train_test/"
# csv_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/3x3/means_data.csv"
# write_directory="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/3x3/clusters/"

n_div = 7
n_clusters = 3  # Number of clusters
print("GRID: ", n_div, " / Clusters: ", n_clusters)

save_imgs = False
activate_print = False

write_directory = directory + str(n_div) + "x" + str(n_div) + "/clusters_raw/"
train_directory = directory + str(n_div) + "x" + str(n_div) + "/metric/train_metrics.csv"
test_directory = directory + str(n_div) + "x" + str(n_div) + "/metric/test_metrics.csv"
human_gt_directory = directory + "human_GT_labels.csv"
human_test_gt_directory = directory + "human_test_GT_labels.csv"
all_labels_directory = directory + str(n_div) + "x" + str(n_div) + "/clusters_raw/all_cluster_labels.csv"
test_gt_directory = directory + str(n_div) + "x" + str(n_div) + "/clusters_raw/test_GT_labels.csv"



##################################################################################################
## CLASSIFICATION

# Function to calculate Frobenius norm between two matrices
def frobenius_norm(mat1, mat2):
    return np.linalg.norm(mat1 - mat2, 'fro')

def create_semantic_classes():
    # Class A
    A_matrix = np.zeros(n_div*n_div)
    A_matrix = A_matrix.reshape(-3, 3)
    # print(A_matrix)
    plot_metrics(A_matrix)

    # Class B
    B_matrix = np.array([[-1, -1, -1], [-1, -1, -1], [0, 0, 0]])
    # print(B_matrix)
    plot_metrics(B_matrix)

    # Class C
    C_matrix = np.array([[-1, 0, -1], [-1, 0, -1], [-1, 0, -1]])
    # print(C_matrix)
    plot_metrics(C_matrix)

## Plot metrics in colored grid
def plot_metrics(metrics):
    fig = px.imshow(metrics, text_auto=True, labels=dict(x='x', y='y'))
    fig.update_coloraxes(cmin=-1, cmax=0)
    fig.show()
    # if not all_files:
    #     fig.show()
    # if save_csv:
    #     filename = write_dir + filename + ".jpg"
    #     fig.write_image(filename)

def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6="", arg7=""):
    if(activate):
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6) + str(arg7))

##################################################################################################
## VALIDATION

## Evaluation metrics
def validation_metrics(X, kmeans):
    print("------------------------------------------------")
    # # Calculate silhouette score (you need to compute the distance matrix)
    # silhouette_avg = silhouette_score(distance_matrix, kmeans.labels_, metric='precomputed')
    # print(f"Silhouette Score: {silhouette_avg}")

    # db_index = davies_bouldin_score(distance_matrix, kmeans.labels_)
    # print(f"Davies-Bouldin Index: {db_index}")

    # Calculate silhouette score (you need to compute the distance matrix)
    silhouette_avg = silhouette_score(X, kmeans.labels_)
    print(f"Silhouette Score: {silhouette_avg}")

    db_index = davies_bouldin_score(X, kmeans.labels_)
    print(f"Davies-Bouldin Index: {db_index}")


def calculate_wcss(X, kmeans):
    """
    Calculate the Within-cluster sum of squares (WCSS) for k-means clustering.

    Parameters:
    - X: array-like, shape (n_samples, n_features)
      The data points used in clustering.
    - kmeans: KMeans object
      A trained KMeans model.

    Returns:
    - wcss: The Within-cluster sum of squares (WCSS) value (float).
    """
    
    centroids = kmeans.cluster_centers_ # Get the cluster centers (centroids)

    wcss = 0

    # Iterate over each cluster
    for i in range(kmeans.n_clusters):
        
        cluster_points = X[kmeans.labels_ == i] # Get the points assigned to cluster i
        distances = np.linalg.norm(cluster_points - centroids[i], axis=1) # Calculate the squared distances to the centroid of cluster i
        wcss += np.sum(distances ** 2) # Sum of squared distances for this cluster

    return wcss


def calculate_wcss_with_distances(D, labels, n_clusters):
    """
    Calculate the Within-cluster sum of squares (WCSS) based on a precomputed distance matrix.

    Parameters:
    - D: array-like, shape (n_samples, n_samples)
      The pairwise distance matrix between data points.
    - labels: array-like, shape (n_samples,)
      The labels assigned by the KMeans algorithm to each data point.
    - n_clusters: int
      The number of clusters (same as the number of centroids).

    Returns:
    - wcss: float
      The Within-cluster sum of squares (WCSS) value.
    """
    # Initialize WCSS
    wcss = 0

    # Iterate over each cluster
    for i in range(n_clusters):
        # Get indices of data points belonging to cluster i
        cluster_points = np.where(labels == i)[0]
        
        # If the cluster has more than one point, calculate the WCSS for it
        if len(cluster_points) > 1:
            # Submatrix of pairwise distances for the cluster points
            D_cluster = D[cluster_points][:, cluster_points]
            
            # Calculate the "centroid" of the cluster in terms of pairwise distances
            # The centroid is effectively the mean distance to all points within the cluster
            mean_distance = np.mean(D_cluster)
            
            # Sum of squared distances within the cluster
            # In distance matrix space, we are summing squared distances to the mean
            cluster_wcss = np.sum((D_cluster - mean_distance) ** 2)
            
            # Add to the total WCSS
            wcss += cluster_wcss

    return wcss


def compute_dunn_index(distance_matrix, cluster_labels):
    """
    Compute the Dunn Index for clustering validation.
    
    Parameters:
    - distance_matrix (ndarray): A square matrix of pairwise distances.
    - cluster_labels (ndarray): An array of cluster labels for each data point.
    
    Returns:
    - float: The Dunn Index.
    """
    unique_clusters = np.unique(cluster_labels)
    num_clusters = len(unique_clusters)
    
    if num_clusters < 2:
        raise ValueError("Dunn Index requires at least two clusters.")
    
    # Intra-cluster distances (delta_c): maximum distance within a cluster
    intra_cluster_distances = []
    for cluster in unique_clusters:
        indices = np.where(cluster_labels == cluster)[0]
        if len(indices) > 1:
            cluster_distances = distance_matrix[np.ix_(indices, indices)]
            intra_cluster_distances.append(np.max(cluster_distances))
        else:
            intra_cluster_distances.append(0)
    
    max_intra_cluster_distance = max(intra_cluster_distances)
    
    # Inter-cluster distances (delta): minimum distance between clusters
    inter_cluster_distances = []
    for i, cluster_i in enumerate(unique_clusters):
        for j, cluster_j in enumerate(unique_clusters):
            if i < j:  # Avoid redundant comparisons
                indices_i = np.where(cluster_labels == cluster_i)[0]
                indices_j = np.where(cluster_labels == cluster_j)[0]
                distances = distance_matrix[np.ix_(indices_i, indices_j)]
                inter_cluster_distances.append(np.min(distances))
    
    min_inter_cluster_distance = min(inter_cluster_distances)
    
    # Dunn Index: Ratio of minimum inter-cluster distance to maximum intra-cluster distance
    dunn_index = min_inter_cluster_distance / max_intra_cluster_distance
    return dunn_index

def dunn_index(X, labels):
    unique_labels = np.unique(labels)
    n_clusters = len(unique_labels)
    intra_distances = []
    inter_distances = []

    # Compute the minimum inter-cluster distance
    for i in range(n_clusters):
        for j in range(i+1, n_clusters):
            inter_distances.append(np.min(cdist(X[labels == unique_labels[i]], X[labels == unique_labels[j]])))
    
    # Compute the maximum intra-cluster distance
    for i in range(n_clusters):
        intra_distances.append(np.max(cdist(X[labels == unique_labels[i]], [X[labels == unique_labels[i]].mean(axis=0)])))
    
    # Dunn Index: min inter-cluster distance / max intra-cluster distance
    return np.min(inter_distances) / np.max(intra_distances)

## Alignment of cluster labels to GT - majority voting
def align_clusters_to_labels(cluster_labels, gt_labels, sample_names, name):
    """
    Aligns cluster labels to ground truth labels using majority voting.
    """
    mapping = {}
    for cluster in np.unique(cluster_labels):
        # Get the ground truth labels of samples in the current cluster
        mask = (cluster_labels == cluster)
        true_labels = gt_labels[mask]
        # Find the most frequent label (majority voting)
        if len(true_labels) > 0:
            most_common_label = np.bincount(true_labels).argmax()
            mapping[cluster] = most_common_label

    # Reassign cluster labels based on the mapping
    aligned_labels = np.array([mapping[cluster] for cluster in cluster_labels])

    # Identify mismatched samples and save in CSV
    mismatched_mask = aligned_labels != gt_labels
    # print("MISMATCHED NAMES: ", sample_names[mismatched_mask])
    mismatched_samples = pd.DataFrame({
        'SampleName': sample_names[mismatched_mask],
        'GTLabel': gt_labels[mismatched_mask],
        'ClusterLabel': aligned_labels[mismatched_mask]
    })
    # mismatched_samples_dir = write_directory + name + "_mismatched.csv"
    # mismatched_samples.to_csv(mismatched_samples_dir, index=False)
    
    return aligned_labels, mapping, mismatched_samples

def evaluate_clustering(cluster_labels, gt_labels, sample_names, name):
    """
    Evaluates clustering performance by aligning cluster labels with ground truth
    and calculating accuracy.
    """
    # Align clusters to ground truth labels
    aligned_labels, mapping, mismatched_samples = align_clusters_to_labels(cluster_labels, gt_labels, sample_names, name)

    # Compute accuracy
    accuracy = accuracy_score(gt_labels, aligned_labels)

    # Confusion matrix (optional for more detailed evaluation)
    confusion = confusion_matrix(gt_labels, aligned_labels)

    print("Cluster to Ground Truth Mapping:", mapping)
    print("Aligned Labels:", aligned_labels)
    print("Accuracy:", accuracy)
    print("Confusion Matrix: \n", confusion)
    print("Confusion Matrix: \n", mismatched_samples)

    ## Save confusion matrix
    sns.heatmap(confusion, annot=True, fmt="d", cmap="YlGnBu")
    # sns.heatmap(confusion, annot=True, cmap="YlGnBu", fmt=".2f", cbar=True)
    plt.title("Confusion Matrix")
    confusion_matrix_filename = write_directory + name + "_confusion_matrix.png"
    # plt.savefig(confusion_matrix_filename)
    plt.show()

    return accuracy, confusion, aligned_labels, mapping, mismatched_samples



##################################################################################################
## MAIN

# Load the CSV file into a DataFrame (Assuming each row is a flattened matrix)
train_df = pd.read_csv(train_directory)
train_filenames = train_df.iloc[:, 0]
# matrix_data = df.values  # Extract the matrix data (skipping the first row and first column)
train_matrix_data = train_df.iloc[:, 1:].values
original_shape = (n_div, n_div)  # Update this to match the shape of your matrices
train_matrices = [train_matrix_data[i].reshape(original_shape) for i in range(train_matrix_data.shape[0])] # Reshape the rows (flattened matrices) back into matrices

# print(train_matrix_data)
# print(train_matrices[0])


# K-Means clustering on the distance matrix
kmeans = KMeans(n_clusters=n_clusters, random_state=42)
# kmeans.fit(distance_matrix)
kmeans.fit(train_matrix_data)
# Output the cluster labels
cluster_labels = kmeans.labels_
print_info(activate_print, "Cluster labels:", kmeans.labels_)
# print("FIT PREDICT: ", kmeans.fit_predict(train_matrix_data))
# Access the centroids
centroids = kmeans.cluster_centers_
print_info(activate_print, "Cluster centroids: ", centroids)
# ## Save cluster labels of train data in CSV
# pred_train_labels = pd.DataFrame({
#     'SampleName': train_filenames,
#     'ClusterLabel': cluster_labels
# })

# create_semantic_classes()


## Save metric images in a folder correpsonding to the cluster
if save_imgs:
    # Create a folder for saving clusterized data
    os.makedirs(write_directory, exist_ok=True)

    # Loop over each cluster
    for cluster_id in np.unique(cluster_labels):
        # Create a folder for the current cluster
        cluster_folder = os.path.join(write_directory, f"cluster_{cluster_id}")
        os.makedirs(cluster_folder, exist_ok=True)
        
        # Get the indices of matrices in this cluster
        cluster_indices = np.where(cluster_labels == cluster_id)[0]
        
        # Loop over each matrix in the cluster and save it as an image
        for idx in cluster_indices:
            matrix = train_matrices[idx]  # Get the matrix for this index
            filename = train_filenames[idx]
            
            # Create a heatmap for the matrix
            plt.figure(figsize=(6, 6))
            im = plt.imshow(matrix, cmap='plasma', interpolation='nearest', vmin=0, vmax=-1)
            plt.colorbar(im)  # Add a colorbar to the heatmap
            plt.title(f"Matrix {filename} in Cluster {cluster_id}")
            
            output_file = os.path.join(cluster_folder, f"{filename}.png")  # Save as .png with original filename
            plt.savefig(output_file)
            plt.close()  # Close the figure to avoid memory issues

            # print(f"Saved matrix {idx} in cluster {cluster_id} to {output_file}")

    print("All matrices saved!")


# Validation
validation_metrics(train_matrix_data, kmeans)
# wcss = calculate_wcss(matrix_data, kmeans)
# wcss = calculate_wcss_with_distances(matrix_data, kmeans.labels_, n_clusters)
wcss = calculate_wcss(train_matrix_data, kmeans)
print("WCSS: ", wcss)
            
# dunn_index = compute_dunn_index(matrix_data, kmeans.labels_)
dunn_index = dunn_index(train_matrix_data, kmeans.labels_)
print("Dunn Index:", dunn_index)





# ##### TEST DATA
# Load the CSV file into a DataFrame (Assuming each row is a flattened matrix)
test_df = pd.read_csv(test_directory)
test_filenames = test_df.iloc[:, 0]
test_matrix_data = test_df.iloc[:, 1:].values
original_shape = (n_div, n_div)  # Update this to match the shape of your matrices
test_matrices = [test_matrix_data[i].reshape(original_shape) for i in range(test_matrix_data.shape[0])] # Reshape the rows (flattened matrices) back into matrices

## Predict labels for the test set
test_labels = predicted_labels = kmeans.predict(test_matrix_data) 
## Save cluster labels of test data in CSV
pred_test_labels = pd.DataFrame({
    'SampleName': test_filenames,
    'ClusterLabel': test_labels
})
# cluster_labels_dir = write_directory + "pred_test_labels.csv"
# pred_test_labels.to_csv(cluster_labels_dir, index=False)




# #### EVALUATE CLUSTERING 
# ## Get TEST GT labels
# test_gt_df = pd.read_csv(test_gt_directory)
# test_gt_filenames = test_gt_df.iloc[:, 0]
# test_gt_labels = test_gt_df.iloc[:, 1:].values
# test_gt_labels = test_gt_labels.flatten() # Transform to a flat array
# ## Get HUMAN TEST GT labels
# human_test_gt_df = pd.read_csv(human_test_gt_directory)
# human_test_gt_filenames = human_test_gt_df.iloc[:, 0]
# human_test_gt_labels = human_test_gt_df.iloc[:, 1:].values
# human_test_gt_labels = human_test_gt_labels.flatten() # Transform to a flat array
# print("------------------------------------------------")
# print("PREDICTED TEST LABELS: ", test_labels)
# print("TEST GT LABELS: ", test_gt_labels)
# print("HUMAN GT LABELS: ", human_test_gt_labels)
# # print("------------------------------------------------")
# # print("PREDICTED TEST LABELS: ", test_filenames)
# # print("TEST GT LABELS: ", test_gt_filenames)
# # print("HUMAN GT LABELS: ", human_gt_filenames)

# ## Alignment of cluster labels to TEST GT + accuracy evaluation
# print("------------------------------------------------")
# print("\033[94m ALL DATA MODEL CLUSTER GT ACCURACY \033[0m")
# # img_dir = write_directory + "all_data_GT.png"
# name = "all_data_GT"
# accuracy, confusion, aligned_labels, mapping, mismatched_samples = evaluate_clustering(test_labels, test_gt_labels, test_gt_filenames, name)
# print(aligned_labels)

# ## Alignment of cluster labels to HUMAN GT + accuracy evaluation
# print("------------------------------------------------")
# print("\033[94m HUMAN GT ACCURACY \033[0m")
# # img_dir = write_directory + "confusion_matrix_human_GT.png"
# name = "human_GT"
# accuracy, confusion, aligned_labels, mapping, mismatched_samples = evaluate_clustering(test_labels, human_test_gt_labels, human_test_gt_filenames, name)

## 
print("------------------------------------------------")
print("\033[94m ALL DATA CLUSTER LABELS with HUMAN GT ACCURACY \033[0m")
## Get all data cluster labels
all_labels_df = pd.read_csv(all_labels_directory)
all_labels_filenames = all_labels_df.iloc[:, 0]
all_labels = all_labels_df.iloc[:, 1:].values
all_labels = all_labels.flatten() # Transform to a flat array
## Get all human gt labels
human_gt_df = pd.read_csv(human_gt_directory)
human_gt_filenames = human_gt_df.iloc[:, 0]
human_gt_labels = human_gt_df.iloc[:, 1:].values
human_gt_labels = human_gt_labels.flatten() # Transform to a flat array
name = "all_labels_human_GT"
accuracy, confusion, aligned_labels, mapping, mismatched_samples = evaluate_clustering(all_labels, human_gt_labels, human_gt_filenames, name)


print("------------------------------------------------")
print("\033[94m ALL DATA PREDICTIONS WITH TRAIN DATA MODEL with HUMAN GT ACCURACY \033[0m")
## Save cluster labels of all data with model trained only with train data in CSV
all_pred_filenames = np.concatenate((train_filenames, test_filenames))
all_pred_labels = np.concatenate((cluster_labels, test_labels))
print("hola ", all_pred_filenames)
print("hola2 ", all_pred_labels)
pred_all_labels = pd.DataFrame({
    'SampleName': all_pred_filenames,
    'ClusterLabel': all_pred_labels
})
## The files will be saved in a different order
all_predict_labels_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/statistical_analysis/pred_all_labels.csv"
pred_all_labels.to_csv(all_predict_labels_dir, index=False)
## Upload the previous file but with the files ordered (ordered manually)
all_pred_labels_directory = directory + str(n_div) + "x" + str(n_div) + "/clusters_raw/pred_all_labels.csv"
all_pred_labels_df = pd.read_csv(all_pred_labels_directory)
all_pred_labels_filenames = all_pred_labels_df.iloc[:, 0]
all_pred_labels = all_pred_labels_df.iloc[:, 1:].values
all_pred_labels = all_pred_labels.flatten() # Transform to a flat array
name = "pred_all_data2"
accuracy, confusion, aligned_labels, mapping, mismatched_samples = evaluate_clustering(all_pred_labels, human_gt_labels, human_gt_filenames, name)


### TO DO


