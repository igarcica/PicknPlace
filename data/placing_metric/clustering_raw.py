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

directory="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/train_test/"
# csv_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/3x3/means_data.csv"
# write_directory="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/3x3/clusters/"

n_div = 2 # Number of grid divisions
# n_clusters = 2  # Number of clusters
activate_print = False
print("\033[92m ----- Grid division: \033[96m "+str(n_div)+"\033[92m ----- \033[0m")


save_imgs = False

csv_directory = directory + str(n_div) + "x" + str(n_div) + "/metric/all_metrics.csv"
# write_directory = directory + str(n_div) + "x" + str(n_div) + "/clusters_raw/" + str(n_clusters) + "clusters/"

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

def validation_metrics(X, kmeans):
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

# Function to compute Dunn Index
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

##################################################################################################
## MAIN

## SHOULD BE MOVED TO grasping_grid_metric.py
def save_train_test_csv(csv_file_dir):
    print("\033[94m Saving train and test CSV files \033[0m")
    df = pd.read_csv(csv_file_dir)
    test_obj = ["check", "linenap"] # Filter rows containing any of the tokens
    test_df = df[df['Filename'].str.contains('|'.join(test_obj))].sort_values('Filename')  # Contains test_obj names
    train_df = df[~df['Filename'].str.contains('|'.join(test_obj))].sort_values('Filename')  # Does not contain tst_obj names
    write_dir = directory + str(n_div) + "x" + str(n_div) + "/metric/" 
    test_cluster_labels_dir = write_dir + "test_metrics.csv" ## CSV file to save test metrics
    test_df.to_csv(test_cluster_labels_dir, index=False)
    train_cluster_labels_dir = write_dir + "train_metrics.csv"
    train_df.to_csv(train_cluster_labels_dir, index=False)

save_train_test_csv(csv_directory)

for n_clusters in range(2,8): ## Clusterize for all number of clusters (from 2 to 7)
    print("\033[92m ----- Clustering for N Clusters: \033[96m "+str(n_clusters)+"\033[92m ----- \033[0m")

    write_directory = directory + str(n_div) + "x" + str(n_div) + "/clusters_raw/" + str(n_clusters) + "clusters/"

    # Load the CSV file into a DataFrame (Assuming each row is a flattened matrix)
    df = pd.read_csv(csv_directory)
    filenames = df.iloc[:, 0]
    # matrix_data = df.values  # Extract the matrix data (skipping the first row and first column)
    matrix_data = df.iloc[:, 1:].values
    original_shape = (n_div, n_div)  # Update this to match the shape of your matrices
    matrices = [matrix_data[i].reshape(original_shape) for i in range(matrix_data.shape[0])] # Reshape the rows (flattened matrices) back into matrices

    # print(matrices[0])

    # Compute pairwise distance matrix using Frobenius norm - Measures the similarity between each data
    num_matrices = len(matrices)
    distance_matrix = np.zeros((num_matrices, num_matrices))

    for i in range(num_matrices):
        for j in range(i + 1, num_matrices):
            dist = frobenius_norm(matrices[i], matrices[j])
            distance_matrix[i, j] = dist
            distance_matrix[j, i] = dist
    # # Visualizing the distance matrix as a heatmap
    # plt.figure(figsize=(8, 6))
    # plt.imshow(distance_matrix, cmap="YlGnBu")
    # # sns.heatmap(distance_matrix, annot=True, cmap="YlGnBu", fmt=".2f", cbar=True)
    # plt.title("Pairwise Distance Matrix")
    # plt.show()

    # K-Means clustering on the distance matrix
    kmeans = KMeans(n_clusters=n_clusters, random_state=42)
    # kmeans.fit(distance_matrix)
    kmeans.fit(matrix_data)
    # Output the cluster labels
    cluster_labels = kmeans.labels_
    print_info(activate_print, "Cluster labels:", kmeans.labels_)
    # Access the centroids
    centroids = kmeans.cluster_centers_
    print_info(activate_print, "Cluster centroids: ", centroids)

    ## Save cluster labels in CSV
    cluster_labels_df = pd.DataFrame({
        'SampleName': filenames,
        'ClusterLabel': cluster_labels
    })
    cluster_labels_dir = write_directory + "complete_model_all_data_labels.csv"
    cluster_labels_df.to_csv(cluster_labels_dir, index=False)
    ## Seprate into CSV with train and test data
    test_obj = ["check", "linenap"] # Filter rows containing any of the tokens
    test_df = cluster_labels_df[cluster_labels_df['SampleName'].str.contains('|'.join(test_obj))].sort_values('SampleName')  # Contains test_obj names
    train_df = cluster_labels_df[~cluster_labels_df['SampleName'].str.contains('|'.join(test_obj))].sort_values('SampleName')  # Does not contain tst_obj names
    test_cluster_labels_dir = write_directory + "complete_model_test_data_labels.csv"
    test_df.to_csv(test_cluster_labels_dir, index=False)
    train_cluster_labels_dir = write_directory + "complete_model_train_data_labels.csv"
    train_df.to_csv(train_cluster_labels_dir, index=False)

    # create_semantic_classes()

    # Validation
    validation_metrics(matrix_data, kmeans)



    # # Visualizing the matrices with their cluster labels
    # plt.figure(figsize=(10, 6))
    # for idx, matrix in enumerate(matrices):
    #     plt.subplot(2, 3, idx + 1)
    #     im = plt.imshow(matrix, cmap='viridis', vmin=np.min(distance_matrix), vmax=np.max(distance_matrix))
    #     plt.title(f'Cluster {kmeans.labels_[idx]}')
    #     plt.colorbar(im)



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
                matrix = matrices[idx]  # Get the matrix for this index
                filename = filenames[idx]
                
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


    # wcss = calculate_wcss(matrix_data, kmeans)
    # wcss = calculate_wcss_with_distances(matrix_data, kmeans.labels_, n_clusters)
    wcss = calculate_wcss(matrix_data, kmeans)
    print("WCSS: ", wcss)
            
    # dunn_index = compute_dunn_index(matrix_data, kmeans.labels_)
    dunn_ind = dunn_index(matrix_data, kmeans.labels_)
    print("Dunn Index:", dunn_ind)

    ### TO DO


