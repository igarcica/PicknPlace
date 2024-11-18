import numpy as np
import pandas as pd
import os
from sklearn.cluster import KMeans
from sklearn.metrics.pairwise import pairwise_distances
import matplotlib.pyplot as plt
from sklearn.metrics import silhouette_score
from sklearn.metrics import davies_bouldin_score


csv_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/3x3/means_data.csv"
write_directory="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/3x3/clusters/"

n_div = 3
n_clusters = 3  # Number of clusters

save_imgs=True


# Function to calculate Frobenius norm between two matrices
def frobenius_norm(mat1, mat2):
    return np.linalg.norm(mat1 - mat2, 'fro')


# Load the CSV file into a DataFrame (Assuming each row is a flattened matrix)
df = pd.read_csv(csv_directory)
filenames = df.iloc[:, 0]
# matrix_data = df.values  # Extract the matrix data (skipping the first row and first column)
matrix_data = df.iloc[:, 1:].values
original_shape = (n_div, n_div)  # Update this to match the shape of your matrices
matrices = [matrix_data[i].reshape(original_shape) for i in range(matrix_data.shape[0])] # Reshape the rows (flattened matrices) back into matrices



# Compute pairwise distance matrix using Frobenius norm
num_matrices = len(matrices)
distance_matrix = np.zeros((num_matrices, num_matrices))

for i in range(num_matrices):
    for j in range(i + 1, num_matrices):
        dist = frobenius_norm(matrices[i], matrices[j])
        distance_matrix[i, j] = dist
        distance_matrix[j, i] = dist

# K-Means clustering on the distance matrix
kmeans = KMeans(n_clusters=n_clusters, random_state=42)
kmeans.fit(distance_matrix)

# Calculate silhouette score (you need to compute the distance matrix)
silhouette_avg = silhouette_score(distance_matrix, kmeans.labels_, metric='precomputed')
print(f"Silhouette Score: {silhouette_avg}")

db_index = davies_bouldin_score(distance_matrix, kmeans.labels_)
print(f"Davies-Bouldin Index: {db_index}")


# Output the cluster labels
cluster_labels = kmeans.labels_
print("Cluster labels:", kmeans.labels_)

# # Visualizing the matrices with their cluster labels
# plt.figure(figsize=(10, 6))
# for idx, matrix in enumerate(matrices):
#     plt.subplot(2, 3, idx + 1)
#     im = plt.imshow(matrix, cmap='viridis', vmin=np.min(distance_matrix), vmax=np.max(distance_matrix))
#     plt.title(f'Cluster {kmeans.labels_[idx]}')
#     plt.colorbar(im)

# Visualizing the distance matrix as a heatmap
plt.figure(figsize=(8, 6))
plt.imshow(distance_matrix, cmap="YlGnBu")
# sns.heatmap(distance_matrix, annot=True, cmap="YlGnBu", fmt=".2f", cbar=True)
plt.title("Pairwise Distance Matrix")
plt.show()



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


        
        
        
        

### TO DO
## Get first row of csv
## GIT COMMIT

