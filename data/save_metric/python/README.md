Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Deformation metric

These scripts are used to compute the deformation metric.

## Getting started

This folder contains the following files:

- data/: Scripts to save data from pick and place executions and compute deformation metric.
    - **save_metric/**: 
        - c/: Contains the code (in C++) to compute and write csv files with the deformation data (using the pcd files with full view and vision node)
            - execution.py: Publishes the content of PCD files as point cloud during 8seg one at a time.
            - /node: ROS node that subscribes to topic with deformation data (computed and published by the vision_pick_place node) and writes CSV files.
        - ptyhon/: Contains the code (in python) to compute and write the csv files with the deformation data (using pcd files with segmented garment).
            - def_metric.py: Computes the deformation metric and saves it in CSV files.
            - clustering.py: Computes the centroid and inter/intra distances of the deformation data from csv files.
            - kmeans.py: Clusterises pick files based on deformation metrics and computes success rate comparing it to the ground truth.
            - plot_results.py: Prints the deformation metrics in the corresponding files for visual information.


### How to compute the deformation metric and clusterise

Go to the /data folder, to compute the deformation metric, set the path with the PCD files of the experiments and the path where you want to save the results in the "def_metrics.py" script. Also select the number of desired buckets.

``python def_metric.py``

Running the script will output a CSV file on the introduce path with a list of the experiments file names and the corresponding deformation metric given the number of buckets.


To visualise these metrics on top of the images for visual information, run the following script indicating the paths with the RGB images of the experiments and the folder where to save the resulting images.

``python plot_results.py``


To clusterise and compute the success rate based on a given ground truth, run the following script:

``python kmeans.py``

This script will return a CSV file with the predicted cluster of each file, the label given telating to the ground truth and the success of each class and total.

To execute other clustering algorithms such as Hierarchical agglomerative clustering run:

``python3 hierarchical.py``
