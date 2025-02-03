# Grasping metric

## Obtain metrics given a PCD file + clusterize

1. Obtain grid metrics for a fiven number of grid divisions:
Modify the grid division, the data_directory to the one where PCD files are, and directory to where the data is goind to be saved. This saves a CSV with the grid metrics.
``python3 grasping_grid_metric.py``

2. Get the complete model clusters and labels to be used as GT for evaluating clustering accuracy.
Execute the following for obtaining three CSV files with the labels obtained with the complete model (trained with all the data), with labels of all files, just the labels of train files and just the labels of test files.
``python3 clusters_raw.py``

3. Get the partial model training with train data and obtain the labels separately for the test and train data.

`python3 clusters_raw_traintest.py`


# Placing metric

``placing_grid_metric.py``


# Prediction module

Train and predict deformation class given a set of object parameters (object, number of folds, grasped edge size, stiffness, friction).
Necessary files: ``data/placing_metric/prediction_module_train.csv`` and ``data/placing_metric/prediction_module_test.csv``

``prediction_module.py``

