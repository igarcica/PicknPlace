Institut de Robòtica i Informàtica Industrial, CSIC-UPC. Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Baseline results plots

This repository is a python package for visualizing the results of the baseline system. It plots the spreading error of the entire cloth and of the two zones of the garment (GZ and HZ), allowing to distinguish in a simple glanze how the initial grasing position affects to the task result.

<p align="center">
  <img src="./readme_img.png" />
</p>

## Usage

It is necessary to place the csv files with the data containing the baseline results in the folder */data*. This files should contain at least three columns: the distance to the corner in both adjacent edges (x and y) and the spreading error. In addition, for plotting the spreading error in each zone, another column is necessary to identify the zone (GZ/HZ). 

In our case, as we used three different garments and we implemented two baselines for comparison which performed two different trajectories for placing, we have in addition to the previous ones, two more columns for distinguishing between the cloth object and the trajectory. Note that for plotting purposes, separated csv files are necessary for each object and error type. 


To execute the code and visualize the results, run in the root of the repository:

``python article_figs.py``

It will display 6 different windows with the results for the three objects and two trajectories. Note that in this repository we include the csv files with the results of our baseline as example.
