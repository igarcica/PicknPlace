import pandas as pd
from scipy.stats import chi2_contingency
from scipy.stats import f_oneway
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np




directory="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/statistical_analysis/"

n_div = 7
n_clusters = 3
# cluster_to_check = 'AllCluster3x3'
cluster_to_check = 'HumanCluster'
print("GRID: ", n_div)

show_imgs = False
save_imgs = False
activate_print = False

# write_directory = directory + str(n_div) + "x" + str(n_div) + "/clusters_raw/"
cluster_labels_directory = directory + "human_GT_labels.csv"
features_directory = directory + "features.csv"


##################################################################################################

## Read CSV with cluster labels
cluster_labels_df = pd.read_csv(cluster_labels_directory)
# print(cluster_labels_df.head()) # Display the first few rows to inspect
## Read CSV with features
features_df = pd.read_csv(features_directory)


# ## Contingency table
# for cluster in range(0,n_clusters):
#     print("cluster: ", cluster)
#     cluster_data = features_df[features_df['Cluster'] == cluster] # Filter rows where "Cluster" equals 1
#     contingency_table = cluster_data[feature].value_counts() # Create a contingency table for the "Grasp" column
    
#     print("Contingency Table for 'Grasp' is 'Cluster' ", cluster)
#     print(contingency_table)

## Contingency table and Chi-square test for categorical parameters (Object, Layers, Grasp, Edge)
print("\033[94---------CHI-SQUARE TEST---------\033[0")
exclude_columns = ['Filename', 'NonGraspedSize', 'GraspedSize', 'Area', 'FoldStiffness', 'Friction', 'HumanCluster', 'AllCluster3x3', 'AllCluster7x7', 'TrainCluster3x3', 'AllDataTrain7x7'] # Specify the column(s) to exclude
for column in features_df.columns: # Loop through and print column headers, excluding specific ones
    if column not in exclude_columns:
        print("-----------------------------------------------------")
        print("Feature: ", column)
        feature = column
        # feature = 'Layers'
        contingency_table = pd.crosstab(features_df[cluster_to_check], features_df[feature])
        print(contingency_table)
        contingency_table_dir = directory + feature + "_contingency_table.csv"
        contingency_table.to_csv(contingency_table_dir, index=False)

        # Chi-Square Test
        chi2, p, dof, expected = chi2_contingency(contingency_table)
        print("Chi-Square Test for Size:")
        print(f"Chi2: {chi2}, p-value: {p}")

        # Check significance
        if p < 0.05:
            print(f"The feature \033[94m '{feature}' \033[0m is significantly associated with the clusters.")
        else:
            print(f"The feature \033[94m '{feature}' \033[0m is \033[94m NOT \033[0m significantly associated with the clusters.")
        

        ## Check which feature values are signigicant to which cluster
        ## Calculate standardized residuals
        observed = contingency_table.values
        std_residuals = (observed - expected) / (expected**0.5)

        # Convert residuals to a DataFrame for easy interpretation
        residuals_df = pd.DataFrame(std_residuals, 
                                    index=contingency_table.index, 
                                    columns=contingency_table.columns)

        print("Standardized Residuals:")
        print(residuals_df)

        # Highlight values with high absolute residuals
        significant_residuals = residuals_df.abs() > 2
        print("\nSignificant Residuals (absolute value > 2):")
        print(significant_residuals)

        ## Visualize relationships with stacked bar chart
        if(show_imgs):
            sns.countplot(x=cluster_to_check, hue=feature, data=features_df)   
            plt.title(f"Distribution of '{feature}' Across '{cluster_to_check}'")
            plt.show()



## ANOVA test for continuous parameters (Size, Area, Stiffness, Friction)
print("\033[94---------ANOVA TEST---------\033[0")
exclude_columns = ['Filename', 'Object', 'Layers', 'Grasp', 'Edge', 'HumanCluster', 'AllCluster3x3', 'AllCluster7x7', 'TrainCluster3x3', 'AllDataTrain7x7'] # Specify the column(s) to exclude
for column in features_df.columns: # Loop through and print column headers, excluding specific ones
    if column not in exclude_columns:
        print("-----------------------------------------------------")
        print("Feature: ", column)
        feature = column
        # Split the data by cluster
        groups = [features_df[features_df[cluster_to_check] == c][feature] for c in features_df[cluster_to_check].unique()]
        # ANOVA Test
        f_stat, p_value = f_oneway(*groups)
        print(f"ANOVA Test for {feature}:")
        print(f"F-statistic: {f_stat}, p-value: {p_value}")
        # Check significance
        if p_value < 0.05:
            print(f"The feature \033[94m '{feature}' \033[0m is significantly different among clusters.")
        else:
            print(f"The feature \033[94m '{feature}' \033[0m is \033[94m NOT \033[0m significantly different among clusters.")

        ## Identify which clusters have significantly different feature values
        ## Perform Tukey's HSD (Honestly Significant Difference) test (Post-Hoc for ANOVA)
        tukey = pairwise_tukeyhsd(endog=features_df[feature],  # Feature values
                                groups=features_df[cluster_to_check],    # Cluster labels
                                alpha=0.05)

        print(tukey)

        # Extract results from Tukey HSD test
        results = tukey.summary()
        # Extract relevant columns for plotting
        groups = [f"{row[0]}-{row[1]}" for row in results.data[1:]]
        means = [row[2] for row in results.data[1:]]
        lower_bounds = [row[4] for row in results.data[1:]]
        upper_bounds = [row[5] for row in results.data[1:]]
        # Plot confidence intervals
        if(show_imgs):
            plt.errorbar(means, np.arange(len(groups)), xerr=[np.abs(lower_bounds), np.abs(upper_bounds)], fmt='o')
            plt.yticks(np.arange(len(groups)), groups)
            plt.axvline(x=0, color='gray', linestyle='--')
            plt.title(f"Tukey's HSD Confidence Intervals for '{feature}' accross '{cluster_to_check}'")
            plt.xlabel("Mean Difference")
            plt.show()

            sns.boxplot(x=cluster_to_check, y=feature, data=features_df)
            plt.title(f"Distribution of '{feature}' across '{cluster_to_check}'")
            plt.show()
