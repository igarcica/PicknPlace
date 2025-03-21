import pandas as pd
from scipy.stats import chi2_contingency
from scipy.stats import f_oneway
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np




directory="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/statistical_analysis/"

n_div = 3
n_clusters = 3
# cluster_to_check = 'AllCluster3x3'
# cluster_to_check = 'HumanCluster'
cluster_to_check = 'ClusterLabel'
print("GRID: ", n_div)

show_imgs = True
save_imgs = False
save_csv = False
activate_print = False

clusters_directory = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/train_test_3objs/" + str(n_div) + "x" + str(n_div) + "/clusters_raw/" + str(n_clusters)+ "clusters/"
clusters_file = clusters_directory + "pred_all_labels.csv"
human_cluster_directory = directory + "human_GT_labels.csv"
# features_directory = directory + "features.csv"
features_directory = directory + "features_modified_nongrasped.csv"

# plt.ion()


##################################################################################################

features_df = pd.read_csv(features_directory) ## Read CSV with features
cluster_df = pd.read_csv(clusters_file) ## Read CSV with cluster labels
# print(cluster_df.head()) # Display the first few rows to inspect
# human_cluster_labels_df = pd.read_csv(human_cluster_directory) ## Read CSV with features

# ## Contingency table
# for cluster in range(0,n_clusters):
#     print("cluster: ", cluster)
#     cluster_data = features_df[features_df['Cluster'] == cluster] # Filter rows where "Cluster" equals 1
#     contingency_table = cluster_data[feature].value_counts() # Create a contingency table for the "Grasp" column
    
#     print("Contingency Table for 'Grasp' is 'Cluster' ", cluster)
#     print(contingency_table)

## The saved CSV with cluster labels correspond to the following class names. Therefore we make a remaping to plot them in order A,B,C
cluster_mapping = {0: "B", 1: "A", 2: "C"} # Define the mapping
cluster_df["mapped_cluster"] = cluster_df[cluster_to_check].map(cluster_mapping) # Apply the mapping to the cluster labels

chi2_pvalues = []
chi2_feature_names = []
## Contingency table and Chi-square test for categorical parameters (Object, Layers, Grasp, Edge)
print("\033[94m ---------CHI-SQUARE TEST---------\033[0m")
exclude_columns = ['Filename', 'NonGraspedSize', 'GraspedSize', 'Area', 'FoldStiffness', 'Friction', 'HumanCluster', 'AllCluster3x3', 'AllCluster7x7', 'TrainCluster3x3', 'AllDataTrain7x7', 'Elasticity', 'Elasticity2', 'ClusterLabel'] # Specify the column(s) to exclude
for column in features_df.columns: # Loop through and print column headers, excluding specific ones
    if column not in exclude_columns:
        print("-----------------------------------------------------")
        print("Feature: ", column)
        feature = column
        # feature = 'Layers'
        contingency_table = pd.crosstab(cluster_df["mapped_cluster"], features_df[feature])
        print(contingency_table)
        if(save_csv):
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
        
        chi2_pvalues.append(p)
        chi2_feature_names.append(feature)

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

        # # Highlight values with high absolute residuals
        # significant_residuals = residuals_df.abs() > 2
        # print("\nSignificant Residuals (absolute value > 2):")
        # print(significant_residuals)

        ## Visualize relationships with stacked bar chart
        # if(show_imgs):
        #     combined_df = pd.concat([features_df, cluster_df], axis=1) # Combine the DataFrames for plotting
        #     # combined_df["mapped_cluster"] = combined_df[cluster_to_check].map(cluster_mapping) # Update the cluster labels in the combined DataFrame
        #     sns.countplot(x="mapped_cluster", hue=feature, data=combined_df, order=["A", "B", "C"])   
        #     # plt.title(f"Distribution of '{feature}' Across '{cluster_to_check}'")
        #     # plt.title(f"Distribution of '{feature}' parameter")
        #     # plt.xlabel("Deformation classes")
        #     # plt.ylabel("Contingency table count")
        #     plt.title(f"'{feature}'", fontsize=20)
        #     plt.xlabel("Deformation classes", fontsize=18)
        #     # plt.ylabel("Deformation classes pairwise comparison", fontsize=18)
        #     plt.ylabel("Contingency table count", fontsize=18)
        #     plt.xticks(fontsize=14)  # Increase x-tick labels size
        #     plt.yticks(fontsize=14)
        #     plt.show()

# ## Plot P-values
# print(chi2_pvalues)
# fig, ax = plt.subplots()
# ax.bar(chi2_feature_names, chi2_pvalues)
# ax.set_ylabel('Chi2 p-value')
# ax.set_title('Chi2 test results')
# plt.show()


## ANOVA test for continuous parameters (Size, Area, Stiffness, Friction)
print("\033[94---------ANOVA TEST---------\033[0")
anova_pvalues = []
anova_feature_names = []
i=0
fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(30, 60))  # Create a 2-row, 3-column figure
axes = axes.flatten() # Flatten the axes array for easy iteration
fig2, axes2 = plt.subplots(nrows=2, ncols=3, figsize=(30, 60))  # Create a 2-row, 3-column figure
axes2 = axes2.flatten() # Flatten the axes array for easy iteration
# exclude_columns = ['Filename', 'Object', 'Layers', 'Grasp', 'Edge', 'HumanCluster', 'AllCluster3x3', 'AllCluster7x7', 'TrainCluster3x3', 'AllDataTrain7x7',  'NonGraspedEdgeSize', 'GraspedEdgeSize', 'Area', 'Stiffness'] # Specify the column(s) to exclude
exclude_columns = ['Filename', 'Object', 'NumberLayers', 'GraspLocation', 'EdgeType', 'HumanCluster', 'AllCluster3x3', 'AllCluster7x7', 'TrainCluster3x3', 'AllDataTrain7x7',  'Elasticity2', 'ClusterLabel'] # Specify the column(s) to exclude
for column in features_df.columns: # Loop through and print column headers, excluding specific ones
    if column not in exclude_columns:
        print("-----------------------------------------------------")
        print("Feature: ", column)
        feature = column
        # Split the data by cluster
        groups = [features_df[cluster_df["mapped_cluster"] == c][feature] for c in cluster_df["mapped_cluster"].unique()]
        # ANOVA Test
        f_stat, p_value = f_oneway(*groups)
        print(f"ANOVA Test for {feature}:")
        print(f"F-statistic: {f_stat}, p-value: {p_value}")
        # Check significance
        if p_value < 0.05:
            print(f"The feature \033[94m '{feature}' \033[0m is significantly different among clusters.")
        else:
            print(f"The feature \033[94m '{feature}' \033[0m is \033[94m NOT \033[0m significantly different among clusters.")

        anova_pvalues.append(p_value)
        anova_feature_names.append(feature)

        ## Identify which clusters have significantly different feature values
        ## Perform Tukey's HSD (Honestly Significant Difference) test (Post-Hoc for ANOVA)
        tukey = pairwise_tukeyhsd(endog=features_df[feature],  # Feature values
                                groups=cluster_df["mapped_cluster"],    # Cluster labels
                                alpha=0.05)

        print(tukey)

        # Extract results from Tukey HSD test
        results = tukey.summary()
        print(results)

        # Extract relevant columns for plotting
        groups = [f"{row[0]}-{row[1]}" for row in results.data[1:]]
        means = [row[2] for row in results.data[1:]]
        lower_bounds = [row[4] for row in results.data[1:]]
        upper_bounds = [row[5] for row in results.data[1:]]
        
        if(show_imgs):
            
            # fig = tukey.plot_simultaneous()
            # plt.show()

            # ## Plot confidence intervals (mean differences between clusters) of all features
            # axes[i].errorbar(means, np.arange(len(groups)), xerr=[np.abs(lower_bounds), np.abs(upper_bounds)], fmt='o')
            # axes[i].set_yticks(np.arange(len(groups)))
            # axes[i].set_yticklabels(groups)
            # axes[i].axvline(x=0, color='gray', linestyle='--')
            # axes[i].set_title(f"'{feature}'", fontsize=20)
            # axes[i].set_xlabel("Mean Difference", fontsize=18)
            # axes[i].set_ylabel("Pairwise comparison", fontsize=18)
            # plt.xticks(fontsize=14)  # Increase x-tick labels size
            # plt.yticks(fontsize=14)
            # fig.subplots_adjust(hspace=0.5, wspace=0.3)  # Increase spacing between rows
            # fig.suptitle("Tukey's HSD Confidence Intervals", fontsize=24, fontweight='bold')
            # # plt.tight_layout()# Adjust layout to prevent overlap

            ## Plot confidence intervals (mean differences between clusters)
            plt.errorbar(means, np.arange(len(groups)), xerr=[np.abs(lower_bounds), np.abs(upper_bounds)], fmt='o')
            plt.yticks(np.arange(len(groups)), groups)
            plt.axvline(x=0, color='gray', linestyle='--')
            # plt.title(f"Tukey's HSD Confidence Intervals for '{feature}' accross '{cluster_to_check}'")
            # plt.title(f"Tukey's HSD Confidence Intervals for '{feature}'", fontsize=18)
            plt.title(f"'{feature}'", fontsize=20)
            plt.xlabel("Mean Difference", fontsize=18)
            # plt.ylabel("Deformation classes pairwise comparison", fontsize=18)
            plt.ylabel("Pairwise comparison", fontsize=18)
            plt.xticks(fontsize=14)  # Increase x-tick labels size
            plt.yticks(fontsize=14)
            plt.show()

            # ## Plot BOX plots of all features
            # feature_name = ['Not grasped edge size (cm)', 'Grasped edge size (cm)', 'Area (cm²)', 'Stiffness (%)', 'Elasticity (%)', 'Friction (%)']
            # combined_df = pd.concat([features_df, cluster_df], axis=1) # Combine the DataFrames for plotting
            # sns.boxplot(x=cluster_df["mapped_cluster"], y=features_df[feature], order=["A", "B", "C"], ax=axes2[i])
            # axes2[i].set_title(f"'{feature}'", fontsize=20)
            # axes2[i].set_xlabel("Deformation class", fontsize=18)
            # axes2[i].set_ylabel(f"{feature_name[i]}", fontsize=18)
            # plt.xticks(fontsize=14)  # Increase x-tick labels size
            # plt.yticks(fontsize=14)
            # fig2.subplots_adjust(hspace=0.5, wspace=0.3)  # Increase spacing between rows
            # fig2.suptitle("Distribution of numerical parameters", fontsize=24, fontweight='bold')

            ## Plot BOX/Violing plots
            feature_name = ['Not grasped edge size (cm)', 'Grasped edge size (cm)', 'Area (cm²)', 'Stiffness (%)', 'Elasticity (%)', 'Friction (%)']
            combined_df = pd.concat([features_df, cluster_df], axis=1) # Combine the DataFrames for plotting
            # sns.boxplot(x=cluster_to_check, y=feature, data=combined_df)
            sns.boxplot(x=cluster_df["mapped_cluster"], y=features_df[feature], order=["A", "B", "C"])
            # plt.title(f"Distribution of '{feature}' across '{cluster_to_check}'")
            # plt.title(f"Distribution of '{feature}' values in the deformation classes", fontsize=24)
            # plt.title(f"Distribution of '{feature}' values", fontsize=20)
            plt.title(f"'{feature}'", fontsize=20)
            plt.xlabel("Deformation class", fontsize=18)
            # plt.ylabel(f"'{feature}' values")
            # plt.ylabel("Elasticity (%)", fontsize=18)
            plt.ylabel(f"{feature_name[i]}", fontsize=18)
            plt.xticks(fontsize=14)  # Increase x-tick labels size
            plt.yticks(fontsize=14)
            plt.show()

            # # plot violin plot
            # copy_df = features_df[[feature]].copy()
            # copy_df['Class'] = cluster_df["mapped_cluster"] # Merge features and class dataframes into a single dataframe
            # # copy_df.iloc[:, :-1] = copy_df.iloc[:, :-1].apply(pd.to_numeric)
            # df_melted = copy_df.melt(id_vars='Class', var_name='Feature', value_name='Value') # Melt the dataframe for seaborn    
            # sns.violinplot(x='Class', y='Value', data=df_melted, box=True, showmeans=True, showmedians=True, opacity=0.4)
            # # fig.update_traces(quartilemethod="exclusive") # or "inclusive", or "linear" by default
            # # plt.violinplot(cluster_df["mapped_cluster"], features_df[feature], showmeans=False, showmedians=True)
            # plt.title(f"Distribution of '{feature}' values in the deformation classes")
            # plt.xlabel("Deformation class")
            # plt.ylabel(f"'{feature}' values")
            # plt.show()

            i+=1
plt.show()

        

# ## Plot P-values
print(anova_pvalues)
# fig, ax = plt.subplots()
# ax.bar(anova_feature_names, anova_pvalues)
# ax.set_ylabel('ANOVA p-value')
# ax.set_title('ANOVA test results')
# plt.show()

# ### PLOT P-VALUES
# import matplotlib.pyplot as plt

# fig, ax = plt.subplots()

# fruits = ['apple', 'blueberry', 'cherry', 'orange']
# counts = [40, 100, 30, 55]
# bar_labels = ['red', 'blue', '_red', 'orange']
# bar_colors = ['tab:red', 'tab:blue', 'tab:red', 'tab:orange']

# ax.bar(fruits, counts, label=bar_labels, color=bar_colors)

# ax.set_ylabel('fruit supply')
# ax.set_title('Fruit supply by kind and color')
# ax.legend(title='Fruit color')

# plt.show()


# # Matriz de ejemplo
# matriz_pval = np.array([[1.0, 0.03, 0.0001],
#                          [0.03, 1.0, 0.045],
#                          [0.0001, 0.045, 1.0]])

# labels = ['A', 'B', 'C']

# # Crear dataframe
# df_pval = pd.DataFrame(matriz_pval, index=labels, columns=labels)

# # Heatmap
# sns.heatmap(df_pval, annot=True, cmap="coolwarm", center=0.05)
# plt.title("Mapa de calor de diferencias significativas")
# plt.show()


# Grafica de p-values para todos los parametros para identificar cuales son significativos. Luego contingency y tukey para ver en que clases influye
# Cambiar nombre de NonGraspedSize por Not Grasped Edge Size