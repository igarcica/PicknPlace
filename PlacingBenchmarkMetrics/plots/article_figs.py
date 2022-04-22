import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np

#Load file with Et values
A_total_error = pd.read_csv('./data/A_total_error.csv', index_col=0)
B_total_error = pd.read_csv('./data/B_total_error.csv', index_col=0)
C_total_error = pd.read_csv('./data/C_total_error.csv', index_col=0)

#Load file with Egz and Ehz values
A_error_zones = pd.read_csv('./data/A_error_zones.csv', index_col=0)
B_error_zones = pd.read_csv('./data/B_error_zones.csv', index_col=0)
C_error_zones = pd.read_csv('./data/C_error_zones.csv', index_col=0)

# Fig. 6 - Plot mean quality function Et + standard deviation
order=["Std_dev", "Error"]
g = sns.relplot(x="x", y="y", size="error", hue="error_stdev", col="garment", row="trajectory",
            sizes=(0, 3000), size_norm=(0,0.3), alpha=0.7, palette=["yellowgreen","mediumseagreen"], 
            height=4, data=A_total_error, legend=False)
g.set_axis_labels("x", "y")
plt.gca().invert_yaxis()
plt.xticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.yticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.show()

g = sns.relplot(x="x", y="y", size="error", hue="error_stdev", col="garment", row="trajectory",
            sizes=(0, 3000), size_norm=(0,0.3), alpha=0.7, palette=["khaki","gold"],
            height=4, data=B_total_error, legend=False)
g.set_axis_labels("x", "y")
plt.gca().invert_yaxis()
plt.xticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.yticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.show()

g = sns.relplot(x="x", y="y", size="error", hue="error_stdev", col="garment", row="trajectory",
            sizes=(0, 3000), size_norm=(0,0.3), alpha=0.7, palette=["orangered", "lightsalmon"], 
            height=4, data=C_total_error, legend=False)
g.set_axis_labels("x", "y")
plt.gca().invert_yaxis()
plt.xticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.yticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.show()

# Fig. 7 - Plot mean quality function Egz and Ehz
# Plot Garment A
g = sns.relplot(x="x", y="y", size="error", hue="zone", col="garment", row="trajectory",
            sizes=(40, 5000), size_norm=(0,0.5), alpha=0.7, palette=["mediumseagreen", "yellowgreen"],#lightgreen","springgreen"],
            height=4, data=A_error_zones, legend=False)
g.set_axis_labels("x", "y")
plt.gca().invert_yaxis()
plt.xticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.yticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.show()

## Plot Garment B
g = sns.relplot(x="x", y="y", size="error", hue="zone", col="garment", row="trajectory",
            sizes=(40, 5000), size_norm=(0,0.5), alpha=0.7, palette=["khaki", "gold"],#golderod, gold
            height=4, data=B_error_zones, legend=False)
g.set_axis_labels("x", "y")
plt.gca().invert_yaxis()
plt.xticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.yticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.show()

# Plot Garment C
g = sns.relplot(x="x", y="y", size="error", hue="zone", col="garment", row="trajectory",
            sizes=(40, 5000), size_norm=(0,0.5), alpha=0.7, palette=["coral", "orangered"],
            height=4, data=C_error_zones, legend=False)
g.set_axis_labels("x", "y")
plt.gca().invert_yaxis()
plt.xticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.yticks((0,12.5,25,37.5), ("0%","12.5%","25%","37.5%"))
plt.show()

