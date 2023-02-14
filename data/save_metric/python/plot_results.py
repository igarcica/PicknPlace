import cv2
import pandas as pd


read_directory = "./"
write_directory = "./"

image_file = "./o1_gr_e1.png"
metrics_csv_file = "./plots/o1o2_2x2.csv"

metrics_name = ["M1","M2","M3","M4", "M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]

#Read CSV --> Metrics
metrics_df = pd.read_csv(metrics_csv_file)
file_metrics = metrics_df.loc[(metrics_df["File"]==image_file)]

for i in range(1,4:)
    metric = metrics_name[row]
    metric_value = files_metrics[metric][0]
    print(metric)
    print(metric_value)

##Print metrics in images
texto = metric
ubicacion = (200, 700)
tamaño = 25 

img = cv2.imread(image_file)
cv2.putText(img, texto, ubicacion, cv2.FONT_HERSHEY_TRIPLEX, tamaño, (221, 82, 196), 10)
cv2.imwrite(image_file, img)

print("Reading PCD files")
directory = './'
for filename in sorted(os.listdir(directory)):
    f = os.path.join(directory, filename)
    if os.path.isfile(f) and filename.endswith('.png'):
        print(filename)

        img = cv2.imread(filename)
        cv2.putText(img, texto, ubicacion, cv2.FONT_HERSHEY_TRIPLEX, tamaño, (221, 82, 196), 10)
        cv2.imwrite(image_file, img)
    



#Read CSV --> Sizes
