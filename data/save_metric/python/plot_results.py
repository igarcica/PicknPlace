import cv2
import pandas as pd
import os

##################################################################################################
## INPUT PARAMETERS

n_div = 10
n_grids = n_div*n_div #4, 9, 16, 25...
#metrics_name = ["M1","M2","M3","M4", "M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]

#data_directory="/home/pal/Desktop/all/dataset/Picks/RGB/cropped/"
#data_directory="/home/pal/Desktop/more_data/dataset/RGB/"
#data_directory="/home/pal/Desktop/more_folds/dataset/RGB/"
data_directory="/home/pal/Desktop/all/RGB/"
write_directory = "./metrics/not_filling_peso/" + str(n_div) + "x" + str(n_div) + "/"

metrics_csv_file = str(n_div) + "x" + str(n_div) + ".csv" ##o1_2x2.csv
metrics_csv_dir = write_directory+metrics_csv_file

all_files = True
image_file = "o1_gr_e01.png"
image_dir = data_directory+image_file

save_img = True
show_img = False

crop_imgs=False
print_metrics=False


##################################################################################################
## FUNCTIONS

def crop_image(img_file):

    img = cv2.imread(img_file)

    ##Crop image
    print("Original img size: ", img.shape)
    cv2.imshow("Original",img)
    crop_img = img[0:510, 370:840]
    print("Cropped image size: ", img.shape)
    cv2.imshow("Cropped",img)

    ###Save croped images
    image_file = img_file.replace(data_directory,"")
    write_filename = "./"+image_file
    print(write_filename)
    cv2.imwrite(write_filename, crop_img)

    return crop_img

def print_metric_img(img, image_file, metrics_df):
    print("Printing metrics in image ", image_file)

    ## Get deformation metrics for the given experiment based on the file name
    image_name = image_file.replace(".png","")
    file_metrics = metrics_df.loc[(metrics_df["File"]==image_name)]

    ##Print metrics in images
    size = 1
    divs = n_div*2
    locs_x = []
    locs_y = []
    k=1
    ## Get locations where to put text based on number of buckets
    for i in range(0,divs,2):
        locs_x.append((img.shape[0]/divs*(i+1))-20)
        locs_y.append((img.shape[1]/divs*(i+1))-20)

    ## Print the each metric value in the corresponding bucket of the grid
    for n in range(0,n_div): # y coordinates
        for m in range(0,n_div): # x coordinates
            metric_name = "M"+str(k)
            metric_value = file_metrics[metric_name].values
            #metric_value = float(file_metrics[metric_name]) # Search next metric (M1, M2, M3,...)
            text = str(round(metric_value,3))
            if(print_metrics):
                cv2.putText(img, text, (locs_y[n], locs_x[m]), cv2.FONT_HERSHEY_SIMPLEX, size, (221, 82, 196), 2)
            k+=1

    ## Print file name
    cv2.putText(img, image_name, (295,505), cv2.FONT_HERSHEY_SIMPLEX, size, (0,0,0), 2)

    ## Print ground truth class
    GT_class = str(file_metrics["Class_GT"].values[0])
    cv2.putText(img, GT_class, (100,500), cv2.FONT_HERSHEY_SIMPLEX, size, (0,0,0), 2)

    if(save_img):
        result_image_dir = write_directory+image_file
        cv2.imwrite(result_image_dir, img)

    if(show_img):
        cv2.imshow("Result",img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

##################################################################################################

## MAIN


metrics_df = pd.read_csv(metrics_csv_dir) ##Read CSV --> Metrics
#print(metrics_df)


## Print metrics or crop the given image
if not(all_files):
    img = cv2.imread(image_dir)
    #print(img.shape)
    if crop_imgs :
        crop_image(image_dir)
    else:
        metrics_df = pd.read_csv(metrics_csv_dir) ##Read CSV --> Metrics
        print_metric_img(img, image_file, metrics_df) ## Print metrics in image


## Print metrics or crop for each PNG file in the given folder
if(all_files):
    for filename in sorted(os.listdir(data_directory)):
        f = os.path.join(data_directory, filename)
        if os.path.isfile(f) and filename.endswith('.png'):
            img = cv2.imread(f)
            if crop_imgs:
                crop_image(f)
            else:
                print_metric_img(img, filename, metrics_df)
