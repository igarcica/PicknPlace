import cv2
import pandas as pd
import os

n_div = 2
n_grids = n_div*n_div #4, 9, 16, 25...
metrics_name = ["M1","M2","M3","M4", "M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]

#data_directory="/home/pal/Desktop/all/dataset/Picks/RGB/cropped/"
data_directory="/home/pal/Desktop/more_data/dataset/RGB/"
write_directory = "./results/4/" + str(n_div) + "x" + str(n_div) + "/"

metrics_csv_file = str(n_div) + "x" + str(n_div) + ".csv" ##o1_2x2.csv
metrics_csv_dir = write_directory+metrics_csv_file

all_files = True
image_file = "o1_gr_can.png"
image_dir = data_directory+image_file

save_img = True
show_img = False

crop_imgs=False
print_metrics=True


##################################################################################################

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
    print("Printing metrics in images...")
    image_name = image_file.replace(".png","")
    file_metrics = metrics_df.loc[(metrics_df["File"]==image_name)]
    #metric_value = float(file_metrics[metrics_name[0]])
    # for i in range(0,n_grids):
    #     metric_value = file_metrics[metrics_name[i]]

    ##Print metrics in images
    size = 1
    divs = n_div*2
    locs_x = []
    locs_y = []
    k=0
    for i in range(0,divs,2):
        locs_x.append((img.shape[0]/divs*(i+1))-20)
        locs_y.append((img.shape[1]/divs*(i+1))-20)
        #print(img.shape[0]/divs*(i+1))
        #print(img.shape[1]/divs*(i+1))
    for n in range(0,n_div):
        for m in range(0,n_div):
            metric_value = float(file_metrics[metrics_name[k]])
            text = str(round(metric_value,3))
            cv2.putText(img, text, (locs_y[n], locs_x[m]), cv2.FONT_HERSHEY_SIMPLEX, size, (221, 82, 196), 2)
            k+=1
    cv2.putText(img, image_name, (295,505), cv2.FONT_HERSHEY_SIMPLEX, size, (0,0,0), 2)

    if(save_img):
        result_image_dir = write_directory+image_file
        cv2.imwrite(result_image_dir, img)

    if(show_img):
        cv2.imshow("Result",img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

##################################################################################################
metrics_df = pd.read_csv(metrics_csv_dir) ##Read CSV --> Metrics
print(metrics_df)

if not(all_files):
    img = cv2.imread(image_dir)
    #print(img.shape)
    if(crop_imgs):
        crop_image(image_dir)
    if(print_metrics):
        metrics_df = pd.read_csv(metrics_csv_dir) ##Read CSV --> Metrics
        print_metric_img(img, image_file, metrics_df) ## Print metrics in image

if(all_files):
    for filename in sorted(os.listdir(data_directory)):
        f = os.path.join(data_directory, filename)
        if os.path.isfile(f) and filename.endswith('.png'):
            img = cv2.imread(f)
            if(crop_imgs):
                crop_image(f)
            if(print_metrics):
                print_metric_img(img, filename, metrics_df)


# img = cv2.imread(image_dir)
# cv2.imshow("test",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
#
# ###Crop images and save images
# #crop_img = crop_image(img)
#
#
# ##Read CSV --> Metrics
# metrics_df = pd.read_csv(metrics_csv_dir)
# image_name = image_file.replace(".png","")
# file_metrics = metrics_df.loc[(metrics_df["File"]==image_name)]
# metric_value = float(file_metrics[metrics_name[0]])
# # for i in range(0,n_grids):
# #     metric_value = file_metrics[metrics_name[i]]
#
#
# ##Print metrics in images
# text = str(metric_value)
# size = 1
#
# divs = n_div*2
# locs_x = []
# locs_y = []
# k=0
# for i in range(0,divs,2):
#     locs_x.append((img.shape[0]/divs*(i+1))-20)
#     locs_y.append((img.shape[1]/divs*(i+1))-20)
#     #print(img.shape[0]/divs*(i+1))
#     #print(img.shape[1]/divs*(i+1))
# for n in range(0,n_div):
#     for m in range(0,n_div):
#         metric_value = float(file_metrics[metrics_name[k]])
#         text = str(metric_value)
#         cv2.putText(img, text, (locs_y[n], locs_x[m]), cv2.FONT_HERSHEY_SIMPLEX, size, (221, 82, 196), 2)
#         k+=1
#
# result_image_dir = write_directory+image_file
# cv2.imwrite(result_image_dir, img)
# cv2.imshow("Result",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()




#for filename in sorted(os.listdir(read_directory)):
#    f = os.path.join(read_directory, filename)
#    if os.path.isfile(f) and filename.endswith('.png'):
#        print(f)
#        img = cv2.imread(f)
##        h,w,c = img.shape
##        print(w)
##        cv2.putText(img, text, loc, cv2.FONT_HERSHEY_TRIPLEX, size, (221, 82, 196), 10)
#        cv2.imshow("test",img)
#        cv2.waitKey(0)
#        cv2.destroyAllWindows()
##        #cv2.imwrite(image_file, img)




#Read CSV --> Sizes
