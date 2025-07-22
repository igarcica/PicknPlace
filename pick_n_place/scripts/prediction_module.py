#!/usr/bin/env python3
import rospy
import ros_numpy
import pandas as pd
import joblib
from sklearn.preprocessing import OneHotEncoder, StandardScaler
from pick_n_place.srv import PredictDefClass, PredictDefClassResponse

# global global_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/placing_metric/"

def load_trained_model():

    global_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/placing_metric/"
    trained_model_dir = global_dir + "random_forest_model.pkl"
    # Feature columns
    numerical_features = ["NonGraspedSize", "GraspedSize", "Area", "FoldStiffness"]  # Friction is not significant
    categorical_features = ["Layers", "Grasp"] #Object and Edge are not significant
    target_column = "DefClass"  

    # Load the saved model
    rf_model_loaded = joblib.load(trained_model_dir)
    # print("Deformation Class Prediction: Model loaded successfully!")
    rospy.loginfo("Deformation Class Prediction: Model loaded successfully")

    # new_sample_processed = prepare_data(new_sample)

    # # Predict deformation class
    # predicted_class = rf_model_loaded.predict(new_sample_processed)

    # print(f"Predicted deformation class: {predicted_class[0]}")

    # return predicted_class[0]
    return rf_model_loaded

def prepare_data(new_sample):
    global_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/placing_metric/"

    # Load train and test datasets
    train_file = global_dir + "prediction_module_train.csv"  # Update with actual filename
    test_file = global_dir + "prediction_module_test.csv"    # Update with actual filename

    df_train = pd.read_csv(train_file)
    df_test = pd.read_csv(test_file)

    # Feature columns
    numerical_features = ["NonGraspedSize", "GraspedSize", "Area", "FoldStiffness"]  # Friction is not significant
    categorical_features = ["Layers", "Grasp"] #Object and Edge are not significant
    target_column = "DefClass"  

    # Separate features and target variable
    X_train = df_train[numerical_features + categorical_features]
    y_train = df_train[target_column]

    X_test = df_test[numerical_features + categorical_features]
    y_test = df_test[target_column]

    # Encode categorical variables (Fit on train, Transform on both)
    encoder = OneHotEncoder(drop="first", sparse=False)
    X_train_cat = encoder.fit_transform(X_train[categorical_features])
    X_train_cat_df = pd.DataFrame(X_train_cat, columns=encoder.get_feature_names(categorical_features))

    # Scale numerical features (Fit on train, Transform on both)
    scaler = StandardScaler()
    X_train_num_scaled = scaler.fit_transform(X_train[numerical_features])
    X_train_num_df = pd.DataFrame(X_train_num_scaled, columns=numerical_features)

    # Combine numerical and categorical data
    X_train_processed = pd.concat([X_train_num_df, X_train_cat_df], axis=1)

    # Encode categorical variables (Use the same encoder fitted on training data)
    new_sample_cat = encoder.transform(new_sample[categorical_features])
    new_sample_cat_df = pd.DataFrame(new_sample_cat, columns=encoder.get_feature_names(categorical_features))

    # Scale numerical variables (Use the same scaler fitted on training data)
    new_sample_num_scaled = scaler.transform(new_sample[numerical_features])
    new_sample_num_df = pd.DataFrame(new_sample_num_scaled, columns=numerical_features)

    # Combine numerical and categorical data
    new_sample_processed = pd.concat([new_sample_num_df, new_sample_cat_df], axis=1)

    return new_sample_processed


def handle_service(req):

    # Example new sample (Replace these values with real ones)
    sample = pd.DataFrame({
        "Layers": [req.layers],
        "Grasp": [req.grasp],
        "NonGraspedSize": [req.nongraspedsize],
        "GraspedSize": [req.graspedsize],
        "Area": [req.area],
        "FoldStiffness": [req.stiffness],
        "Friction": [req.friction]
    })

    # int_def_class = predict_def_class(sample)
    new_sample_processed = prepare_data(sample)

    # Predict deformation class
    int_def_class = trained_model.predict(new_sample_processed)

    # print(f"Predicted deformation class: {int_def_class[0]}")
    rospy.logdebug("Service predict_def_class: Predicted %i", int_def_class[0])

    #If class is 0 then send "A", etc
    if(int_def_class == 0):
        str_def_class = "B"
    elif(int_def_class == 1):
        str_def_class = "A"
    elif(int_def_class == 2):
        str_def_class = "C"

    return PredictDefClassResponse(str_def_class)

if __name__ == '__main__':
    rospy.init_node('deformation_class_prediction', anonymous=True)
    rospy.loginfo("Deformation Class Prediction: Node ready")
    s = rospy.Service('/pick_n_place/predict_def_class', PredictDefClass, handle_service)
    trained_model = load_trained_model()
    rospy.spin()


### ROS PREDICTION MODULE
## ROS Node to predict deformation class for new set of object parameters


