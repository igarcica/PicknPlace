import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import OneHotEncoder, StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report
import joblib

######### SEPARATED TRAIN/TEST #########

# Load train and test datasets
train_file = "prediction_module_train.csv"  # Update with actual filename
test_file = "prediction_module_test.csv"    # Update with actual filename

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
X_test_cat = encoder.transform(X_test[categorical_features])

X_train_cat_df = pd.DataFrame(X_train_cat, columns=encoder.get_feature_names(categorical_features))
X_test_cat_df = pd.DataFrame(X_test_cat, columns=encoder.get_feature_names(categorical_features))

# Scale numerical features (Fit on train, Transform on both)
scaler = StandardScaler()
X_train_num_scaled = scaler.fit_transform(X_train[numerical_features])
X_test_num_scaled = scaler.transform(X_test[numerical_features])

X_train_num_df = pd.DataFrame(X_train_num_scaled, columns=numerical_features)
X_test_num_df = pd.DataFrame(X_test_num_scaled, columns=numerical_features)

# Combine numerical and categorical data
X_train_processed = pd.concat([X_train_num_df, X_train_cat_df], axis=1)
X_test_processed = pd.concat([X_test_num_df, X_test_cat_df], axis=1)

# Train Random Forest model
rf_model = RandomForestClassifier(n_estimators=100, random_state=42)
rf_model.fit(X_train_processed, y_train)

# Make predictions
y_pred = rf_model.predict(X_test_processed)

# Evaluate model performance
accuracy = accuracy_score(y_test, y_pred)
print(f"Accuracy: {accuracy:.2f}")
print("Classification Report:\n", classification_report(y_test, y_pred))

# Feature importance
feature_importance = pd.Series(rf_model.feature_importances_, index=X_train_processed.columns)
print("\nFeature Importance:\n", feature_importance.sort_values(ascending=False))




# Save the trained model
model_filename = "random_forest_model.pkl"
joblib.dump(rf_model, model_filename)
print(f"Model saved as {model_filename}")

# # Load the saved model
# rf_model_loaded = joblib.load("random_forest_model.pkl")
# print("Model loaded successfully!")

# ### PREDICT NEW SAMPLE
# # Example new sample (Replace these values with real ones)
# new_sample = pd.DataFrame({
#     "Layers": ["8l"],
#     "Grasp": ["short"],
#     "NonGraspedSize": [26],
#     "GraspedSize": [25],
#     "Area": [650],
#     "FoldStiffness": [100],
#     "Friction": [80]
# })

# # Encode categorical variables (Use the same encoder fitted on training data)
# new_sample_cat = encoder.transform(new_sample[categorical_features])
# new_sample_cat_df = pd.DataFrame(new_sample_cat, columns=encoder.get_feature_names(categorical_features))

# # Scale numerical variables (Use the same scaler fitted on training data)
# new_sample_num_scaled = scaler.transform(new_sample[numerical_features])
# new_sample_num_df = pd.DataFrame(new_sample_num_scaled, columns=numerical_features)

# # Combine numerical and categorical data
# new_sample_processed = pd.concat([new_sample_num_df, new_sample_cat_df], axis=1)

# # Predict deformation class
# predicted_class = rf_model.predict(new_sample_processed)

# print(f"Predicted deformation class: {predicted_class[0]}")





# # Load dataset
# file_path = "prediction_module.csv"
# df = pd.read_csv(file_path)

# # Identify feature types
# numerical_features = ["NonGraspedSize", "GraspedSize", "Area", "FoldStiffness"]  # Friction is not significant
# categorical_features = ["Object", "Layers", "Grasp"] #Object and Edge are not significant

# X = df[numerical_features + categorical_features] # Separate features and target variable
# y = df["DefClass"]  # Classes (supervised)

# # Encode categorical variables
# encoder = OneHotEncoder(drop="first", sparse=False)  # One-hot encoding
# X_cat = encoder.fit_transform(df[categorical_features])
# # X_cat_df = pd.DataFrame(X_cat, columns=encoder.get_feature_names_out(categorical_features))
# X_cat_df = pd.DataFrame(X_cat, columns=encoder.get_feature_names(categorical_features))


# # Scale numerical features
# scaler = StandardScaler()
# X_num_scaled = scaler.fit_transform(df[numerical_features])
# X_num_df = pd.DataFrame(X_num_scaled, columns=numerical_features)

# # Combine numerical and categorical data
# X_processed = pd.concat([X_num_df, X_cat_df], axis=1)

# # Split into training and testing sets (80% train, 20% test)
# X_train, X_test, y_train, y_test = train_test_split(X_processed, y, test_size=0.2, random_state=42)

# # Train Random Forest model
# rf_model = RandomForestClassifier(n_estimators=100, random_state=42)
# rf_model.fit(X_train, y_train)

# # Make predictions
# y_pred = rf_model.predict(X_test)

# # Evaluate model performance
# accuracy = accuracy_score(y_test, y_pred)
# print(f"Accuracy: {accuracy:.2f}")
# print("Classification Report:\n", classification_report(y_test, y_pred))

# # Feature importance
# feature_importance = pd.Series(rf_model.feature_importances_, index=X_processed.columns)
# print("\nFeature Importance:\n", feature_importance.sort_values(ascending=False))