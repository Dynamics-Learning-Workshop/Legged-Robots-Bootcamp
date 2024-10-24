import pickle
import numpy as np

# with open('test_model.pkl', 'rb') as f:
    # poly, model = pickle.load(f)
with open('pca_model.pkl', 'rb') as f:
    pca, model = pickle.load(f)

print("Data loaded successfully from pickle.")
# def predict(d):
#     dd_poly = poly.transform([d])
#     U_predicted = model.predict(dd_poly)
    
#     return U_predicted
def predict(d):
    input_min = np.array([-1.57079633, -3.14159265, -3.14159265, -2.86283747])
    input_max = np.array([1.57079633, 0.62831853, 3.14159265, 0.0482202])
    
    d_normalized = (d - input_min) / (input_max - input_min) 
    dd_pca = pca.transform([d_normalized])
    U_predicted = model.predict(dd_pca)
    
    return U_predicted

# lala = process_point(0.4, -0.5, 0.0, 0.7064577720614232)

dd = np.array([0.4, -0.5, 0.0, -1.5]) 
# -0.31588649827928467

print(predict(d=dd))

# Load the arrays
# input_array = np.load('input_array.npy')
# output_array = np.load('output_array.npy')
# print(input_array.shape)
# print(input_array[1,:])
# print(output_array[1])
# print(predict(input_array[1,:]))