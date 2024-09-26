import pickle
import numpy as np

with open('test_model.pkl', 'rb') as f:
    poly, model = pickle.load(f)

print("Data loaded successfully from pickle.")
def predict(d):
    dd_poly = poly.transform([d])
    U_predicted = model.predict(dd_poly)
    
    return U_predicted

# lala = process_point(0.4, -0.5, 0.0, 0.7064577720614232)
q1_query = 0.4
u0_query = -0.5
u1_query = 0.0
q0dot_plus_query = -1.5


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