from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import numpy as np
import pickle

input_array = np.load('input_array_16.npy')
output_array = np.load('output_array_16.npy')
print(input_array[:,3])
print(output_array[3])
print(np.max(input_array[:,3]))
print(np.min(input_array[:,3]))

def regress(data, y):
    degree = 3
    poly = PolynomialFeatures(degree=degree)
    X_poly = poly.fit_transform(data)
    
    model = LinearRegression()
    model.fit(X_poly, y)

    # Save the polynomial transformation and regression model to a file
    with open('test_model.pkl', 'wb') as f:
        pickle.dump((poly, model), f)

    print("Model saved successfully.")
    return True

regress(input_array, output_array)