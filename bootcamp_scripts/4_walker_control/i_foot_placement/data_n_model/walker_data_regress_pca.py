from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import numpy as np
import pickle
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GridSearchCV
from sklearn.pipeline import Pipeline


# modified_
input_array = np.load('input_array_16.npy')

input_array[:,2] = input_array[:,1] + input_array[:,2]

# normalized_input_array = (input_array - input_array.min(axis=0)) / (input_array.max(axis=0) - input_array.min(axis=0))
scaler = StandardScaler()
normalized_input_array = scaler.fit_transform(input_array)
output_array = np.load('output_array_16.npy')

print(input_array.min(axis=0))
print(input_array.max(axis=0))

def regress(data, y):
    pca = PCA(n_components=3)
    X_reduced = pca.fit_transform(data)
    
    degree = 3
    poly = PolynomialFeatures(degree=degree)
    X_poly = poly.fit_transform(X_reduced)
    
    model = LinearRegression()
    model.fit(X_poly, y)

    with open('pca_model_here.pkl', 'wb') as f:
        pickle.dump((pca, poly, model), f)

    print("Model saved successfully.")
    return True

regress(normalized_input_array, output_array)