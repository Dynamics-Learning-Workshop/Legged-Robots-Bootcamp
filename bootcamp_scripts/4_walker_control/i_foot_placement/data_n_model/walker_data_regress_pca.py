from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
import numpy as np
import pickle
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

input_array = np.load('input_array_16.npy')
output_array = np.load('output_array_16.npy')

# print(input_array[:,3])
# print(output_array[3])
print(np.max(input_array[:,3])) # max of desired q0dot
print(np.min(input_array[:,3])) # min of desired q0dot
print(np.median(input_array[:,3]))
print(np.mean(input_array[:,3]))
# Calculate the mean of the 4th column
# mean_value = np.mean(input_array[:, 3])
# print(f"Mean of the 4th column: {mean_value}")

# # Plot the distribution of the 4th column
# plt.figure(figsize=(8, 6))
# plt.hist(input_array[:, 3], bins=30, color='skyblue', edgecolor='black')
# plt.title('Distribution of the 4th Column')
# plt.xlabel('Values')
# plt.ylabel('Frequency')

# # Show the plot
# plt.axvline(mean_value, color='red', linestyle='dashed', linewidth=1)
# plt.text(mean_value, plt.ylim()[1] * 0.9, f'Mean: {mean_value:.2f}', color='red')
# plt.show()
# exit()
# print(np.mean(input_array[:,3]))

def regress(data, y):
    # degree = 7
    # poly = PolynomialFeatures(degree=degree)
    # X_poly = poly.fit_transform(data)
    
    pca = PCA(n_components=3)
    X_reduced = pca.fit_transform(data)
    
    model = LinearRegression()
    model.fit(X_reduced, y)

    # Save the polynomial transformation and regression model to a file
    # with open('test_model.pkl', 'wb') as f:
        # pickle.dump((poly, model), f)
    # Save the PCA model
    with open('pca_model.pkl', 'wb') as f:
        pickle.dump((pca, model), f)

    print("Model saved successfully.")
    return True

regress(input_array, output_array)