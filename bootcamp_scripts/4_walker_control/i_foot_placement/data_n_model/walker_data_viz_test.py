import numpy as np
import matplotlib.pyplot as plt

input_array = np.load('input_array_16.npy')
output_array = np.load('output_array_16.npy')

# 1. Visualize the Original Distribution
plt.figure(figsize=(8, 6))
plt.hist(input_array[:, 3], bins=30, color='skyblue', edgecolor='black')
plt.title('Original Distribution of the 4th Column')
plt.xlabel('Values')
plt.ylabel('Frequency')
plt.show()

# 2. Resample to create a more even distribution
min_val = np.min(input_array[:, 3])
max_val = np.max(input_array[:, 3])

# Create a new array with values uniformly distributed between min and max of original data
even_distribution = np.random.uniform(low=min_val, high=max_val, size=len(input_array[:, 3]))

# Replace the original 4th column with the new evenly distributed data
modified_array = input_array.copy()  # Create a copy of the original array
modified_array[:, 3] = even_distribution  # Replace the 4th column

# 3. Visualize the New Distribution
plt.figure(figsize=(8, 6))
plt.hist(modified_array[:, 3], bins=30, color='lightgreen', edgecolor='black')
plt.title('Modified Even Distribution of the 4th Column')
plt.xlabel('Values')
plt.ylabel('Frequency')
plt.show()

# 4. Save the Modified Array as an .npy file
np.save('modified_input_array.npy', modified_array)
print("Modified array saved as 'modified_input_array.npy'.")