import numpy as np
import matplotlib.pyplot as plt

# Load the original arrays
input_array = np.load('input_array_16.npy')
output_array = np.load('output_array_16.npy')

# 1. Visualize the Original Distribution of the 4th column in the input array
plt.figure(figsize=(8, 6))
plt.hist(input_array[:, 3], bins=30, color='skyblue', edgecolor='black')
plt.title('Original Distribution of the 4th Column (Input Array)')
plt.xlabel('Values')
plt.ylabel('Frequency')
plt.show()

# 2. Resample to create a more even distribution for the 4th column in the input array
min_val = np.min(input_array[:, 3])
max_val = np.max(input_array[:, 3])

# Create a new array with values uniformly distributed between min and max of original data
even_distribution = np.random.uniform(low=min_val, high=max_val, size=len(input_array[:, 3]))

# Get the sorted indices of the new even distribution
sorted_indices = np.argsort(even_distribution)

# Replace the original 4th column with the new evenly distributed data
modified_input_array = input_array.copy()  # Create a copy of the original input array
modified_input_array[:, 3] = even_distribution  # Replace the 4th column

# Sort the output array according to the sorted indices of the new input distribution
modified_output_array = output_array.copy()
modified_output_array = output_array[sorted_indices]

# 3. Visualize the New Distribution for the 4th column in the modified input array
plt.figure(figsize=(8, 6))
plt.hist(modified_input_array[:, 3], bins=30, color='lightgreen', edgecolor='black')
plt.title('Modified Even Distribution of the 4th Column (Input Array)')
plt.xlabel('Values')
plt.ylabel('Frequency')
plt.show()

# 4. Save the Modified Arrays as .npy files
np.save('modified_input_array.npy', modified_input_array)
np.save('modified_output_array.npy', modified_output_array)
print("Modified input and output arrays saved as 'modified_input_array.npy' and 'modified_output_array.npy'.")
