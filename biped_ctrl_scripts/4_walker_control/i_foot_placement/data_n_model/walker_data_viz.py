import numpy as np
import matplotlib.pyplot as plt

# Load arrays
input_array = np.load('input_array_16.npy')
output_array = np.load('output_array_16.npy')

print(input_array[444,:])
print(output_array[444])

exit()

# Calculate stats for the 4th column of input_array
mean_value = np.mean(input_array[:, 3])
max_value = np.max(input_array[:, 3])
min_value = np.min(input_array[:, 3])
median_value = np.median(input_array[:, 3])

print(f"Max: {max_value}, Min: {min_value}, Median: {median_value}, Mean: {mean_value}")

# Create a figure with 2 subplots
fig, axs = plt.subplots(2, 1, figsize=(8, 10))

# Plot the histogram of the 4th column (input_array)
axs[0].hist(input_array[:, 3], bins=30, color='skyblue', edgecolor='black')
axs[0].set_title('Distribution of the 4th Column (input_array)')
axs[0].set_xlabel('Values')
axs[0].set_ylabel('Frequency')
axs[0].axvline(mean_value, color='red', linestyle='dashed', linewidth=1)
axs[0].text(mean_value, axs[0].get_ylim()[1] * 0.9, f'Mean: {mean_value:.2f}', color='red')

# Create the scatter plot to show the correlation between input_array[:, 3] and output_array
axs[1].scatter(input_array[:, 3], output_array, color='green', alpha=0.6, edgecolor='black')
axs[1].set_title('Correlation Between Input Column 4 and Output Array')
axs[1].set_xlabel('Input Array (4th Column)')
axs[1].set_ylabel('Output Array')

# Adjust layout and show the plot
plt.tight_layout()
plt.show()
