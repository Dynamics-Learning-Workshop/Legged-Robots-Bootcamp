import torch
import numpy as np
from sklearn.preprocessing import StandardScaler
import torch.nn as nn

# Load the input array
input_array = np.load('input_array_16.npy')

# Fit the scaler on the entire dataset
scaler = StandardScaler()
scaler.fit(input_array)

# Now, assume you have a single data point (4x1 format)
single_data_point = np.array([0.4, -0.5, 0.0, -3.5], dtype=np.float32)  # Replace with your actual values

# To apply the transformation, ensure the data point is in the correct shape (1, 4)
single_data_point_reshaped = single_data_point.reshape(1, -1)  # Reshape to (1, 4)

# Standardizing the single data point using the fitted scaler
normalized_single_data_point = scaler.transform(single_data_point_reshaped)

# Convert to PyTorch tensor
input_tensor = torch.tensor(normalized_single_data_point, dtype=torch.float32)

# Define the neural network model
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(4, 64)  # Input layer
        self.fc2 = nn.Linear(64, 32)  # Hidden layer
        self.fc3 = nn.Linear(32, 1)   # Output layer

    def forward(self, x):
        x = torch.relu(self.fc1(x))   # Activation function
        x = torch.relu(self.fc2(x))   # Activation function
        x = self.fc3(x)                # Output layer
        return x

# Instantiate the model
model = NeuralNetwork()

# Load the trained model's weights
model.load_state_dict(torch.load('neural_network_model.pth'))

# Move the model to the appropriate device (e.g., GPU if available)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

# Move the input tensor to the same device
input_tensor = input_tensor.to(device)

# Make the prediction
model.eval()  # Set the model to evaluation mode
with torch.no_grad():  # No need to track gradients
    output_tensor = model(input_tensor)  # Forward pass

# Convert output back to NumPy array if needed
output = output_tensor.cpu().numpy()  # Move to CPU and convert to NumPy
print("Predicted output:", output)
