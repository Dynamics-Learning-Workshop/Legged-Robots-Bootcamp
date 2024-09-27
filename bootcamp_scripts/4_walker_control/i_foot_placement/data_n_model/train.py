import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split

# Load your data
input_array = np.load('input_array_16.npy')
output_array = np.load('output_array_16.npy')

# Standardizing the input data
scaler = StandardScaler()
normalized_input_array = scaler.fit_transform(input_array)

# Convert to PyTorch tensors
X = torch.tensor(normalized_input_array, dtype=torch.float32)
y = torch.tensor(output_array, dtype=torch.float32).view(-1, 1)  # Reshape if necessary

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Define the neural network model
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(X_train.shape[1], 64)  # Input layer
        self.fc2 = nn.Linear(64, 32)                 # Hidden layer
        self.fc3 = nn.Linear(32, 1)                  # Output layer

    def forward(self, x):
        x = torch.relu(self.fc1(x))  # Activation function
        x = torch.relu(self.fc2(x))  # Activation function
        x = self.fc3(x)              # Output layer
        return x

# Instantiate the model
model = NeuralNetwork()

# Define the loss function and optimizer
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Training the model
num_epochs = 100
for epoch in range(num_epochs):
    model.train()  # Set the model to training mode
    optimizer.zero_grad()  # Clear gradients

    # Forward pass
    outputs = model(X_train)
    loss = criterion(outputs, y_train)

    # Backward pass and optimization
    loss.backward()
    optimizer.step()

    # Print loss for every 10 epochs
    if (epoch+1) % 10 == 0:
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')

# Evaluate the model
model.eval()  # Set the model to evaluation mode
with torch.no_grad():
    test_outputs = model(X_test)
    test_loss = criterion(test_outputs, y_test)

print(f'Test Loss: {test_loss.item():.4f}')

# Save the model
torch.save(model.state_dict(), 'neural_network_model.pth')
print("Model saved successfully.")
