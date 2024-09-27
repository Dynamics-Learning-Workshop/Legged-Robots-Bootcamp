import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split

# Check if CUDA is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Load your data
input_array = np.load('modified_input_array.npy')
# input_array[:,3] = input_array[:,3]
output_array = np.load('modified_output_array.npy')


# Convert to PyTorch tensors and move to the appropriate device
X = torch.tensor(input_array, dtype=torch.float32).to(device)
y = torch.tensor(output_array, dtype=torch.float32).view(-1, 1).to(device)  # Reshape if necessary

# Split the data into training and testing sets and transfer to device
X_train, X_test, y_train, y_test = train_test_split(X.cpu(), y.cpu(), test_size=0.2, random_state=42)
X_train, y_train = X_train.to(device), y_train.to(device)
X_test, y_test = X_test.to(device), y_test.to(device)


# Define the neural network model
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(4,64)  # Input layer
        self.fc2 = nn.Linear(64, 32)                 # Hidden layer
        self.fc3 = nn.Linear(32, 1)                  # Output layer

    def forward(self, x):
        x = torch.relu(self.fc1(x))  # Activation function
        x = torch.relu(self.fc2(x))  # Activation function
        x = self.fc3(x)              # Output layer
        return x

# Instantiate the model and move it to the appropriate device
model = NeuralNetwork().to(device)

# Define the loss function and optimizer
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Training the model
num_epochs = 100
for epoch in range(num_epochs):
    model.train()  # Set the model to training mode
    optimizer.zero_grad()  # Clear gradients

    # Forward pass
    outputs = model(X_train)  # Input is already on the device
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
    test_outputs = model(X_test)  # Input is already on the device
    test_loss = criterion(test_outputs, y_test)

print(f'Test Loss: {test_loss.item():.4f}')

# Save the model
torch.save(model.state_dict(), 'neural_network_model.pth')
print("Model saved successfully.")
