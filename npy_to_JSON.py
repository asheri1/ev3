import numpy as np
import json

# Step 1: Load the .npy file
array = np.load('Q_table.npy')

# Step 2: Convert NumPy array to a Python data structure
# For a simple array, you can convert it directly to a list
python_data = array.tolist()

# Step 3: Serialize the Python data structure to a JSON string
json_data = json.dumps(python_data)

# Step 4: Write the JSON data to a file
with open('your_file.json', 'w') as json_file:
    json_file.write(json_data)
