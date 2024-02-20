import numpy as np

# Load the Q-table
Q_table = np.load('q_table.npy')

# Save the Q-table to a text file
with open('a.txt', 'w') as f:
    for row in Q_table:
        np.savetxt(f, row.reshape(1, row.size), fmt='%s')
f.close()

