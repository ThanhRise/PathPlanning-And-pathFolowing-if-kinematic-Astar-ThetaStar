import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm

# Generate a sample grid environment
grid_size = 15
obstacle_positions1 = [(2, 10), (3, 9), (4, 8), (5, 7), (6, 6), (8, 7), (9, 6), (10,5), (11, 4), (12, 3)]
obstacle_positions2 = [(4, 10), (5, 9), (6, 8), (7, 7), (8, 6), (9, 5), (10,4), (11, 3), (12, 2), (13, 1)]
goal_position = (8, 0)
start_position = (0, 8)

# Create a grid
grid = np.zeros((grid_size, grid_size))

# Set obstacle positions in the grid
for obstacle in obstacle_positions1:
    grid[obstacle] = 1
for obstacle in obstacle_positions2:
    grid[obstacle] = 1

# Set goal position in the grid
grid[goal_position] = 2

# Set start position in the grid
grid[start_position] = 3


# using svm to find hyperplane between two types of obstacles

# non-linearly svm classifier
clf = svm.SVC(kernel='rbf', C=1000)


# training data
X = []
y = []
for obstacle in obstacle_positions1:
    X.append(obstacle)
    y.append(1)
for obstacle in obstacle_positions2:
    X.append(obstacle)
    y.append(2)
clf.fit(X, y)

# plot the hyperplane
# create grid to evaluate model
xx = np.linspace(0, 15, 150)
yy = np.linspace(0, 15, 150)
YY, XX = np.meshgrid(yy, xx)
xy = np.vstack([XX.ravel(), YY.ravel()]).T
Z = clf.decision_function(xy).reshape(XX.shape)
# plot decision boundary and margins
plt.contour(XX, YY, Z, colors='k', levels=[-1, 0, 1], alpha=0.5,
            linestyles=['--', '-', '--'])
# plot support vectors
plt.scatter(clf.support_vectors_[:, 0], clf.support_vectors_[:, 1], s=100,
            linewidth=1, facecolors='none', edgecolors='k')
# plot obstacles
for obstacle in obstacle_positions1:
    plt.scatter(obstacle[0], obstacle[1], s=100, color='red')
for obstacle in obstacle_positions2:
    plt.scatter(obstacle[0], obstacle[1], s=100, color='blue')

plt.show()


