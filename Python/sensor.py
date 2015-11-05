# Find best fitting sensor model

import numpy as np

def sensor(x,y):
	"""
	Use least squares to fit line y = Ae^(kx)
	"""

	w = np.log(y)
	X = np.vstack([x, np.ones(len(x))]).T

	m , c = np.linalg.lstsq(X, w)[0]

	A = np.exp(c)

	k = m

	return A, k
