from khepera_functions import *
import math
import matplotlib.pyplot as plt
import numpy as np

s = open_connection()
go(s,2)

l = 0
r = 0

while True:

	old_l = l
	old_r = r

	[l,r] = read_counts(s)

	print "Difference in L: " + str(l - old_l)
	print "Difference in R: " + str(r - old_r)