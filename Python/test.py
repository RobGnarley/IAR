from khepera_functions import *
import math
import matplotlib.pyplot as plt
import numpy as np

s = open_connection()
set_counts(s, 0, 0)
go(s,1)

while True:

	encoders = read_counts(s)

	print encoders
