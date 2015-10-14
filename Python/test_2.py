import math

d = [1207,1216,1217,1221,1218,1220,1212,1224,1222,1228,1214,1215,1208,1210,1185,1187,1163,1170,1212,1216]

n = 20.0

av_d = float(sum(d)) / n

mu = 100.0 / av_d

var = sum([(100.0 / x - mu)**2 for x in d]) / n

print 'var: ' + str(var)

print 'std: ' + str(math.sqrt(var))