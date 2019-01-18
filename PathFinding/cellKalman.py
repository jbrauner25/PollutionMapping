
#Calculates the measured value variance for a given point given
#the variance is linear with respect to distance'''
def meas_var_dist(distance):
    var = (1/5)*distance
    return var

#Performs the Kalman Filter algorithm for 1 cycle, returns the posteri estimate
#	and variance for the next cycle'''
def kalman_filter(priori_est, priori_estVar, meas_val, meas_valVar):
    if priori_estVar is None:
        return meas_val, meas_valVar
    KG = priori_estVar/(priori_estVar + meas_valVar)
    posteri_est = priori_est + KG*(meas_val - priori_est)
    posteri_estVar = (1 - KG) * priori_estVar
    return posteri_est, posteri_estVar
