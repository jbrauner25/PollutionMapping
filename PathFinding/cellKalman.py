
#Calculates the measured value variance for a given point given
#the variance is linear with respect to distance'''
def meas_var_dist(distance):
    var = (1/5)*distance
    return var

#Performs the Kalman Filter algorithm for 1 cycle, returns the posteri estimate
#	and variance for the next cycle'''
def kalman_filter(node_pol, node_var, meas_pol, meas_var):
    if node_var is None:
        return meas_pol, meas_var
    kg = node_var / (node_var + meas_var)
    new_pol = float(node_pol) + kg * (float(meas_pol) - float(node_pol))
    new_var = (1 - kg) * node_var
    return new_pol, new_var
