V_MAX = 90. / 3.6
V_MIN = 70. / 3.6
V_NOM = 80. / 3.6
MIN_INTERSECTION_LENGTH = 5e3

# problem_data['F'] = {'F0':1, 'F0p':0.9, 'F1':1./80, 'F1p':1./80*.9} # fuel model
# problem_data['F'] = {'F0':0., 'F0p':1.-eta, 'F1':2./80, 'F1p':2./80*eta} # fuel model

# everything in SI
c_0D = .6
c_PD = .4
m = 4e4
c_r = 7e-3
A = 10.

p_0 = 5.3628e-4
p_1 = 5.1526e-8

g = 9.81
rho = 1.225

# f = (Fm1/v + F0 + F2*v**2)*d
bar_Fm1 = p_0
bar_F0 = c_r * m * g * p_1
bar_F2 = .5 * rho * A * c_0D * p_1
bar_FP2 = .5 * rho * A * c_PD * p_1

eta = 0.6

# todo: fill in the calculations for F0,...

# v_nom = problem_data['v_nom']/3.6
F0 = bar_F0 - bar_F2 * V_NOM ** 2
F1 = 2 * bar_F2 * V_NOM
F0p = bar_F0 - eta * bar_F2 * V_NOM ** 2
F1p = 2 * eta * bar_F2 * V_NOM
# adapt to km and h
# F0 = F0*1000. # m --> km
# F0p = F0p*1000.
# F1 = F1*1000./3.6 # m/s --> km/h
# F1p = F1p*1000./3.6

# Savings with this fuel model at v_nom 0.15913792711482916 (F0+F1*v_nom - F0p - F1p*v_nom)/(F0 + F1*v_nom)


# K = 3
# problem_data['K'] = [0,1,3] #range(K) # trucks
# active_trucks = [0,1,3] #range(K)

EPS = 1e-6  # threshold to consider to positions coinciding

START_INTERVAL = 48. * 3600

# time gap for spontaneous platooning
TIME_GAP = 60.  # one minute

LEADER = -1
NONE = -2