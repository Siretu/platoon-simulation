# -*- coding: utf-8 -*-

import random
import pairwise_planning as pp
import clusteralg as cl
import convex_optimization as cv
import route_calculation as rc
import copy
import numpy as np
import sample_routes as sr
import matplotlib.pyplot as plt
import cPickle as pkl
import time

# units a in km and h

#cd /mnt/data/Uni/PHD/truck_platoon_planning/osrm/osrm_backend_edge_ids/build/
#./osrm-routed sweden-latest.osrm


def build_path_data_sets(problem_data, route_links, route_weights, route_lat, route_lon, start_times, arrival_dlines, active_trucks, start_poses=None):
  
  path_data_sets = {}
  for k in active_trucks:
    path_data = {}
    path_data['path'] = route_links[k]
    path_data['path_set'] = set(path_data['path'])    
    path_data['path_weights'] = route_weights[k]
    if start_poses != None:    
      path_data['start_pos'] = start_poses[k]
    else:
      path_data['start_pos'] = {'i': 0, 'x': 0.}
    path_data['t_s'] = start_times[k]
    path_data['arrival_dline'] = arrival_dlines[k]
    path_data_sets[k] = path_data
  
  return path_data_sets

def plot(K):
  
  # %%%%%%%%%%%% Generate Routes and start times / deadlines
  print 'retrieving the routes'  
  
  route_links, route_weights, K_set, routes = rc.get_routes_from_osrm(K, True)
#  route_links, route_weights, K_set, routes = rc.get_routes_from_pkl('./testroutes100/')
  

  #debug: give all trucks the same route
  #
  #route_links = {k:route_links[K_set[0]] for k in K_set}
  #route_weights = {k:route_weights[K_set[0]] for k in K_set}
  
  
  problem_data['K'] = copy.copy(K_set)
  active_trucks = copy.copy(K_set)
  
  
  # test data, will be generated from osrm
  #route_links = {0:[1,2,3,4,5], 1:[12,13,3,4,5], 3:[21,22,23,1,2,3,24]}
  #route_weights = {0:[5.5,5.,6.,4.,8.], 1:[5.,7.,6.,4.,8.], 3:[7.,2.,3.,5.5,5.,6.,100.]}
  # TODO: move into problem data ???
  
  
  # TODO: might need to change this to dicts because of truck dropping out
  
  start_times = {k:random.random()*start_interval for k in problem_data['K']}
  #start_times = {0:0.14,1:0.14,3:0.01}
  arrival_dlines = {k:start_times[k] + sum(route_weights[k])/problem_data['v_nom'] for k in problem_data['K']}
  
  # %%%%%%%%%%%% loop until all trucks have arrived
  
  # %%%%%% build coordination graph
  print 'building coordination graph'
  path_data_sets = build_path_data_sets(problem_data, route_links, route_weights, route_lat, route_lon, start_times, arrival_dlines, active_trucks)
  
  default_plans = pp.get_default_plans(problem_data,path_data_sets)
  
  G_p = pp.build_G_p(problem_data,path_data_sets,default_plans)
  
  N_f_gr,N_l_gr,leaders_gr,counter_gr = cl.clustering(G_p,'greedy')
  # create adjecancy matrix

  adj_matrix = np.ones((K,K,3))
  Klist = list(K_set)
  Klist.sort()
  for i in G_p:
    for j in G_p[i]:
      ii = Klist.index(i)
      jj = Klist.index(j)
      if G_p[i][j] > 0: 
        if leaders_gr[i] == j:
          adj_matrix[ii,jj,:] = np.array([1.,0.,0.])
        else:
          adj_matrix[ii,jj,:] = np.array([0.,0.,0.])
  
  plt.figure()
  plt.imshow(adj_matrix, interpolation="nearest")
  plt.show(block=False)
  
  return



# %%%%%%%%%%%% Definition of Problem Data

problem_data = {}
problem_data['v_max'] = 90./3.6
problem_data['v_min'] = 70./3.6
problem_data['v_nom'] = 80./3.6
problem_data['min_intersection_length'] = 5e3

#problem_data['F'] = {'F0':1, 'F0p':0.9, 'F1':1./80, 'F1p':1./80*.9} # fuel model
#problem_data['F'] = {'F0':0., 'F0p':1.-eta, 'F1':2./80, 'F1p':2./80*eta} # fuel model

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
bar_F0 =c_r*m*g*p_1
bar_F2 = .5*rho*A*c_0D*p_1
bar_FP2 = .5*rho*A*c_PD*p_1

eta = 0.6

# todo: fill in the calculations for F0,...

#v_nom = problem_data['v_nom']/3.6
v_nom = problem_data['v_nom']
F0 = bar_F0 - bar_F2*v_nom**2
F1 = 2*bar_F2*v_nom
F0p = bar_F0 - eta*bar_F2*v_nom**2
F1p = 2*eta*bar_F2*v_nom
# adapt to km and h
#F0 = F0*1000. # m --> km
#F0p = F0p*1000.
#F1 = F1*1000./3.6 # m/s --> km/h
#F1p = F1p*1000./3.6

problem_data['F'] = {'F0':F0, 'F0p':F0p, 'F1':F1, 'F1p':F1p} # fuel model

# Savings with this fuel model at v_nom 0.15913792711482916 (F0+F1*v_nom - F0p - F1p*v_nom)/(F0 + F1*v_nom)


#K = 3
#problem_data['K'] = [0,1,3] #range(K) # trucks
#active_trucks = [0,1,3] #range(K)

problem_data['eps'] = 1e-6 # threshold to consider to positions coinciding

start_interval = 2.*3600

# time gap for spontanous platooning
time_gap = 60. # one minute



#folder = './testroutes/'
#route_links, route_weights, K_set, routes = rc.get_routes_from_pkl(folder)

K = 500

plot(K)
    
#f = open('./testsimres/{}.pkl'.format(time.time()),'w')
#pkl.dump(results,f)
#f.close()
#  
#plot_results(results)