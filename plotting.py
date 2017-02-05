# -*- coding: utf-8 -*-


import numpy as np
import matplotlib.pyplot as plt
import cPickle as pkl
import os



def plot_results(results):
  
  f_relat_before_convex_gr = []
  f_relat_before_convex_ra = []
  f_relat_after_convex_gr = []
  f_relat_after_convex_ra = []
  upper_bound = []
  Ks = list(results)
  Ks.sort()
   
  for K in Ks:
    f_relat_before_convex_gr.append(results[K]['f_relat_before_convex_gr'])
    f_relat_before_convex_ra.append(results[K]['f_relat_before_convex_ra'])
    f_relat_after_convex_gr.append(results[K]['f_relat_after_convex_gr'])
    f_relat_after_convex_ra.append(results[K]['f_relat_after_convex_ra'])
    upper_bound.append(results[K]['upper_bound'])
    
  fig, ax1 = plt.subplots()
  ax1.plot(Ks,f_relat_before_convex_gr,'r-+')
  ax1.plot(Ks,f_relat_before_convex_ra,'r--+')
  ax1.plot(Ks,f_relat_after_convex_gr,'b-+')
  ax1.plot(Ks,f_relat_after_convex_ra,'b--+')
  ax1.plot(Ks,upper_bound,'k-+')
  
  plt.show(block=False)
  
  return
  
def process_folder(folder):
  # takes one folder and accumulates all the data  
  # loading data

  Ks = [100, 200, 300, 500, 1000, 1500, 2000, 3000]

  simulations = os.listdir(folder)
  results = []

  for sim in simulations:
    if not os.path.isdir('{}{}'.format(folder,sim)):
      f = open('{}{}'.format(folder,sim),'r')
      results.append(pkl.load(f))
      f.close()  
  
  av_results = {K:{} for K in Ks}  
  
  for K in Ks:
    av_results[K]['f_relat_before_convex_gr'] = np.mean([result[K]['f_relat_before_convex_gr'] for result in results])
    av_results[K]['f_relat_before_convex_ra'] = np.mean([result[K]['f_relat_before_convex_ra'] for result in results])
    av_results[K]['f_relat_after_convex_gr'] = np.mean([result[K]['f_relat_after_convex_gr'] for result in results])
    av_results[K]['f_relat_after_convex_ra'] = np.mean([result[K]['f_relat_after_convex_ra'] for result in results])
    av_results[K]['upper_bound'] = np.mean([result[K]['upper_bound']/result[K]['f_total_default'] for result in results])
    #TODO: replace later by upper_bound_relat    
    
    
    # process size stats
    av_stats = {}
    for result in results:
      size_stats = result[K]['size_stats_gr']
      for size in size_stats:
        if size in av_stats:
          av_stats[size].append(size_stats[size])
        else:
          av_stats[size] = [size_stats[size]]
    for size in av_stats:
      av_stats[size] = np.mean(av_stats[size])
    av_results[K]['size_stats_gr'] = av_stats
    
    av_stats = {}
    for result in results:
      size_stats = result[K]['size_stats_ra']
      for size in size_stats:
        if size in av_stats:
          av_stats[size].append(size_stats[size])
        else:
          av_stats[size] = [size_stats[size]]
    for size in av_stats:
      av_stats[size] = np.mean(av_stats[size])
    av_results[K]['size_stats_ra'] = av_stats
  
  
  return av_results

  
#f = open('./testsimres/1454926330.31.pkl','r')
#results = pkl.load(f)
#f.close()
#  
#plot_results(results)

folder = './testsimres/'
av_results = process_folder(folder)

plot_results(av_results)