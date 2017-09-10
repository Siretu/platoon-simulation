# -*- coding: utf-8 -*-


import cPickle as pkl
import httplib
import os.path
import random
import time

import numpy as np
import scipy.ndimage
import simplejson as json

num_routes = 1000
min_route_length = 100 * 1000
max_route_length = 400 * 1000  # in meters
osrm_url = 'simserver.now.im:5001'

density_map = scipy.ndimage.imread(os.path.join(os.path.dirname(__file__), 'sedac_sweden_only.png'), mode='L')
resolution = 0.1  # .1 deg/pixel
# offset_left = 1690
# offset_top = 250
height, width = density_map.shape
density_map[density_map == 255] = 0
density_map[density_map < 120] = 0
coordinate_precision = 1000000.


def pixel_to_latlon(x, y):
    lat = 90 - resolution * y
    lon = -180 + resolution * x
    return lat, lon


def latlon_to_pixel(lat, lon, coor_prec):
    # assumes to get latlon in degrees multiplied with coordinate_precision

    coor_prec = float(coor_prec)
    y = (90 - lat / coor_prec) / resolution
    x = (180 + lon / coor_prec) / resolution
    return int(x), int(y)


prob_dist = 1. * density_map.flatten() / np.sum(density_map.flatten())
cs_prob_dist = np.cumsum(prob_dist)


def sample_start_goal(seed):
    random.seed(seed)
    rand_pointer = random.random()
    #  start_index = np.where(cs_prob_dist >= rand_pointer)[0][0]
    #  start_index = np.argmax(cs_prob_dist >= rand_pointer)
    start_index = np.searchsorted(cs_prob_dist, rand_pointer)
    #  if alt_start_index == start_index: print 'does match'
    start_y = start_index / width
    start_x = np.mod(start_index, width)
    start_lat, start_lon = pixel_to_latlon(start_x, start_y)
    start_loc = start_lat, start_lon

    rand_pointer = random.random()
    #  goal_index = np.where(cs_prob_dist >= rand_pointer)[0][0]
    goal_index = np.searchsorted(cs_prob_dist, rand_pointer)
    goal_y = goal_index / width
    goal_x = np.mod(goal_index, width)
    goal_lat, goal_lon = pixel_to_latlon(goal_x, goal_y)
    goal_loc = goal_lat, goal_lon

    #  print start_loc, goal_loc

    return start_loc, goal_loc


def calc_route_retry(seed, verbose=False):
    start_loc, goal_loc = sample_start_goal(seed)  # use the sequence number as the seed
    if start_loc == goal_loc:
        if verbose:
            print "start and goal are the same, abort"
        return False

    route = calc_route2(start_loc, goal_loc, verbose)

    rt = 1.
    rts = 1.
    # retries, they search in increasing distance in each direction
    while not route:
        # try again with slightly different coordinates
        start_loc = (start_loc[0] + .01 * rt * rts, start_loc[1] + .01 * rt * rts)
        goal_loc = (goal_loc[0] + .01 * rt * rts, goal_loc[1] + .01 * rt * rts)
        route = calc_route2(start_loc, goal_loc, verbose)
        rt += 1.
        rts *= -1
        if rt == 100:
            break
    if route:
        return route
    else:
        print 'could not find a route even for the altered coordinates between {} and {}'.format(start_loc, goal_loc)
        return False


def calc_route2(start, goal, verbose=False):
    # start = [64.896553, 20.373343]
    # goal = [57.637585, 13.076223]
    request_template = '/route/v1/driving/{},{};{},{}?overview=false&generate_hints=false'
    request = request_template.format(start[1], start[0], goal[1], goal[0])
    conn = httplib.HTTPConnection(osrm_url)
    conn.request("GET", request)
    r1 = conn.getresponse()
    result = json.loads(r1.read())
    conn.close()
    if 'routes' not in result:
        return
    result = result['routes'][0]
    result['link_lengths'] = np.array(result['distances'])
    result['node_coords_lat'] = np.array(result['lats'])
    result['node_coords_lon'] = np.array(result['lons'])
    result['node_ids'] = np.array(result['node_ids'])

    return result


def calc_route(start_loc, goal_loc, verbose=False):
    conn = httplib.HTTPConnection(osrm_url)

    conn.request("GET", "/locate?loc={:.6f},{:.6f}".format(*start_loc))
    sl_json = conn.getresponse()
    sl = json.load(sl_json)
    if sl['status'] == 0:
        start_loc = sl["mapped_coordinate"]
    else:
        if verbose:
            print "Could not locate a node close to the start location {:.6f},{:.6f}".format(*start_loc)
        conn.close()
        return False

    conn.request("GET", "/locate?loc={:.6f},{:.6f}".format(*goal_loc))
    gl_json = conn.getresponse()
    gl = json.load(gl_json)
    if gl['status'] == 0:
        goal_loc = gl["mapped_coordinate"]
    else:
        if verbose:
            print "Could not locate a node close to the goal location {:.6f},{:.6f}".format(*goal_loc)
        conn.close()
        return False

    if verbose:
        print "GET", "/viaroute?loc={:.6f},{:.6f}&loc={:.6f},{:.6f}&geometry=false".format(*(start_loc + goal_loc))
    conn.request("GET", "/viaroute?loc={:.6f},{:.6f}&loc={:.6f},{:.6f}&geometry=false".format(*(start_loc + goal_loc)))
    r1 = conn.getresponse()
    route = json.load(r1)
    if route['status'] == 207:
        if verbose:
            print "no route found between {} and {}".format(start_loc, goal_loc)
        conn.close()
        return False

    conn.close()

    # filter start, end, and geometry only links

    segment_durations = np.array(route['segment_durations'])
    real_edge_indices = np.where(segment_durations > 0)[0]

    if real_edge_indices.shape[0] < 2:
        return False

    # extract relevant data and convert to np arrays
    offset = -1
    link_lengths = np.array(route['link_lengths'], dtype='float')
    if np.sum(link_lengths) < min_route_length:
        return False
    compr_link_lengths = [np.sum(link_lengths[0:real_edge_indices[0] + 1])]
    for i in xrange(real_edge_indices.shape[0] - 1):
        compr_link_lengths.append(
            np.sum(link_lengths[real_edge_indices[i] + 1 + offset:real_edge_indices[i + 1] + 1 + offset]))
    compr_link_lengths = np.array(compr_link_lengths, dtype='float')
    compr_link_lengths = compr_link_lengths[1:]

    if np.sum(compr_link_lengths) > max_route_length:  # prune the route if too long
        ll_cum = np.cumsum(compr_link_lengths)
        fits = np.nonzero(ll_cum < ll_cum[-1] - max_route_length)[0]
        if fits.shape[0] > 0:
            last = fits[-1] + 1  # last element to start that fits the route
            start = np.random.randint(1, last + 1)
            end = np.nonzero(ll_cum > ll_cum[start - 1] + max_route_length)[0][0] - 1  # last link (inclusive)
        else:
            start = 0
            end = compr_link_lengths.shape[0]
    else:
        start = 0
        end = link_lengths.shape[0] - 1

    compr_link_lengths = compr_link_lengths[start:end + 1]
    node_ids = np.array(route['unpacked_IDs'], dtype='uint64')
    node_ids = node_ids[start + 1:end + 2]
    node_coords_lat = np.array(route['node_coords_lat'], dtype='int')[real_edge_indices]
    node_coords_lat = node_coords_lat[start + 1:end + 2]
    node_coords_lon = np.array(route['node_coords_lon'], dtype='int')[real_edge_indices]
    node_coords_lon = node_coords_lon[start + 1:end + 2]

    processed_route = {'node_ids': node_ids, 'link_lengths': compr_link_lengths, 'node_coords_lat': node_coords_lat,
                       'node_coords_lon': node_coords_lon}

    #  compr_segment_durations = segment_durations[real_edge_indices]
    #  speeds = compr_link_lengths[:]/compr_segment_durations[:]*36.

    # skipping length restriction for now
    #  ll_cum = np.cumsum(link_lengths)
    #  if ll_cum[-1] > max_route_length:
    #    fits = np.nonzero(ll_cum < ll_cum[-1] - max_route_length)[0]
    #    if fits.shape[0] > 0:
    #      last = fits[-1]+1 # last element to start that fits the route
    #      start = np.random.randint(1, last+1)
    #      end = np.nonzero(ll_cum > ll_cum[start-1] + max_route_length)[0][0]-1 # last link (inclusive)
    #    else:
    #      start = 0
    #      end = link_lengths.shape[0]
    #    # select random part
    #
    ##      print start
    ##      print np.sum(link_lengths[start:])
    #
    #  else:
    #    start = 0
    #    end = link_lengths.shape[0]-1
    #
    #  link_lengths = link_lengths[start:end+1]
    #  node_ids = np.array(route['node_ids'][start:end+1],dtype='uint64')
    #  node_coords_lat = np.array(route['node_coords_lat'][start:end+1],dtype='int')
    #  node_coords_lon = np.array(route['node_coords_lon'][start:end+1],dtype='int')
    #  route = {'node_ids':node_ids,'link_lengths':link_lengths,'node_coords_lat':node_coords_lat,'node_coords_lon':node_coords_lon}

    #  node_coords_lat = np.array(route['node_coords_lat'],dtype='int')
    #  node_coords_lon = np.array(route['node_coords_lon'],dtype='int')
    #  plot_routes_in_density_map([{'node_coords_lat':node_coords_lat,'node_coords_lon':node_coords_lon}])
    return processed_route


# def plot_routes_in_density_map(routes):
#     density_map_rgb = density_map.reshape(height, width, 1).repeat(3, 2)
#     for route in routes:
#         lat = route['node_coords_lat']
#         lon = route['node_coords_lon']
#         x, y = latlon_to_pixel(lat, lon, coordinate_precision)
#         for i in xrange(y.shape[0]):
#             if x[i] < width and y[i] < height:
#                 density_map_rgb[y[i], x[i], :] = [255, 0, 0]
#         density_map_rgb[y[0], x[0], :] = [0, 255, 0]
#         density_map_rgb[y[-1], x[-1], :] = [0, 0, 255]
#
#     plt.figure()
#     plt.imshow(density_map_rgb, interpolation="nearest")
#     plt.show(block=False)
#     return


def routes_to_pkl():
    import gc
    seed = 0  # random.randint(0,1e6)
    start_t = time.time()
    for i in xrange(seed, seed + num_routes):
        # check if the file for that route already exists
        if not os.path.isfile('./testroutes/{}.pkl'.format(i)):
            # This is probably an error
            route = calc_route(i)
            gc.collect()
            if route:
                try:
                    f = open('./testroutes/{}.pkl'.format(i), 'w')
                    pkl.dump(route, f, protocol=pkl.HIGHEST_PROTOCOL)
                    f.close()
                except:
                    print "could not save the result"
    print "calculation took {} seconds".format(time.time() - start_t)
    print "that is {} routes/second".format(num_routes / (time.time() - start_t))
    return

# routes_to_pkl()
# routes = [r for r in routes if 'status' not in r]
#
# plot_routes_in_density_map(routes)
