# -*- coding: utf-8 -*-

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

import json
import numpy as np
import matplotlib.pyplot as plt
import map_plotter as mp
 
# Configuration
from route_calculation import get_routes, get_path_data_sets


def plot_map(folder):
    # routes = get_routes(folder)
    paths = get_path_data_sets(folder)

    map_type = 'light-v9'

    xref = 800
    yref = 600

    scale_config = {'offset_x':20,'offset_y':20,
                    'text_offset_x':20,'text_offset_y':25,'real_length':100e3}

    # if you want to use default parameters, uncomment
    #scale_config = {}

    with open('api_token.txt','r') as f_token:
      api_token = f_token.readline().rstrip() # doesn't work rstrip(), probably EOF that creates trouble

    lat = [np.array([y/1000000.0+0 for y in x['node_coords_lat']]) for x in paths.values()]
    lon = [np.array([y/1000000.0+0 for y in x['node_coords_lon']]) for x in paths.values()]

    lat_min = min([min(x) for x in lat])
    lat_max = max([max(x) for x in lat])
    # lat_max = 59.35
    lon_min = min([min(x) for x in lon])
    lon_max = max([max(x) for x in lon])
    # with open('./testdata.json', 'r') as f:
    #     data = json.load(f)
    #
    # lat = np.array(data['lat'])
    # lon = np.array(data['lon'])
    # Create the map configuration
    map_config = mp.get_map_config(xref,yref,lat_min, lat_max, lon_min, lon_max)
    # map_config = mp.get_map_config(xref, yref, lat.min(), lat.max(), lon.min(), lon.max())

    # Create a matplotlib figure object with the map
    fig = mp.get_figure(api_token,map_type,map_config,scale_config)
    # To get rid of the scale use instead
    #fig = mp.get_figure(api_token,map_type,map_config)

    # This function tranforms the data so it can be plotted on fig with the regular
    # matplotlib plotting functions
    for x in range(len(lat)):
        x,y = mp.transform_data(lat[x],lon[x],map_config)
        plt.gca().set_color_cycle(None)
        fig.gca().set_color_cycle(None)
        fig.gca().plot(x,y, lw=0.5)

    plt.show()

if __name__ == "__main__":
    plot_map("./testing/testroutes/test400-1/")