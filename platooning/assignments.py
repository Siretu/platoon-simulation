from constants import F0p, F1p, F0, F1
from pairwise_planning import calculate_default, DefaultPlan
import numpy as np


class Truck:
    def __init__(self, i, path_data_set):
        self.id = i
        self.path = path_data_set['path']
        self.edge_path = np.array([x for x in path_data_set['edge_ids'] if x != 4294967295])
        self.edge_set = set(self.edge_path)
        self.edge_offsets = path_data_set['edge_offsets']
        self.path_set = path_data_set['path_set']
        self.path_weights = path_data_set['path_weights']
        self.path_weights_cum = np.concatenate((np.zeros(1), np.cumsum(path_data_set['path_weights'])))
        self.start_time = path_data_set['t_s']
        self.deadline = path_data_set['arrival_dline']
        self.current_pos = path_data_set['start_pos']

        self.default_plan = calculate_default(path_data_set)
        self.plan = self.default_plan
        self.speed_history = []
        self.speed_history += self.plan.calculate_history(self.start_time, self.deadline)
        self.done = False
        self.current_time = self.start_time
        self.plan_history = [self.plan]
        self.completed_link_distance = 0

    def update(self, current_t):
        # Haven't started yet
        if current_t < self.start_time:
            return
        self.current_time = current_t
        self.current_pos = self.pos_from_total()

    def pos_from_total(self):
        remaining = self.current_distance() - self.completed_link_distance
        total = remaining
        current_i = self.current_pos['i']
        for i, x in enumerate(self.path_weights[current_i:]):
            if remaining >= x:
                remaining -= x
            else:
                self.completed_link_distance += total - remaining
                return {'i': i + current_i, 'x': remaining}

        self.done = True

    def change_plan(self, new_plan, current_time):
        if len(self.speed_history) > 0 and current_time >= self.start_time:
            self.speed_history = [x for x in self.speed_history if x.start_time <= current_time]
            self.speed_history[-1].end_time = current_time
        self.plan_history.append(new_plan)
        self.plan = new_plan
        if current_time < self.start_time: # Calculate from start time instead
            history = self.plan.calculate_history(self.start_time, self.deadline)
            self.speed_history = history
        else:
            history = self.plan.calculate_history(current_time, self.deadline)
            self.speed_history += history

    def current_fuel_consumption(self):
        total = 0
        for i, speed_change in enumerate(self.speed_history):
            if i == len(self.speed_history) - 1:
                end_t = self.deadline
            else:
                end_t = self.speed_history[i+1].start_time
            d = speed_change.speed * (end_t - speed_change.start_time)
            if speed_change.platooning and False:
                fV = F0p + F1p * speed_change.speed
            else:
                fV = F0 + F1 * speed_change.speed
            total += fV * d
        return total

    def current_distance(self, time=None):
        if not time:
            time = self.current_time
        total = 0
        for i, speed_change in enumerate(self.speed_history):
            if speed_change.start_time > time:
                break
            if i == len(self.speed_history) - 1 or time < self.speed_history[i+1].start_time:
                end_t = time
            else:
                end_t = self.speed_history[i+1].start_time
            d = speed_change.speed * (end_t - speed_change.start_time)
            total += d
        return total


class SpeedChange:
    def __init__(self, start_time, speed, platooning=False, end_time=None):
        self.start_time = start_time
        self.speed = speed
        self.platooning = platooning
        self.end_time = end_time

    def __str__(self):
        return "%f m/s at t: %f" % (self.speed, self.start_time)
