from constants import F0p, F1p, F0, F1
from pairwise_planning import calculate_default, DefaultPlan


class Truck:
    def __init__(self, i, path_data_set):
        self.id = i
        self.path = path_data_set['path']
        self.path_set = path_data_set['path_set']
        self.path_weights = path_data_set['path_weights']
        self.start_time = path_data_set['t_s']
        self.deadline = path_data_set['arrival_dline']
        self.current_pos = path_data_set['start_pos']
        self.total_pos = 0

        self.default_plan = calculate_default(path_data_set)
        self.plan = self.default_plan
        self.speed_history = []
        self.speed_history += self.plan.calculate_history(self.start_time, self.deadline)
        self.done = False
        self.current_time = -1
        self.plan_history = [self.plan]

    def update(self, previous_t, current_t):
        self.current_time = current_t
        self.total_pos += current_t - previous_t
        self.current_pos = self.pos_from_total()

    # This is incorrect. It calculates position based on time instead of based on distance
    def pos_from_total(self):
        remaining = self.current_distance(self.total_pos)
        for i, x in enumerate(self.path_weights):
            if remaining >= x:
                remaining -= x
            else:
                return {'i': i, 'x': remaining}

        self.done = True

    def change_plan(self, new_plan, current_time):
        if len(self.speed_history) > 0:
            self.speed_history = [x for x in self.speed_history if x.start_time <= current_time]
            self.speed_history[-1].end_time = current_time

        self.plan_history.append(new_plan)
        self.plan = new_plan
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
            if speed_change.platooning:
                fV = F0p + F1p * speed_change.speed
            else:
                fV = F0 + F1 * speed_change.speed
            total += fV * d
        return total

    def current_distance(self, final_time):
        total = 0
        for i, speed_change in enumerate(self.speed_history):
            if i == len(self.speed_history) - 1:
                end_t = final_time
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
