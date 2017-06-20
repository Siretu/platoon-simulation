from pairwise_planning import calculate_default, PlatoonPlan


class Truck:
    def __init__(self, i, path_data_set):
        self.id = i
        self.path = path_data_set['path']
        self.path_set = path_data_set['path_set']
        self.path_weights = path_data_set['path_weights']
        self.start_time = path_data_set['t_s']
        self.deadline = path_data_set['arrival_dline']
        self.start_pos = path_data_set['start_pos']

        t_a, v_default, f = calculate_default(path_data_set)
        self.default_plan = PlatoonPlan(f, None, None, None, None, None, t_a)
        self.default_plan.type = 'default'
        self.default_plan.speed = v_default