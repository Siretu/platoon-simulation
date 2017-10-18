def average_plan_length(assignments, all=True):
    averages = [(x.plan_history[-1].end_time - x.plan_history[0].start_time) / len(x.plan_history) for x in assignments if all or len(x.plan_history) > 1]
    return sum(averages) / len(averages)


def average_platoon_time(assignments):
    platoon_information = [average_truck_platoon_time(truck) for truck in assignments]
    return sum([x[0] for x in platoon_information]) / sum([x[1] for x in platoon_information])


def average_truck_platoon_time(truck):
    platooning = sum([x.end_time - x.start_time for x in truck.speed_history if x.platooning >= 0])
    total = sum([x.end_time - x.start_time for x in truck.speed_history])
    return [platooning, total]