def average_plan_length(assignments, all=True):
    averages = [(x.plan_history[-1].end_time - x.plan_history[0].start_time) / len(x.plan_history) for x in assignments if all or len(x.plan_history) > 1]
    return sum(averages) / len(averages)