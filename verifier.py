import numpy as np

# Verify that the speed history allows the truck to make it to the end by the deadline
def verify_deadline(truck):
    total = 0
    for change in truck.speed_history:
        total += change.speed * (change.end_time - change.start_time)
    if total - truck.path_weights_cum[-1] < -0.001:
        print "Error: Speed history does not reach end"
    if total - truck.path_weights_cum[-1] > 0.001:
        print "Error: Speed history goes past end"


# Make sure that speed_history is consistent and continuous
def verify_speed_history(truck):
    history = truck.speed_history
    if history[0].start_time != truck.start_time:
        print "Error: Speed history start time mismatch"
    if history[-1].end_time > truck.deadline + 0.1:
        print "Error: Speed history end time mismatch"
    if history[-1].end_time != truck.plan.arrival_time:
        print "Error: Arrival time mismatch"

    for i, change in enumerate(history[1:]):
        if change.start_time < history[i].end_time:
            print "Error: Overlap in speed history"
        elif change.start_time > history[i].end_time:
            print "Error: Gap in speed history"


def verify_platooning(truck, assignments):
    for change in truck.speed_history:
        if change.platooning != -1:
            leader = assignments[change.platooning]
            if leader.is_platoon_follower(change.start_time) or leader.is_platoon_follower(change.end_time):
                print "Error: platoon leader is simultaneously a platoon follower"
            if not is_platooning(truck, leader, change):
                print "Error: Platooning mismatch"


def is_platooning(follower, leader, change):
    follower_s = follower.link_pos(follower.current_distance(change.start_time+1))['i']
    follower_platoon_start = follower.path[follower_s]
    follower_e = follower.link_pos(follower.current_distance(change.end_time-1))['i']
    follower_platoon_end = follower.path[follower_e]
    leader_s = leader.link_pos(leader.current_distance(change.start_time+1))['i']
    leader_platoon_start = leader.path[leader_s]
    leader_e = leader.link_pos(leader.current_distance(change.end_time-1))['i']
    leader_platoon_end = leader.path[leader_e]
    return follower_platoon_start == leader_platoon_start and follower_platoon_end == leader_platoon_end


def verify_truck(truck, assignments):
    verify_speed_history(truck)
    verify_deadline(truck)
    verify_platooning(truck, assignments)