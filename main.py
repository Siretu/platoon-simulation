from platooning.platooning_methods import GreedyPlatooning
from route_calculation import get_path_data_sets
from run_simulation import dynamic_simulation


def main():
    path_data = get_path_data_sets('./testing/testroutes/test100-1/')
    result = dynamic_simulation(GreedyPlatooning(), path_data_sets=path_data, folder='./testing/testroutes/test100-1/')
    print sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])


if __name__ == "__main__":
    main()