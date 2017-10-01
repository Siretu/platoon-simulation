import cProfile
import time
import email_settings

from platooning.platooning_methods import GreedyPlatooning, RandomPlatooning, SubModularityPlatooning
from route_calculation import get_path_data_sets, generate_routes
from run_simulation import dynamic_simulation, average_fuel_savings
import matplotlib.pyplot as plt
import numpy as np


start = 0
pr = None
def setup():
    global start
    global pr
    start = time.time()
    pr = cProfile.Profile()
    pr.disable()
    pr.enable()


def tear_down(result=0):
    pr.disable()
    pr.dump_stats("profile.pstat")
    t = time.time() - start
    print "%s: %.3f" % ("Time", t)
    email_settings.mail("erikihr@gmail.com", "Simulation", "Total time: %f" % t)


def main():
    setup()
    # generate_routes(10000, './testing/testroutes/test10000-2/')
    # generate_routes(10000, './testing/testroutes/test10000-3/')
    # generate_routes(10000, './testing/testroutes/test10000-4/')
    # generate_routes(10000, './testing/testroutes/test10000-5/')
    # generate_routes(400, './testing/testroutes/test400-1/')
    # generate_routes(400, './testing/testroutes/test400-2/')
    # plot_clustering_savings_graph()
    # print plot_interval_graph()
    # print plot_horizon_graph()
    #result = average_fuel_savings(GreedyPlatooning(),['./testing/testroutes/test10000-1/'])
    #print result
    print clustering_data(testset="./testing/testroutes/test10000-%d/", nr=5)
    # result = dynamic_simulation(GreedyPlatooning(), folder='./testing/testroutes/test400-5/')
    # print sum([x.current_fuel_consumption() for x in result]) / sum([x.default_plan.fuel for x in result])
    # plot_expected_graph()
    tear_down()



def clustering_data(testset="./testing/testroutes/test400-%d/", nr=5):
    fuel_savings = []
    for method in [RandomPlatooning(0), SubModularityPlatooning(True), SubModularityPlatooning(False)]:
        fuel_savings.append(average_fuel_savings(method,[testset % (x+1) for x in range(nr)]))
        print fuel_savings
        email_settings.mail("erikihr@gmail.com", "Partial results", str(fuel_savings))
    return fuel_savings

def horizon_data():
    total = []
    for method in [GreedyPlatooning(), RandomPlatooning(0), SubModularityPlatooning(True), SubModularityPlatooning(False)]:
        savings = []
        for horizon in range(0, 7210, 300):
            print "%s: %d/7200" % (method, horizon)
            fuel_savings = average_fuel_savings(method,
                                                ['./testing/testroutes/test400-1/', './testing/testroutes/test400-2/',
                                                 './testing/testroutes/test400-3/', './testing/testroutes/test400-4/',
                                                 './testing/testroutes/test400-5/'], horizon)
            savings.append(fuel_savings)
            print fuel_savings
        print savings
        total.append(savings)
    print total
    return total


def interval_data():
    total = []
    for method in [GreedyPlatooning(), RandomPlatooning(0), SubModularityPlatooning(True), SubModularityPlatooning(False)]:
        savings = []
        for interval in range(600, 14410, 600):
            print "%d/14400" % interval
            fuel_savings = average_fuel_savings(method,
                                                ['./testing/testroutes/test400-1/', './testing/testroutes/test400-2/',
                                                 './testing/testroutes/test400-3/', './testing/testroutes/test400-4/',
                                                 './testing/testroutes/test400-5/'], interval=interval)
            savings.append(fuel_savings)
            print fuel_savings
        total.append(savings)
    print total
    return total

def plot_plan_fuel_over_time():
    data = [0.04257685171991539, 0.043240624726071999, 0.042726363999243497, 0.042461843775266205, 0.041792827797636156, 0.041120199915871056, 0.041999790590254646, 0.041253700563820606, 0.041890863284667089, 0.042822045450078727, 0.043330085501208628, 0.041593369411016656, 0.041235595812267788, 0.041579141839865039, 0.042234671565046636, 0.041690283970487643, 0.041440025488240793, 0.040767679356238332, 0.041271149742330128, 0.041111206360194094, 0.041338689270989756, 0.041211539802186482, 0.042289560909163734, 0.040935053585119709, 0.041917145340909956]

    x = range(len(data))
    plt.plot(x, data)
    plt.ylabel("Expected fuel saving")
    plt.xlabel("Time")
    plt.show()

def plot_expected_graph():
    data = [53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53851.970542722745, 53851.970542722745, 53841.257247676651, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53840.878190264455, 53834.162983977512, 53828.38321097476, 53828.38321097476, 53814.70125177523, 53814.70125177523, 53814.70125177523, 53807.940365717513, 53791.420951764783, 53730.362795895126, 53730.362795895126, 53735.421378168016, 53735.421378168016, 53699.772274595161, 53688.694823376863, 53688.290828207741, 53681.634955992136, 53681.634955992136, 53681.634955992136, 53681.634955992136, 53674.385483652819, 53674.385483652819, 53677.614409226415, 53674.369160992428, 53674.369160992428, 53650.690734228163, 53650.690734228163, 53631.438292345163, 53612.334215701703, 53612.334215701703, 53612.334215701703, 53612.334215701703, 53584.568526468196, 53589.138902754697, 53596.73912349873, 53596.73912349873, 53590.06260845832, 53590.06260845832, 53551.742607259483, 53548.570299875078, 53547.086399870212, 53549.80783460704, 53530.206670091415, 53496.610523302181, 53496.610523302181, 53496.610523302181, 53496.610523302181, 53496.610523302181, 53496.610523302181, 53487.066365268678, 53487.066365268678, 53487.066365268678, 53487.066365268678, 53486.626601059877, 53474.160673441031, 53463.387564463366, 53393.367722164883, 53385.693736717541, 53385.693736717541, 53382.189517927851, 53375.174174379252, 53375.174174379252, 53375.174174379252, 53374.365702525029, 53374.365702525029, 53374.365702525029, 53374.365702525029, 53367.929619977294, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53356.221238487647, 53356.221238487647, 53356.221238487647, 53356.221238487647, 53356.221238487647, 53336.514144637447, 53296.100081649201, 53240.488651998588, 53240.488651998588, 53210.105819956487, 53210.105819956487, 53210.105819956487, 53151.329817702645, 53130.042859596186, 53133.868654374033, 53133.682724004917, 53125.424663592799, 53125.120961098262, 53125.424663592799, 53125.120961098262, 53125.120961098262, 53125.120961098262, 53125.120961098262, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53124.473690413826, 53098.089298007137, 53106.486090648497, 53106.486090648497, 53106.486090648497, 53106.486090648497, 53069.915837513297, 53069.915837513297, 53069.915837513297, 53052.758986052751, 53057.496250684242, 53018.283924366173, 53018.283924366173, 53018.283924366173, 52991.048085296265, 52991.048085296265, 52991.048085296265, 52991.048085296265, 52991.048085296265, 52981.150072388147, 52995.385839291383, 52995.385839291383, 52992.489432387592, 52992.489432387592, 52992.489432387592, 52998.645330228144, 52998.645330228144, 52998.645330228144, 52998.018184739645, 52998.018184739645, 52963.072703427679, 52963.072703427679, 52963.072703427679, 52963.072703427679, 52906.72794382296, 52906.727943822974, 52906.727943822974, 52903.733982126985, 52903.733982126985, 52926.055075954224, 52926.055075954224, 52908.297191277816, 52908.297191277816, 52908.297191277816, 52908.297191277816, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52888.272779855077, 52888.272779855077, 52888.272779855077, 52888.272779855077, 52867.344741207518, 52889.769056231933, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52882.715935497166, 52868.970953782904, 52869.95293236729, 52817.926477553323, 52815.543544809931, 52807.803246096257, 52807.763632757153, 52767.784149474217, 52729.026667693368, 52729.026667693368, 52729.026667693368, 52666.552448539238, 52650.659744062621, 52625.502239449081, 52619.809343651068, 52617.911910584968, 52617.911910584968, 52617.911910584968, 52617.911910584968, 52617.911910584968, 52584.199250871505, 52584.199250871505, 52584.199250871505, 52577.740461899361, 52577.740461899361, 52577.740461899361, 52577.740461899361, 52577.740461899361, 52535.58308533841, 52528.229724538862, 52521.918852451476, 52501.981516247557, 52501.981516247557, 52482.551341594633, 52482.459744714768, 52482.459744714768, 52478.380844128988, 52478.380844128988, 52454.973517573904, 52449.102257886203, 52449.102257886203, 52430.690116359794, 52430.690116359794, 52430.690116359794, 52430.690116359794, 52430.690116359794, 52369.41311841397, 52369.41311841397, 52369.41311841397, 52365.89422849378, 52365.89422849378, 52365.89422849378, 52365.89422849378, 52365.89422849378, 52364.814553091142, 52364.814553091142, 52364.814553091142, 52358.572411466477, 52358.572411466477, 52358.572411466477, 52344.772416334556, 52303.702972729414, 52308.34246810418, 52306.061399328901, 52306.061399328901, 52306.061399328901, 52306.061399328901, 52292.503279685567, 52292.503279685567, 52292.503279685581, 52292.503279685567, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52292.503279685581, 52285.065651562691, 52285.065651562691, 52285.065651562691, 52285.065651562691, 52267.467684097865, 52267.467684097865, 52267.467684097865, 52267.467684097865, 52221.073312715947, 52221.073312715947, 52189.308303648417, 52189.308303648417, 52189.308303648417, 52188.773968799753, 52154.811653552199, 52157.351975417405, 52157.351975417405, 52157.351975417405, 52157.351975417405, 52152.99748803633, 52152.297311219969, 52152.297311219969, 52144.57189033585, 52144.57189033585, 52144.57189033585, 52144.57189033585, 52141.500194838343, 52132.549769266043, 52120.552081161659, 52120.552081161659, 52120.552081161659, 52120.552081161659, 52120.552081161659, 52100.839894722973, 52075.281833465982, 52075.281833465982, 52075.281833465982, 52075.281833465982, 52075.281833465982, 52075.245999940409, 52075.245999940409, 52075.245999940409, 52075.245999940409, 52069.725896085467, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52033.950734957834, 51960.653191448349, 51950.776826447196, 51950.776826447196, 51941.863505500238, 51941.863505500238, 51941.863505500238, 51941.863505500238, 51941.863505500238, 51912.405696997652, 51910.790562698356, 51910.790562698356, 51910.790562698356, 51910.362517890739, 51906.681439053966, 51906.681439053966, 51906.681439053966, 51906.681439053966, 51883.343562477297, 51885.494876079552, 51876.703081219282, 51837.112090763898, 51837.112090763898, 51837.112090763898, 51837.437546157889, 51837.437546157889, 51770.728080297791, 51770.728080297791, 51770.728080297791, 51770.728080297791, 51770.728080297791, 51752.331031112233, 51752.331031112233, 51752.331031112233, 51731.709480540623, 51702.700673461615, 51702.700673461615, 51696.153450190461, 51696.153450190461, 51696.153450190461, 51700.352673750538, 51683.898829198013, 51683.898829198013, 51665.453081212057, 51676.36708082955, 51673.074201377989, 51673.074201377989, 51673.074201377989, 51657.536226489516, 51635.955383419197, 51635.955383419197, 51629.073465681562, 51629.073465681562, 51629.073465681562, 51618.480745913948, 51618.33690650206, 51618.33690650206, 51579.689434992702, 51579.689434992702, 51568.676978949137, 51568.676978949137, 51568.676978949137, 51568.676978949137, 51568.676978949137, 51568.676978949137, 51546.608393430368, 51546.608393430368, 51502.232010450585, 51459.582900192952, 51441.532753978499, 51439.717873450136, 51429.502112870301, 51407.206388421619, 51407.206388421626]
    data = [53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53873.657440730603, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53872.379664897788, 53851.970542722745, 53851.970542722745, 53841.257247676651, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53842.119397206159, 53840.878190264455, 53834.162983977512, 53828.38321097476, 53828.38321097476, 53814.70125177523, 53814.70125177523, 53814.70125177523, 53807.940365717513, 53791.420951764783, 53730.362795895126, 53730.362795895126, 53735.421378168016, 53735.421378168016, 53699.772274595161, 53688.694823376863, 53688.290828207741, 53681.634955992136, 53681.634955992136, 53681.634955992136, 53681.634955992136, 53674.385483652819, 53674.385483652819, 53677.614409226415, 53674.369160992428, 53674.369160992428, 53650.690734228163, 53650.690734228163, 53631.438292345163, 53612.334215701703, 53612.334215701703, 53612.334215701703, 53612.334215701703, 53584.568526468196, 53589.138902754697, 53596.73912349873, 53596.73912349873, 53590.06260845832, 53590.06260845832, 53551.742607259483, 53548.570299875078, 53547.086399870212, 53549.80783460704, 53530.206670091415, 53496.610523302195, 53496.610523302195, 53496.610523302195, 53496.610523302195, 53496.610523302195, 53496.610523302195, 53487.066365268678, 53487.066365268678, 53487.066365268678, 53487.066365268678, 53486.626601059877, 53474.160673441031, 53463.387564463366, 53393.367722164883, 53385.693736717541, 53385.693736717541, 53382.189517927851, 53375.174174379252, 53375.174174379252, 53375.174174379252, 53374.365702525029, 53374.365702525029, 53374.365702525029, 53374.365702525029, 53367.929619977294, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53360.438436783123, 53356.221238487647, 53356.221238487647, 53356.221238487647, 53356.221238487647, 53356.221238487647, 53336.514144637447, 53296.100081649201, 53240.488651998588, 53240.488651998588, 53210.105819956487, 53210.105819956487, 53210.105819956487, 53151.329817702645, 53130.042859596186, 53133.868654374033, 53133.682724004917, 53125.424663592799, 53125.120961098262, 53125.424663592799, 53125.120961098262, 53125.120961098262, 53125.120961098262, 53125.120961098262, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53129.708459307454, 53124.473690413826, 53098.089298007137, 53106.486090648512, 53106.486090648512, 53106.486090648512, 53106.486090648512, 53069.915837513297, 53069.915837513297, 53069.915837513297, 53052.758986052751, 53057.496250684242, 53018.283924366173, 53018.283924366173, 53018.283924366173, 52991.048085296265, 52991.048085296265, 52991.048085296265, 52991.048085296265, 52991.048085296265, 52981.150072388147, 52995.385839291397, 52995.385839291397, 52992.489432387607, 52992.489432387607, 52992.489432387607, 52998.645330228144, 52998.645330228144, 52998.645330228144, 52998.018184739645, 52998.018184739645, 52963.072703427679, 52963.072703427679, 52963.072703427679, 52963.072703427679, 52906.72794382296, 52906.727943822974, 52906.727943822974, 52903.733982126985, 52903.733982126985, 52926.055075954224, 52926.055075954224, 52908.297191277816, 52908.297191277816, 52908.297191277816, 52908.297191277801, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52898.677529817709, 52888.272779855077, 52888.272779855077, 52888.272779855077, 52888.272779855077, 52867.344741207518, 52889.769056231933, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52886.0929457741, 52882.715935497166, 52868.970953782919, 52869.952932367305, 52817.926477553323, 52815.543544809931, 52807.803246096257, 52807.763632757153, 52767.784149474202, 52729.026667693368, 52729.026667693368, 52729.026667693368, 52666.552448539223, 52650.659744062621, 52625.502239449081, 52619.809343651068, 52617.911910584968, 52617.911910584968, 52617.911910584968, 52617.911910584968, 52617.911910584968, 52584.199250871505, 52584.199250871505, 52584.199250871505, 52577.740461899361, 52577.740461899361, 52577.740461899361, 52577.740461899361, 52577.740461899361, 52535.58308533841, 52528.229724538862, 52521.918852451476, 52501.981516247557, 52501.981516247557, 52482.551341594633, 52482.459744714753, 52482.459744714753, 52478.380844128988, 52478.380844128988, 52454.973517573904, 52449.102257886203, 52449.102257886203, 52430.690116359794, 52430.690116359794, 52430.690116359794, 52430.690116359794, 52430.690116359794, 52369.41311841397, 52369.41311841397, 52369.41311841397, 52365.894228493766, 52365.894228493766, 52365.894228493766, 52365.894228493766, 52365.894228493766, 52364.814553091142, 52364.814553091142, 52364.814553091142, 52358.572411466463, 52358.572411466463, 52358.572411466463, 52344.772416334541, 52303.702972729414, 52308.34246810418, 52306.061399328901, 52306.061399328901, 52306.061399328901, 52306.061399328901, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685567, 52292.503279685581, 52292.503279685567, 52292.503279685567, 52292.503279685581, 52292.503279685581, 52285.065651562691, 52285.065651562691, 52285.065651562691, 52285.065651562691, 52267.467684097865, 52267.467684097865, 52267.467684097865, 52267.467684097865, 52221.073312715947, 52221.073312715947, 52189.308303648417, 52189.308303648417, 52189.308303648417, 52188.773968799753, 52154.811653552199, 52157.351975417405, 52157.351975417405, 52157.351975417405, 52157.351975417405, 52152.99748803633, 52152.297311219969, 52152.297311219969, 52144.57189033585, 52144.57189033585, 52144.57189033585, 52144.57189033585, 52141.500194838343, 52132.549769266028, 52120.552081161644, 52120.552081161644, 52120.552081161644, 52120.552081161644, 52120.552081161644, 52100.839894722973, 52075.281833465982, 52075.281833465982, 52075.281833465982, 52075.281833465982, 52075.281833465982, 52075.245999940395, 52075.245999940395, 52075.245999940395, 52075.245999940395, 52069.725896085452, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52026.698607372789, 52033.95073495782, 51960.653191448349, 51950.776826447196, 51950.776826447196, 51941.863505500238, 51941.863505500238, 51941.863505500238, 51941.863505500238, 51941.863505500238, 51912.405696997652, 51910.790562698356, 51910.790562698356, 51910.790562698356, 51910.362517890724, 51906.681439053966, 51906.681439053966, 51906.681439053966, 51906.681439053966, 51883.343562477297, 51885.494876079538, 51876.703081219282, 51837.112090763883, 51837.112090763883, 51837.112090763883, 51837.437546157875, 51837.437546157875, 51770.728080297791, 51770.728080297791, 51770.728080297791, 51770.728080297791, 51770.728080297791, 51752.331031112219, 51752.331031112219, 51752.331031112219, 51731.709480540609, 51702.700673461615, 51702.700673461615, 51696.153450190461, 51696.153450190461, 51696.153450190461, 51700.352673750524, 51683.898829198013, 51683.898829198013, 51665.453081212057, 51676.367080829536, 51673.074201377975, 51673.074201377975, 51673.074201377975, 51657.536226489516, 51635.955383419197, 51635.955383419197, 51629.073465681562, 51629.073465681562, 51629.073465681562, 51618.480745913948, 51618.33690650206, 51618.33690650206, 51579.689434992702, 51579.689434992702, 51568.676978949123, 51568.676978949123, 51568.676978949123, 51568.676978949123, 51568.676978949123, 51568.676978949123, 51546.608393430368, 51546.608393430368, 51502.232010450585, 51459.582900192952, 51441.532753978499, 51439.717873450136, 51429.502112870301, 51407.206388421604, 51407.206388421604]

    x = range(len(data))
    plt.plot(x, data)
    plt.plot(x, [data[0]] * len(data))
    # plt.plot(x, [11448.5257358] * len(data))

    plt.ylim([0, 55000])
    plt.ylabel("Expected fuel consumption")
    plt.xlabel("Time")
    plt.legend(['Adapted plans', 'Default plan'])
    plt.show()


def plot_active_graph():
    data = [0, 1, 1, 1, 2, 3, 3, 3, 3, 4, 5, 5, 6, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 18, 19, 20, 20, 20, 20, 20, 20, 21, 22, 22, 22, 23, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 31, 32, 31, 31, 32, 33, 33, 33, 33, 33, 33, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 35, 35, 35, 36, 37, 38, 38, 38, 38, 38, 40, 41, 42, 42, 42, 40, 40, 40, 41, 41, 41, 41, 42, 42, 42, 43, 43, 43, 43, 43, 42, 42, 42, 42, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 45, 45, 45, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47, 46, 47, 47, 48, 48, 49, 50, 51, 52, 53, 53, 53, 53, 52, 53, 53, 53, 54, 53, 53, 53, 53, 53, 53, 54, 54, 54, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 53, 53, 53, 55, 54, 54, 54, 54, 53, 52, 52, 52, 51, 51, 52, 52, 52, 52, 52, 53, 53, 53, 53, 53, 53, 54, 53, 53, 54, 54, 54, 53, 53, 53, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 53, 53, 53, 54, 53, 54, 55, 55, 54, 53, 53, 53, 54, 54, 54, 54, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 53, 53, 53, 52, 52, 52, 53, 53, 51, 51, 51, 51, 51, 52, 52, 52, 52, 52, 52, 53, 53, 53, 53, 52, 52, 51, 51, 52, 52, 55, 55, 55, 55, 56, 55, 55, 55, 55, 56, 56, 56, 57, 56, 56, 57, 57, 55, 55, 54, 54, 53, 53, 53, 53, 53, 53, 53, 54, 54, 54, 53, 54, 54, 56, 58, 58, 58, 58, 57, 57, 57, 57, 57, 56, 56, 55, 56, 56, 56, 57, 56, 56, 56, 57, 57, 57, 56, 58, 58, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 60, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 61, 61, 61, 62, 62, 62, 62, 62, 63, 62, 62, 62, 62, 62, 61, 61, 62, 62, 62, 63, 63, 63, 64, 64, 64, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 64, 63, 64, 63, 63, 63, 63, 63, 62, 62, 63, 63, 63, 64, 65, 64, 64, 64, 63, 63, 63, 63, 63, 64, 64, 66, 67, 67, 68, 68, 68, 68, 69, 68, 68, 67, 66, 66, 66, 64, 64, 64, 63, 64, 64, 63, 63, 63, 63, 63, 62, 61, 62, 61, 61, 60, 60, 60, 60, 60, 60, 60, 60, 60, 61, 61, 60, 60, 60, 61, 62, 62, 61, 60, 60, 60, 61, 60, 60, 60, 60, 60, 60, 60, 62, 62, 62, 62, 60, 62, 62, 64, 63, 64, 64, 65, 64, 64, 63, 63, 63, 63, 62, 62, 63, 63, 63, 63, 64, 64, 64, 64, 64, 62, 61, 61, 63, 64, 65, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 66, 66, 66, 66, 66, 66, 66, 66, 65, 66, 66, 65, 65, 66, 67, 67, 67, 67, 67, 67, 66, 65, 65, 64, 65, 65, 65, 65, 64, 63, 62, 64, 64, 64, 64, 63, 63, 63, 61, 61, 62, 61, 61, 61, 61, 61, 60, 60, 60, 59, 58, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 61, 61, 62, 62, 62, 62, 61, 62, 62, 63, 63, 64, 64, 64, 64, 64, 64, 64, 64, 64, 63, 63, 63, 63, 63, 63, 62, 62, 62, 65, 65, 65, 65, 66, 67, 68, 68, 67, 67, 67, 67, 66, 67, 67, 67, 66, 66, 67, 66, 66, 66, 64, 64, 63, 63, 64, 64, 64, 64, 65, 65, 66, 66, 65, 66, 65, 66, 66, 69, 70, 71, 71, 72, 72, 72, 73, 73, 74, 74, 74, 74, 74, 75, 75, 75, 75, 75, 73, 73, 73, 74, 74, 74, 74, 74, 74, 74, 74, 73, 73, 74, 75, 75, 74, 74, 74, 75, 76, 76, 75, 75, 75, 73, 73, 74, 74, 74, 74, 74, 74, 74, 74, 75, 75, 76, 76, 75, 75, 75, 75, 75, 75, 75, 73, 73, 71, 71, 71, 71, 71, 71, 72, 71, 70, 69, 69, 70, 71, 70, 68, 68, 68, 68, 67, 66, 66, 67, 67, 67, 67, 66, 67, 67, 67, 66, 67, 68, 68, 68, 68, 68, 68, 66, 66, 67, 66, 65, 65, 66, 65, 65, 65, 65, 65, 65, 65, 65, 64, 65, 65, 65, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 63, 63, 63, 63, 63, 63, 64, 65, 65, 65, 65, 65, 65, 67, 67, 66, 66, 66, 66, 66, 65, 65, 64, 65, 65, 65, 64, 64, 63, 65, 66, 66, 67, 67, 66, 66, 66, 65, 65, 65, 64, 64, 65, 65, 65, 64, 63, 64, 64, 65, 65, 64, 64, 65, 65, 65, 65, 63, 62, 62, 62, 64, 64, 63, 63, 63, 64, 64, 64, 64, 63, 63, 63, 63, 63, 63, 62, 62, 62, 62, 62, 62, 62, 61, 60, 60, 60, 60, 61, 61, 61, 61, 60, 60, 60, 58, 57, 58, 58, 59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 59, 58, 58, 58, 58, 57, 57, 57, 56, 56, 56, 56, 56, 56, 56, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 55, 55, 55, 54, 54, 55, 55, 54, 53, 53, 53, 53, 52, 52, 52, 52, 52, 51, 50, 50, 49, 49, 49, 49, 49, 49, 50, 51, 51, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 52, 53, 54, 53, 53, 53, 54, 54, 55, 55, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 55, 55, 54, 54, 54, 55, 55, 56, 56, 56, 56, 56, 56, 55, 56, 56, 56, 56, 56, 56, 56, 57, 57, 56, 55, 55, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 56, 56, 56, 57, 56, 55, 55, 55, 54, 54, 55, 54, 54, 54, 54, 54, 56, 56, 56, 56, 56, 56, 57, 57, 57, 56, 56, 56, 56, 56, 56, 55, 56, 56, 56, 55, 55, 55, 55, 54, 54, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 55, 56, 57, 58, 55, 55, 55, 55, 55, 55, 55, 55, 56, 57, 57, 57, 56, 56, 55, 56, 56, 57, 56, 56, 57, 57, 58, 58, 58, 58, 56, 55, 54, 54, 53, 54, 54, 54, 54, 54, 54, 54, 54, 54, 55, 56, 57, 58, 58, 57, 57, 58, 58, 59, 59, 59, 59, 59, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 60, 60, 59, 59, 58, 58, 59, 60, 60, 60, 60, 59, 58, 57, 57, 57, 58, 58, 58, 58, 57, 57, 57, 57, 57, 57, 57, 56, 56, 56, 55, 54, 55, 56, 57, 57, 57, 56, 57, 57, 57, 57, 58, 59, 57, 56, 56, 55, 54, 54, 54, 53, 53, 53, 53, 53, 53, 52, 52, 52, 52, 53, 53, 54, 55, 55, 55, 55, 55, 55, 55, 55, 56, 55, 55, 55, 54, 56, 57, 58, 59, 59, 59, 59, 58, 57, 57, 56, 56, 56, 56, 56, 55, 56, 57, 58, 58, 58, 58, 58, 58, 58, 58, 57, 56, 56, 56, 56, 57, 57, 57, 56, 57, 56, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 56, 55, 54, 54, 54, 54, 54, 54, 54, 54, 53, 52, 52, 52, 52, 52, 52, 53, 52, 52, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 51, 52, 53, 53, 54, 54, 54, 54, 55, 55, 56, 56, 56, 56, 56, 56, 56, 57, 57, 57, 58, 59, 59, 59, 59, 60, 60, 60, 60, 59, 62, 62, 61, 62, 62, 61, 61, 61, 60, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 59, 59, 58, 58, 58, 58, 58, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 59, 59, 59, 58, 57, 57, 57, 56, 55, 55, 55, 55, 55, 54, 54, 56, 56, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57, 57, 56, 57, 56, 57, 56, 56, 55, 55, 56, 57, 57, 56, 54, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 51, 51, 51, 51, 51, 51, 51, 51, 50, 49, 49, 49, 48, 48, 48, 48, 48, 48, 47, 47, 48, 48, 48, 48, 49, 49, 49, 49, 49, 49, 49, 49, 50, 50, 51, 51, 51, 51, 50, 49, 49, 49, 49, 50, 48, 48, 48, 48, 48, 48, 47, 47, 45, 45, 45, 46, 46, 47, 48, 48, 48, 48, 48, 48, 48, 48, 48, 49, 49, 48, 48, 49, 49, 50, 50, 49, 49, 49, 49, 49, 49, 49, 49, 50, 49, 49, 49, 49, 47, 47, 47, 48, 48, 49, 48, 48, 47, 47, 46, 46, 46, 47, 47, 48, 48, 49, 50, 50, 51, 52, 53, 52, 53, 54, 54, 54, 54, 54, 54, 54, 54, 52, 53, 53, 53, 54, 54, 54, 54, 54, 55, 55, 54, 54, 55, 56, 56, 56, 56, 56, 55, 54, 54, 54, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 55, 56, 57, 57, 57, 57, 57, 57, 57, 57, 55, 55, 55, 55, 56, 56, 56, 55, 55, 55, 55, 55, 55, 55, 55, 55, 54, 53, 53, 53, 53, 54, 54, 52, 52]
    x = range(len(data))
    plt.plot(x, data)
    # plt.plot(x, [11448.5257358] * len(data))

    plt.xticks([36*i for i in range(5, 49, 5)], [str(i)+"h" for i in range(5, 49, 5)])
    plt.ylabel("Active trucks")
    plt.xlabel("Time")
    plt.show()


def plot_horizon_graph():
    data = [[0.04257685171991539, 0.043240624726071999, 0.042726363999243497, 0.042461843775266205, 0.041792827797636156, 0.041120199915871056, 0.041999790590254646, 0.041253700563820606, 0.041890863284667089, 0.042822045450078727, 0.043330085501208628, 0.041593369411016656, 0.041235595812267788, 0.041579141839865039, 0.042234671565046636, 0.041690283970487643, 0.041440025488240793, 0.040767679356238332, 0.041271149742330128, 0.041111206360194094, 0.041338689270989756, 0.041211539802186482, 0.042289560909163734, 0.040935053585119709, 0.041917145340909956], [0.038286281860540193, 0.037635498642601871, 0.038263970382947578, 0.036912435043820115, 0.038093780213181973, 0.038112958290865737, 0.038334145630736338, 0.036837403568618088, 0.038566928481584009, 0.039938299213352638, 0.03913802320049118, 0.038274408779102667, 0.03628575084089878, 0.036519171596552227, 0.038117506852366236, 0.039389165412633355, 0.039099987591722417, 0.040302105135019327, 0.041091322802924247, 0.040254232051881209, 0.040175147556484839, 0.040742244558931121, 0.040993132274765268, 0.041429333208759744, 0.041135842253129627], [0.039543045048311784, 0.040069445897934951, 0.039476473615592457, 0.040179646928607607, 0.039041021588396486, 0.039611477375396539, 0.040886617605510288, 0.038113349569262336, 0.03851757057545073, 0.040307779437964909, 0.040671176213001578, 0.039716123271036664, 0.038225025025876012, 0.037289082039978494, 0.039011776947156872, 0.039791978242690741, 0.040026939490590441, 0.04017011333443947, 0.041083505808621926, 0.039996608035952216, 0.040695624806885024, 0.039895289094308464, 0.039392516356696716, 0.039386041413067674, 0.037824491964886332], [0.038288638135185481, 0.03880443794836512, 0.038985048766235916, 0.03834217972777465, 0.038769213224278801, 0.036741154112205618, 0.03741468814216202, 0.037636448677803649, 0.037610728526537331, 0.039732536794915951, 0.039452568167468824, 0.038440363808861157, 0.037835331257484325, 0.038316757269082548, 0.038471953883773405, 0.039515519778140475, 0.039840215678517851, 0.039162487573239299, 0.037765401338263539, 0.03861315608843361, 0.038650355357199274, 0.038708629254534797, 0.040081480548290818, 0.039141583776356237, 0.039604264753606656]]


    x = range(len(data[0]))
    plt.plot(x, data[0])
    plt.plot(x, data[1])
    plt.plot(x, data[2])
    plt.plot(x, data[3])
    # plt.plot((-10, 27), (0, 0), 'k-')
    plt.xlim([0,24])
    plt.ylim([0,0.05])
    plt.title("Impact of horizon length")
    plt.ylabel("Fuel savings")
    plt.xlabel("Horizon length")
    plt.xticks([6, 12, 18, 24], ["30m", "1h", "1.5h", "2h"])

    plt.legend(['Greedy', 'Random', "Deterministic\nsub modularity", "Stochastic\nsub modularity"], loc='lower left')
    plt.show()


def plot_interval_graph():
    data = [[0.043854918835102306, 0.042366000065660492, 0.038989410785479593, 0.039143877762402954, 0.035500929358246003, 0.03258960502804098, 0.030111599308071769, 0.027821952648465341, 0.023649510155779496, 0.024383130767085516, 0.021612031737271709, 0.020129497321589286, 0.014827023718895549, 0.01771249718050396, 0.014673347057977449, 0.01201079530025162, 0.012128786448053087, 0.009702761166647967, 0.010707866437007451, 0.011350596403770053, 0.0074997264725151112, 0.0077151220150105228, 0.0070413627185034857, 0.0065561227578120104], [0.037676853751219409, 0.037503552052317965, 0.035239584909751834, 0.03609598326369505, 0.031763440434284897, 0.028460559190985779, 0.026263182929288066, 0.023968519589736316, 0.020913692036378738, 0.022355666966662892, 0.02013262875578441, 0.01847191637670249, 0.013432569878493127, 0.01574940467285655, 0.013335802138774699, 0.0098336265110144701, 0.010144644842877937, 0.0080456908801019196, 0.0076951116842233656, 0.0096330964303923856, 0.0055659100782145202, 0.0059186485715964302, 0.0061356366465219695, 0.0041882538973908364], [0.043437296515387191, 0.041670645312957812, 0.038324123033300683, 0.037526429402884066, 0.035708454716361547, 0.032165196670862171, 0.029558027007720367, 0.026343169496349426, 0.021374447537921927, 0.02392228242138732, 0.020470809748314122, 0.019776938921850153, 0.015053446114885372, 0.018269278423906087, 0.014690425324904033, 0.011618216995517794, 0.011686572224632385, 0.010109916120436702, 0.010010525390528758, 0.012413520534409361, 0.007587629322577105, 0.0087902951674377235, 0.0061278097287264409, 0.0070539709969583923], [0.039438260204271526, 0.036985766154292145, 0.036343943676909608, 0.035456323019904291, 0.032586184534194887, 0.031270696016559983, 0.028223582379401591, 0.026578041760672976, 0.021919672841451375, 0.023247307941905548, 0.021034659396081491, 0.019542495002679395, 0.013909331462742957, 0.017393155278587737, 0.013292316655204982, 0.011955154212028019, 0.011145754240744243, 0.0088964337755760292, 0.00935629130362714, 0.011324844650459576, 0.0064717514418457522, 0.0074249112622978512, 0.0072263831559247205, 0.0053799880215421817]]


    x = range(len(data[0]))
    a = plt.plot(x, data[0])
    b = plt.plot(x, data[1])
    c = plt.plot(x, data[2])
    d = plt.plot(x, data[3])
    plt.plot((-10, 27), (0, 0), 'k-')
    plt.xlim([0,23])
    plt.ylim([0, 0.06])
    plt.title("Impact of interval length")
    plt.ylabel("Fuel savings")
    plt.xlabel("Interval length")
    plt.xticks([0, 5, 11, 17, 23], ["10m", "1h", "2h", "3h", "4h"])

    plt.legend(['Greedy', 'Random', "Deterministic\nsub modularity", "Stochastic\nsub modularity"])
    plt.show()


def plot_clustering_savings_graph():
    # y = clustering_data()
    y = [0.04257685171991539, 0.038286281860540193, 0.039543045048311784, 0.038288638135185481]
    labels = ["%.1f%%" % (y[0]*100), "%.1f%%" % (y[1]*100), "%.1f%%" % (y[2]*100), "%.1f%%" % (y[3]*100)]
    N = len(y)
    x = range(N)
    # ["a","b","c","d"]
    width = 1 / 1.5
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    bars = plt.bar(x, y, width, alpha=0.8, color=colors)
    for a, b in zip(x, y):
        plt.text(a, b + 0.005, labels[a], ha='center', va='bottom')
    # plt.tight_layout()
    plt.ylim([0, 0.1])
    plt.title("Clustering methods")
    plt.ylabel("Fuel savings")
    # plt.xlabel("Method")
    # plt.set_xticklabels()
    labels2 = ["Greedy", "Random", "Deterministic\nsub modularity", "Stochastic\nsub modularity"]
    plt.xticks(x, labels2)  # , rotation='vertical')
    plt.show()


if __name__ == "__main__":
    main()
