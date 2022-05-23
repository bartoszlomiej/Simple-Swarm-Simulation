from simulation.Simulation import Simulation

from utils.Resolution import Resolution

if __name__ == "__main__":
    #screen_resolution = Resolution(1920, 1024)
    screen_resolution = Resolution(1024, 720)

    attraction_strength = (screen_resolution.height**2 +
                           screen_resolution.width**2) / 100
    attraction_point = (screen_resolution.width / 2,
                        screen_resolution.height / 2, attraction_strength)

    Simulation(screen_resolution, 40, 55, 2, 40, attraction_point,
               10).run()  #change clock tick to 100
# Simulation(screen_resolution, 100, 44, 2, True, attraction_point, 8).run()
