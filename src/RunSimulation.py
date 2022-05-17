from simulation.Simulation import Simulation

from utils.Resolution import Resolution

if __name__ == "__main__":
    screen_resolution = Resolution(1024, 720)
    attraction_strength = (screen_resolution.height**2 +
                           screen_resolution.width**2) / 8  #*2
    attraction_point = (500, 300, attraction_strength)

    Simulation(screen_resolution, 40, 55, 2, 40, attraction_point).run()
