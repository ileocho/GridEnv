import numpy as np
import pygame as pg

class GridEnv():
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size))
        self.state = (0, 0)
        self.humans_positions = []
        self.non_writable = []

    def place_humans(self, humans):
        for human in humans:
            self.state = (self.grid_size - human[0], human[1] - 1)
            self.grid[self.state] = 5
            self.humans_positions.append(self.state)
            self.add_aura()

    def add_aura(self):
        x, y = self.state[0], self.state[1]
        distances = []
        neighbors_index = []

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                distance= np.sqrt((i-x)**2+(j-y)**2)
                if distance <= 3*np.sqrt(2) and distance > 0:
                    distances.append(distance)
                    neighbors_index.append([i,j])

        print(distances)
        for index, neighbor in enumerate(neighbors_index):
            if distances[index] <= np.sqrt(2):
                self.grid[neighbor[0], neighbor[1]] += 0.6
                print("0.6", distances[index])
            elif 2 <= distances[index] <= np.sqrt(2)*3:
                self.grid[neighbor[0], neighbor[1]] += 0.3
                print("0.3", distances[index])
            elif 3 <= distances[index] <= np.sqrt(2)*4:
                self.grid[neighbor[0], neighbor[1]] += 0.1
                print("0.1", distances[index])
            else:
                pass

    def place_walls(self, walls):
        self.walls = walls
        for wall in walls:
            self.grid[wall.state] = 1



def display(grid):
    screen = pg.display.set_mode((900, 900))
    clock = pg.time.Clock()

    surface = pg.surfarray.make_surface((np.rot90(grid.grid) * 255))
    surface = pg.transform.scale(surface, (800, 800))  # Scaled a bit.
    surface = pg.transform.flip(surface, True, False)

    running = True
    while running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False

        screen.fill((30, 30, 30))
        screen.blit(surface, (50, 50))
        pg.display.flip()
        clock.tick(60)


if __name__ == '__main__':
    grid = GridEnv(20)
    humans = [[1,9],[1,15],[1,20],[8,10],[12,12],[12,18],[18,5],[20,9],[20,20]]
    grid.place_humans(humans)
    print(grid.grid)
    
    display(grid)