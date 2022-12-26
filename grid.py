import numpy as np
import plotly.express as px


class GridEnv():
    def __init__(self, grid_size, humans, walls):
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size))
        self.state = (0, 0)
        self.humans = humans
        self.walls = walls
        self.humans_cells = []
        self.wall_cells = []
        self.non_writable = []

    def build_env(self):
        self.place_walls(self.walls)
        self.place_humans(self.humans)

    def place_walls(self, walls):
        for wall in walls:
            x1, y1 = wall[0][0], wall[0][1]
            x2, y2 = wall[1][0], wall[1][1]

            if x1 == x2:
                for i in range(y1, y2):
                    self.grid[x1, i] = 1
                    self.non_writable.append([x1, i])
                    self.wall_cells.append([x1, i])
            elif y1 == y2:
                for i in range(x1, x2):
                    self.grid[i, y1] = 1
                    self.non_writable.append([i, y1])
                    self.wall_cells.append([i, y1])

        self.wall_aura()

    def wall_aura(self):
        for wall_cell in self.wall_cells:
            x, y = wall_cell[0], wall_cell[1]
            distances = []
            neighbors_index = []

            for i in range(self.grid_size):
                for j in range(self.grid_size):
                    distance = np.sqrt((x - i) ** 2 + (y - j) ** 2)
                    if distance <= np.sqrt(2) and distance > 0:
                        distances.append(distance)
                        neighbors_index.append([i, j])

            for i in range(len(distances)):
                if neighbors_index[i] not in self.non_writable:
                    if distances[i] <= np.sqrt(2):
                        self.grid[neighbors_index[i][0], neighbors_index[i][1]] = 0.5
                        # self.non_writable.append(neighbors_index[i])
                    else:
                        pass

    def place_humans(self, humans):
        for human in humans:
            self.state = (self.grid_size - human[0], human[1] - 1)
            self.grid[self.state] = 1
            self.humans_cells.append(self.state)
            self.non_writable.append(self.state)
            self.human_aura()

    def human_aura(self):
        x, y = self.state[0], self.state[1]
        distances = []
        neighbors_index = []

        for i in range(self.grid_size):
            for j in range(self.grid_size):
                distance = np.sqrt((x - i) ** 2 + (y - j) ** 2)

                if distance <= 2*np.sqrt(2) and distance > 0:
                    distances.append(distance)
                    neighbors_index.append([i, j])

        # propagate the aura from the human to the neighbors cells
        # when a wall is encountered, the aura stops propagating in that direction
        for i in range(len(distances)):
            # if neighbor cell is a wall
            # push the cell after the wall in the direction of the wall to the non_writable list

            if neighbors_index[i] in self.wall_cells:
                if x == neighbors_index[i][0]:
                    if y < neighbors_index[i][1]:
                        self.non_writable.append([neighbors_index[i][0], neighbors_index[i][1] + 1])
                    else:
                        self.non_writable.append([neighbors_index[i][0], neighbors_index[i][1] - 1])
                elif y == neighbors_index[i][1]:
                    if x < neighbors_index[i][0]:
                        self.non_writable.append([neighbors_index[i][0] + 1, neighbors_index[i][1]])
                    else:
                        self.non_writable.append([neighbors_index[i][0] - 1, neighbors_index[i][1]])

            if neighbors_index[i] not in self.non_writable:
                if distances[i] <= np.sqrt(2):
                    self.grid[neighbors_index[i][0], neighbors_index[i][1]] += 0.6
                elif distances[i] <= 2*np.sqrt(2):
                    self.grid[neighbors_index[i][0], neighbors_index[i][1]] += 0.3
                else:
                    pass

    def display(self, inverted=False):
        template = 'plotly_white'
        if inverted:
            self.grid = 1 - self.grid
            template = 'plotly_dark'
        fig = px.imshow(self.grid, title='Grid Environment', template=template)
        fig.show()


if __name__ == '__main__':
    humans = [[1, 9],
              [1, 15],
              [1, 20],
              [8, 10],
              [12, 12],
              [12, 18],
              [18, 5],
              [20, 9],
              [20, 20]]

    walls = [[[2, 3], [20, 3]],
             [[6, 5], [6, 8]],
             [[9, 5], [9, 8]],
             [[12, 5], [12, 8]],
             [[15, 7], [20, 7]],
             [[4, 9], [4, 14]],
             [[9, 9], [9, 14]],
             [[12, 10], [12, 14]],
             [[15, 16], [19, 16]],
             [[1, 15], [6, 15]],
             [[2, 19], [7, 19]],
             [[11, 18], [16, 18]],
             [[15, 16], [20, 16]],
             [[14, 12], [19, 12]]]

    grid = GridEnv(20, humans, walls)
    grid.build_env()
    grid.display(inverted=False)
