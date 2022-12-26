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

            # starting from wall cell, find all neighbors within 1 blocks distance from the wall
            for i in range(self.grid_size):
                for j in range(self.grid_size):
                    distance = np.sqrt((x - i) ** 2 + (y - j) ** 2)
                    if distance <= np.sqrt(2) and distance > 0:
                        distances.append(distance)
                        neighbors_index.append([i, j])

            # if the neighbor is not writable, then it is a wall aura
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
            self.grid[self.state] = 1.2
            self.humans_cells.append(self.state)
            self.non_writable.append(self.state)
            self.human_aura()

    def human_aura(self):
        x, y = self.state[0], self.state[1]
        distances = []
        neighbors_index = []

        # starting from human cell, find all neighbors within 2 blocks distance from the human
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                distance = np.sqrt((x - i) ** 2 + (y - j) ** 2)
                if distance <= np.sqrt(2)*2 and distance > 0:
                    distances.append(distance)
                    neighbors_index.append([i, j])

        # cast a ray from the human to the neighbors and check if there is a wall in between
        # if there is a wall, then the neighbor is not visible to the human
        for i in range(len(distances)):
            if neighbors_index[i] not in self.non_writable:
                if distances[i] <= np.sqrt(2):
                    self.grid[neighbors_index[i][0], neighbors_index[i][1]] = 0.6
                    # self.non_writable.append(neighbors_index[i])
                elif np.sqrt(2) < distances[i] <= np.sqrt(2)*2:
                    if self.ray_casting(self.state, neighbors_index[i]) == True:
                        # if the ray is not blocked, then the neighbor is visible to the human
                        self.grid[neighbors_index[i][0], neighbors_index[i][1]] = 0.3
                        # self.non_writable.append(neighbors_index[i])
                else:
                    pass

    def ray_casting(self, start, end):
        wall_cells = self.wall_cells
        x1, y1 = start[0], start[1]
        x2, y2 = end[0], end[1]

        # we take the absolute value of the difference between the two points
        dx = x2 - x1
        dy = y2 - y1
        # we check if the line is steep or strait
        is_steep = abs(dy) > abs(dx)

        if is_steep:
            # if the line is steep, we swap the x and y coordinates
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # we check if the line is going from left to right
        if x1 > x2:
            # if it is not, we swap the start and end points
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # we calculate the difference between the two points
        dx = x2 - x1
        dy = y2 - y1

        # we calculate the error
        error = int(dx / 2.0)

        # we calculate the ystep (how much we are going to add to y every time)
        ystep = 1 if y1 < y2 else -1

        y = y1
        # we iterate over the points of the line
        for x in range(x1, x2 + 1):
            # we check if the line is steep and if it is, we swap the x and y coordinates
            coord = [y, x] if is_steep else [x, y]

            if coord in wall_cells:
                # if the coordinate is in the wall_cells list, then there is a wall in between
                # we return False to indicate that the ray is blocked
                return False
            # we add the error to y
            error -= abs(dy)
            if error < 0:
                # if the error is less than 0, we add the ystep to y
                y += ystep
                # we add the dx to the error
                error += dx

        # if we get to this point, it means that the ray is not blocked
        return True


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
