import numpy as np
from grid import GridEnv
import matplotlib.pyplot as pp
from matplotlib.colors import ListedColormap
from grid_struct import humans, walls

class Robot():
    def __init__(self, grid_env, grid_size, xrob, yrob):
        self.grid_env = grid_env
        self.grid_proba = grid_env.getgrid()
        self.grid_size = grid_size
        self.grid = np.zeros((grid_size, grid_size))
        # self.current_location = (20-xrob, yrob-1)
        self.x, self.y = 20-xrob, yrob-1
        self.current_location = (self.x, self.y)
        self.bodycount = 0

        self.q_table = dict() # Store all Q-values in dictionary of dictionaries

        for x in range(grid_size): # Loop through all possible grid spaces, create sub-dictionary for each
            for y in range(grid_size):
                self.q_table[(x,y)] = {'UP':0, 'DOWN':0, 'LEFT':0, 'RIGHT':0} # Populate sub-dictionary with zero values for possible move

        self.epsilon = 0.05
        self.alpha = 0.1
        self.gamma = 1

        # Set available actions
        self.actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']

    ## Put methods here:
    def get_available_actions(self):
        """Returns possible actions"""
        return self.actions

    def agent_on_map(self):
        """Prints out current location of the agent on the grid (used for debugging)"""
        robot_grid = np.zeros((self.grid_size, self.grid_size), dtype=str)
        robot_grid[self.x, self.y] = 'x'
        return robot_grid

    def choose_action(self, available_actions):
        """Returns the optimal action from Q-Value table. If multiple optimal actions, chooses random choice.
        Will make an exploratory random action dependent on epsilon."""
        if np.random.uniform(0,1) < self.epsilon:
            action = available_actions[np.random.randint(0, len(available_actions))]
        else:

            q_values_of_state = self.q_table[self.current_location]
            maxValue = max(q_values_of_state.values())
            action = np.random.choice([k for k, v in q_values_of_state.items() if v == maxValue])

        return action

    def learn(self, old_state, reward, new_state, action):
        """Updates the Q-value table using Q-learning"""
        q_values_of_state = self.q_table[new_state]
        max_q_value_in_new_state = max(q_values_of_state.values())
        current_q_value = self.q_table[old_state][action]

        self.q_table[old_state][action] = (1 - self.alpha) * current_q_value + self.alpha * (reward + self.gamma * max_q_value_in_new_state)


    def play(self, trials=500, max_steps_per_episode=1000, learn=False):
        """The play function runs iterations and updates Q-values if desired."""
        reward_per_episode = [] # Initialise performance log

        for trial in range(trials): # Run trials
            self.__init__(self.grid_env, self.grid_size, self.x, self.y) # Reset grid and agent
            self.grid_env.__init__(self.grid_size, humans, walls)
            cumulative_reward = 0 # Initialise values of each game
            step = 0
            game_over = False
            while step < max_steps_per_episode and game_over != True: # Run until max steps or until game is finished
                old_state = self.current_location
                self.update()
                action = self.choose_action(self.actions)
                reward = self.make_step(action)
                new_state = self.current_location

                if learn == True: # Update Q-values if learning is specified
                    self.learn(old_state, reward, new_state, action)

                cumulative_reward += reward
                step += 1

                if self.check_state(): # If game is in terminal state, game over and start next trial
                    # self.__init__()
                    game_over = True
                # print("action : ", action)
                # print("Current position of the agent =", self.current_location)
                # print(self.grid)

            # print("trial : ", trial)
            reward_per_episode.append(cumulative_reward) # Append reward for current trial to performance log



            # print(self.agent_on_map())

        return reward_per_episode # Return performance log


    def get_reward(self, location):
        """Returns the reward for an input position"""
        values = [0.0, 0.1, 0.3, 0.5, 0.6, 1.0, 1.2]
        rewards = [-1, 2, 5, -50, 10, -100, 100]
        reward=0
        for i in range(len(values)):
            if self.grid[location] == values[i]:
                reward = rewards[i]
            if reward == 100:
                # update grid
                self.grid_env.update(location)
                self.grid_proba = self.grid_env.getgrid()
                self.bodycount +=1
                print("Humain n°",self.bodycount,"trouvé !")

        return reward

    def make_step(self, action):
        """Moves the agent in the specified direction. If agent is at a border, agent stays still
        but takes negative reward. Function returns the reward for the move."""
        # Store previous location
        last_location = self.current_location
        if action == 'UP':
            print("UP:", self.current_location)
            # If agent is at the top, stay still, collect reward
            if last_location[0] == 19:
                reward = self.get_reward(last_location)
            else:
                self.current_location = (self.current_location[0]+1, self.current_location[1])
                reward = self.get_reward(self.current_location)

        elif action == 'DOWN':
            # If agent is at bottom, stay still, collect reward
            print("DOWN:", self.current_location)
            if last_location[0] == 0:
                reward = self.get_reward(last_location)
            else:
                self.current_location = (self.current_location[0]-1, self.current_location[1])
                reward = self.get_reward(self.current_location)

        elif action == 'LEFT':
            # If agent is at the left, stay still, collect reward
            if last_location[1] == 0:
                reward = self.get_reward(last_location)
            else:
                print("LEFT:", self.current_location)
                self.current_location = (self.current_location[0], self.current_location[1]-1)
                reward = self.get_reward(self.current_location)

        elif action == 'RIGHT':
            # If agent is at the right, stay still, collect reward
            if last_location[1] == 19:
                reward = self.get_reward(last_location)
            else:
                print("RIGHT:", self.current_location)
                self.current_location = (self.current_location[0], self.current_location[1]+1)
                reward = self.get_reward(self.current_location)


        return reward


    def check_state(self):
        if self.bodycount == 9:
            return True
        else:
            return False

    def getgrid(self):
        return self.grid

    def update(self):
        self.grid[self.current_location[0], self.current_location[1]] = self.grid_proba[self.current_location[0], self.current_location[1]]
        # print(self.grid)


    def display(self, inverted=False):
        if inverted:
            self.grid = 1 - self.grid
        # fig = px.imshow(self.grid, color_continuous_scale='cividis_r', title='Grid Environment', template=template)

        colors = ['white','palegreen', 'limegreen', 'silver', 'forestgreen', 'black', 'red']

        matrix = np.array(self.grid)
        for i in range(matrix.shape[0]):
            for j in range(matrix.shape[1]):

                if matrix[i, j] == 0.1:
                    matrix[i, j] = 0.2
                elif matrix[i, j] == 0.3:
                    matrix[i, j] = 0.4
                elif matrix[i, j] == 0.5:
                    matrix[i, j] = 0.6
                elif matrix[i, j] == 0.6:
                    matrix[i, j] = 0.8

        # Création de la palette de couleurs à partir de la liste de couleurs
        color_map = ListedColormap(colors)

        # Affichage de l'image en utilisant la palette de couleurs
        fig, ax = pp.subplots()
        # for x in range(0, 21):
        #     ax.axvline(x)
        # for y in range(0, 21):
        #     ax.axhline(x)


        ax.imshow(matrix, cmap=color_map, extent=[0, len(matrix[0]), 0, len(matrix)])
        ax.set_yticks(range(len(matrix)))
        ax.set_xticks(range(len(matrix[0])))
        ax.grid()
        pp.show()




if __name__ == '__main__':
    gridenv = GridEnv(20, humans, walls)
    gridenv.build_env()
    gridenv.display(inverted=False)

    # print(gridenv)


    gridrob = Robot(gridenv, 20, 2,2)
    gridrob.display(inverted=False)
    # gridproba = gridrob.getgrid()



    print("Current position of the agent =", gridrob.current_location)
    print(gridrob.agent_on_map())
    available_actions = gridrob.get_available_actions()
    print("Available_actions =", available_actions)
    chosen_action = gridrob.choose_action(available_actions)
    print("Randomly chosen action =", chosen_action)
    reward = gridrob.make_step(chosen_action)
    print("Reward obtained =", reward)

    # print(gridrob)

    reward_per_episode = gridrob.play(trials=500, learn=True)

    # Simple learning curve

    pp.plot(reward_per_episode)
    pp.show()