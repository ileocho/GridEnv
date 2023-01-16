

import random
import Grid_Proba

def alea_human():
    humans = []
    nb_human = 0
    while nb_human<9:
        create_human = [random.randint(1, 20),random.randint(1, 20)]
        check = True
        for human in humans:
            if create_human == human:
                check = False
        if check == True:
            humans.append(create_human)
            nb_human += 1

    return humans

def alea_wall(humans):
    walls = []
    nb_wall = 0
    nb_case = 0
    while nb_wall < 13 or nb_case < 100:
        check = True
        x_init = random.randint(0, 19)
        y_init = random.randint(0, 19)
        cases = 0
        if random.choice([True,False]):
            x_final = random.randint(max(0,x_init-8),min(19,x_init+8))
            cases = max(x_final,x_init)-min(x_final,x_init)
            if cases < 3:
                check = False
            y_final = y_init
            for human in humans:
                for i in range(min(x_final,x_init),max(x_final,x_init)+1):
                    if human == [i,y_final]:
                        check = False
            for wall in walls:
                for i in range(min(x_final,x_init),max(x_final,x_init)+1):
                    for j in range(min(wall[0][0],wall[1][0]),max(wall[0][0],wall[1][0])+1):
                        for k in range(min(wall[0][1],wall[1][1]),max(wall[0][1],wall[1][1])+1):
                            if [j,k]==[i,y_final]:
                                check=False
                
        else:
            x_final = x_init
            y_final = random.randint(max(0,y_init-8),min(19,y_init+8))
            cases = max(y_final,y_init)-min(y_final,y_init)
            if cases < 3:
                check = False
            for human in humans:
                for i in range(min(y_final,y_init),max(y_final,y_init)+1):
                    if human == [x_final,i]:
                        check = False
            for wall in walls:
                for i in range(min(y_final,y_init),max(y_final,y_init)+1):
                    for j in range(min(wall[0][0],wall[1][0]),max(wall[0][0],wall[1][0])+1):
                        for k in range(min(wall[0][1],wall[1][1]),max(wall[0][1],wall[1][1])+1):
                            if [j,k]==[x_final,i]:
                                check=False
        if check == True:
            walls.append([[x_init,y_init],[x_final,y_final]])
            nb_wall+=1
            nb_case+=cases
    return walls
        



for i in range(10):
    humans = alea_human()
    walls =alea_wall(humans)

    grid = Grid_Proba.GridEnv(20, humans, walls)
    grid.build_env()

    grid.display(inverted=False)


                

