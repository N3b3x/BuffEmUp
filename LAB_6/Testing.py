
path = [0,4,8,9,10]
def GetGoals(path):
    Goals = []
    current = []
    while(path):
        south = False
        north = False
        east = False
        west = False
        if(path[1] == path[0] - 4 ):
            north = True
        if(path[1] == path[0] + 4 ):
            south = True
        if(path[1] == path[0] - 1 ):
            east = True
        if(path[1] == path[0] + 1 ):
            west = True
        current.append(path[0])
        path.pop(0)
        while(path):
            if (north == True):
                if (path[1] == path[0] - 4):
                    path.pop(0)
                else:
                    current.append(path[0])
                    Goals.append(current)
                    current = []
                    break

            if (south == True):
                if (path[1] == path[0] + 4):
                    path.pop(0)
                else:
                    current.append(path[0])
                    Goals.append(current)
                    current = []
                    break

            if (east == True):
                if (path[1] == path[0] + 1):
                    path.pop(0)
                else:
                    current.append(path[0])
                    Goals.append(current)
                    current = []
                    break

            if (west == True):
                if (path[1] == path[0] - 1):
                    path.pop(0)
                else:
                    current.append(path[0])
                    Goals.append(current)
                    current = []
                    break

            if(len(path) == 1 ):
                current.append(path[0])
                Goals.append(current)
                return Goals
    return Goals

### SHould return list [[0,8],[8,9,10]]
print(GetGoals(path))