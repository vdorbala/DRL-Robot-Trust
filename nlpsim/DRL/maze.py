
maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

inter_y = [ 6, 14, 22, 30, 38]
mid_y = [ 2, 10, 18, 26, 34, 42]
inter_x=[ 6, 14, 22, 30, 38]
mid_x=[ 2, 10, 18, 26, 34, 42]

for i in mid_y:
    inter_y.append(i)

for i in mid_x:
    inter_x.append(i)

inter_x.sort()
inter_y.sort()


key=1
gazebo={}
for y in inter_y:
    for x in inter_x:
        # inter.append([x,y])
        gazebo[key]=[x,y]
        key +=1
m2g={}
k=1
for i in range(11):
    for j in range(11):
                    m2g[k]=[i,j]
                    k=k+1
