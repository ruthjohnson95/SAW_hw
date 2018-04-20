import numpy as np
from optparse import OptionParser
import sys

def print_csv(length_N_list,f):
    np.savetxt(f, length_N_list)
    return

def print_grid(grid, f):
    f.write('\n')
    for row in grid:
        for item in row:
            f.write("%d " % item)
        f.write("\n")

    return

def check_dead(x, y, grid):
    # check surrounding areas
    if grid[x - 1, y] != 0 and grid[x + 1, y] != 0 and grid[x, y - 1] != 0 and grid[x, y + 1] != 0:
        dead = True
    else:
        dead = False
    return dead

def default_move(x, y, grid):

    # count how many valid paths
    valid_moves = 0
    p1, p2, p3, p4 = 0, 0, 0, 0

    if grid[x+1, y] == 0:
        valid_moves += 1
        p1 = 1
    if grid[x-1, y] == 0:
        valid_moves += 1
        p2 = 1
    if grid[x, y+1] == 0:
        valid_moves += 1
        p3 = 1
    if grid[x, y-1] == 0:
        valid_moves += 1
        p4 = 1

    prob = 1.0/valid_moves

    p = np.multiply([p1, p2, p3, p4], prob)

    # pick random move
    move_temp = np.random.multinomial(1, p, 1)
    move = move_temp.ravel()

    if move[0] == 1: # move right
        x += 1
    elif move[1] == 1: # move left
        x -= 1
    elif move[2] == 1: # move up
        y+= 1
    else: # move down
        y-=1

    # return prob of corresponding move
    p_move = np.sum(np.multiply(p, move))

    return x,y,p_move


def sample(n, guide="default"):

    # make nxn grid
    # 0 - untouched
    # 1 - touched
    # (n+1) grid plus +2 border
    grid = np.zeros((n+3,n+3))

    # add boundary
    grid[0, : ] = 1
    grid[:,0] = 1
    grid[n+2, :] = 1
    grid[:, n+2] = 1

    # keep track of prob of each move
    move_prob = []

    # starting coordinates
    x = 1
    y = 1
    marker = 1
    grid[x, y] = marker  # mark beginning spot

    #print "\n"
    #print grid

    # while not off the grid
    while x>0 and x<n+2 and y>0 and y<n+2: # while not on border

        # propose new move
        if guide == "default":
            x,y,p = default_move(x,y,grid)

        else:
            print("need to specify move function...")
            exit(1)

        # add p to moves list
        move_prob.append(p)

        # mark grid
        marker += 1
        grid[x,y] = marker

        #print "\n"
        #print grid

        # check if stuck
        if check_dead(x, y, grid):
            # dead
            #print "Collision!"
            break



    #print move_prob
    # caclualate prob of SAW
    g = np.prod(move_prob)
    P_w = 1 / float(g)
    N = len(move_prob)

    return P_w, N, grid


def nn_sample(n, guide="default"):

    # keep track of attempts
    attempt = 1
    found = False
    final_grid = None

    # loop through until get (n,n) path
    while(not found):

        # make nxn grid
        # 0 - untouched
        # 1 - touched
        # (n+1) grid plus +2 border
        grid = np.zeros((n+3,n+3))

        # add boundary
        grid[0, : ] = 1
        grid[:,0] = 1
        grid[n+2, :] = 1
        grid[:, n+2] = 1

        # keep track of prob of each move
        move_prob = []

        # starting coordinates
        x = 1
        y = 1
        marker = 1
        grid[x, y] = marker  # mark beginning spot

        #print "\n"
        #print grid

        # while not off the grid
        while x>0 and x<n+2 and y>0 and y<n+2:

            # propose new move
            if guide == "default":
                x,y,p = default_move(x,y,grid)

            else:
                print("need to specify move function...")
                exit(1)

            # add p to moves list
            move_prob.append(p)

            # mark grid
            marker += 1
            grid[x,y] = marker

            #print "\n"
            #print grid

            # check if stuck
            if check_dead(x, y, grid):
                # dead
                #print "Collision!"
                #print('\n')
                #print(grid)

                # check to see if path died at (n,n)
                if x == n+1 and y == n+1:
                    g = np.prod(move_prob)
                    w = 1/float(g)
                    p_w = w/float(attempt)
                    found = True
                    N = len(move_prob)
                    final_grid = grid

                    break # found path, return sample
                else:
                    # add to attempts
                    attempt += 1
                    break # try generating another path

    # return probability and length of SAW
    return p_w, N, final_grid


def total_SAW(n, ITS):
    # keep track of samples
    sample_list = []
    length_list = []
    longest_saw_grid = None
    longest_saw = 0

    for i in range(0, ITS):
        if i%100 == 0:
            print("Iteration: %d" % i)
            sys.stdout.flush()


        prob, N, grid = sample(n)
        sample_list.append(prob)
        length_list.append(N)

        # check if longest SAW
        if N > longest_saw:
            longest_saw = N
            longest_saw_grid = grid

    # print sample_list

    # normalize
    total_SAW = np.sum(sample_list) / float(ITS)

    return total_SAW, length_list, longest_saw_grid



def diagonal_SAW(n, ITS):

    sample_list = []
    length_list = []
    longest_saw_grid = None
    longest_saw = 0

    for i in range(0, ITS):
        if i % 100 == 0:
            print("Iteration: %d" % i)
            sys.stdout.flush()
            
        # collect the weighted samples
        prob, N, grid = nn_sample(n)
        sample_list.append(prob)
        length_list.append(N)

        # check if longest SAW
        if N > longest_saw:
            longest_saw = N
            longest_saw_grid = grid

    # print sample_list

    # normalize
    diag_saw = np.sum(sample_list) / float(ITS)

    return diag_saw, length_list, longest_saw_grid



def main():

    # get input options
    parser = OptionParser()
    parser.add_option("--s", "--seed", dest="seed", default="7")
    parser.add_option("--ITS", "--ITS", dest="ITS", default=500)
    parser.add_option("--id", "--id", dest="id", default="total_saw")
    (options, args) = parser.parse_args()

    # set seed
    seed = int(options.seed)
    np.random.seed(seed)
    ITS = int(options.ITS)
    id = options.id


    # grid size
    n = 10

    # write to file
    f_nlist = open("saw.%s.%d.Nlist" % (id, ITS), 'w')
    f_out = open("saw.%s.%d.out" % (id, ITS), 'w')

    if id == "total_saw":
        total_saw, length_N_list, longest_saw_grid = total_SAW(n, ITS)
        print("Estimated total SAW: %.4g" % total_saw)
        print('\n')
        print longest_saw_grid
        print_csv(length_N_list, f_nlist)
        f_out.write("Estimated total SAW: %.4g \n" % total_saw)
        f_out.write("Longest SAW: ")
        print_grid(longest_saw_grid, f_out)

    elif id == "nn_saw":
        total_nn_saw, length_N_list, longest_saw_grid = diagonal_SAW(n, ITS)
        print("Estimated total nn SAW: %.4g" % total_nn_saw)
        print('\n')
        print longest_saw_grid
        print_csv(length_N_list,f_nlist)

        f_out.write("Estimated total nn SAW: %.4g \n" % total_nn_saw)
        f_out.write("Longest SAW: " )
        print_grid(longest_saw_grid, f_out)

    else:
        print("Not a valid option...exiting")
        exit(1)


if __name__== "__main__":
  main()


# save max path and image
# 3ish