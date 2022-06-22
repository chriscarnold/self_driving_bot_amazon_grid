import heapq
# Importing heapq to create a heap of points that need to be evaluated to get their heuristic values
import random
# Importing random to randomise the position of obstacles in phase 2


class Point:
    """
    Represents a point on the 2D-grid on the Amazon warehouse
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        # Initializing g, h and f values to a point to 0
        self.g = 0
        self.h = 0
        self.f = 0

    # Overwriting equality of points
    def __eq__(self, other):
        return self.position == other.position

    # Defining inequality
    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f


# traversal will return the traversed path in order
def traversal(current_point):
    path = []
    current = current_point
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]


def a_star(grid, start_point, delivery_point):
    """
    :param grid: The 2D grid that the vehicle will navigate through
    :param start_point: Starting point of the traversal
    :param delivery_point: End point
    :return: A list of points which the represents the squares that the vehicle traverses to get to the delivery point
    """

    # Creating a start and end point
    start_point = Point(None, start_point)
    start_point.g = start_point.h = start_point.f = 0
    end_point = Point(None, delivery_point)
    end_point.g = end_point.h = end_point.f = 0

    # Initialize a list of points that have been evaluated(closed_list) and a list of points that are to be evaluated(open_list)
    open_list = []
    closed_list = []

    # Transforming the list into a heap data structure and pushing the start point onto the heap
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_point)

    # Creating iterator variable to avoid loops or the vehicle leaving the warehouse grid
    iterations_var = 0
    max_iterations = (len(grid[0]) * len(grid) // 2)

    # All the adjacent squares that will be evaluated from the current point where the vehicle is
    # (top, bottom, left,right, diagonals)
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Looping until end point has been evaluated
    while len(open_list) > 0:
        iterations_var += 1

        if iterations_var > max_iterations:
            # if the iterator goes over the number of maximum iterations then it is likely that the vehicle got stuck
            print("Unable to reach delivery point!")
            return traversal(current_point)

            # Pop the current point from the heap and append it to the current_point list
        current_point = heapq.heappop(open_list)
        closed_list.append(current_point)

        # Condition for the end of the evaluation
        if current_point == end_point:
            return traversal(current_point)

        # Generating a list of child points
        children = []

        for new_position in adjacent_squares:

            # Get point position of the adjacent square
            point_position = (current_point.position[0] + new_position[0], current_point.position[1] + new_position[1])

            # Checking if the square is on the grid
            if point_position[0] > (len(grid) - 1) or point_position[0] < 0 or point_position[1] > (
                    len(grid[len(grid) - 1]) - 1) or point_position[1] < 0:
                continue

            # Checking if there is no obstacle on the square
            if grid[point_position[0]][point_position[1]] != 0:
                continue

            # Creating a new point
            new_point = Point(current_point, point_position)

            # Appending it to the children list
            children.append(new_point)

        # Looping through the children list
        for child in children:
            # Checking if child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Calculating heuristic values
            child.g = current_point.g + 1
            child.h = ((child.position[0] - end_point.position[0]) ** 2) + (
                    (child.position[1] - end_point.position[1]) ** 2)
            child.f = child.g + child.h

            # Checking if Child is in the heap
            if len([open_point for open_point in open_list if
                    child.position == open_point.position and child.g > open_point.g]) > 0:
                continue

            # Pushing the child on to the heap
            heapq.heappush(open_list, child)

    print("Unable to reach delivery point!", )


# Creating a grid as instructed in Phase 1
def main(print_grid=True):

    grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 1, 1, 1],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    # Setting delivery point and starting point
    start_point = (0, 0)
    delivery_point = (9, 9)

    # Applying A* algorithm for the vehicle to traverse the grid
    path = a_star(grid, start_point, delivery_point)

    step_num = (len(path) - 1)

    if print_grid:
        print("The number of steps are:", step_num)

    print("The vehicle will take the following path:", path)


# Populating the grid with 20 obstacles as instructed in Phase 2
def randomized_obstacles():
    # Starting with an empty grid
    grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    # Creating counter variable to ensure there are 20 obstacles
    counter = 0
    while counter < 20:
        row = random.randint(0, 9)
        column = random.randint(0, 9)

        # Ensuring that Start and Delivery points are not blocked
        if grid[0][0] == 1:
            grid[0][0] = 0
        if grid[9][9] == 1:
            grid[9][9] = 0

        # Ensuring that there are no duplicate obstacles
        grid[row][column] = 1
        counter = sum(sum(grid, []))

    return grid


def main2(print_grid=True):
    # Initializing the grid that has been randomised
    grid = randomized_obstacles()

    # Setting delivery point and starting point
    start_point = (0, 0)
    delivery_point = (9, 9)

    # Applying A* algorithm for the vehicle to traverse the grid
    path = a_star(grid, start_point, delivery_point)

    step_num = (len(path) - 1)

    if print_grid:
        print("The number of steps are:", step_num)

    print("The vehicle will take the following path:", path)

    # Identifying the obstacle that needs to be removed so the vehicle can reach the delivery point
    if path[(len(path) - 1)] != delivery_point:
        print("Please remove obstacle located at point ", path[(len(path) - 2)])


if __name__ == '__main__':
    main()
    main2()
