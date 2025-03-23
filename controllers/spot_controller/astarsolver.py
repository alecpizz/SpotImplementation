# This is a sample Python script.
import heapq


# Press Ctrl+F5 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def solve_maze(maze):
    rows = len(maze)
    cols = len(maze[0])

    start = (1, 0)
    goal = (7, 6)

    def is_valid(row, col):
        return 0 <= row < rows and 0 <= col < cols and maze[row][col] == '0'

    def get_neighbors(row, col, direction):
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for i, (dr, dc) in enumerate(directions):
            new_row, new_col = row + dr, col + dc
            if is_valid(new_row, new_col):
                neighbors.append((new_row, new_col, i))
        return neighbors

    def heuristic(row, col):
        return abs(row - goal[0]) + abs(col - goal[1])

    def make_path(came_from, current):
        path = []
        while current in came_from:
            previous, turn = came_from[current]
            path.append((current, turn))
            current = previous
        path.reverse()
        return path

    open_set = [(heuristic(start[0], start[1]), 0, start, 0, [])]
    came_from = {}
    g_score = {start: 0}
    while open_set:
        f_score, g, current, direction, path = heapq.heappop(open_set)

        if current == goal:
            final_path = make_path(came_from, current)
            commands = []
            prev_dir = 0
            for (r, c), dir_num in final_path:
                if dir_num != prev_dir:
                    diff = (dir_num - prev_dir) % 4
                    if diff == 1:
                        commands.append("right")
                    elif diff == 2:
                        commands.append("right")
                        commands.append("right")
                    else:
                        commands.append("left")
                commands.append("forward")
                prev_dir = dir_num
            return commands

        for neighbor_row, neighbor_col, neighbor_dir in get_neighbors(current[0], current[1], direction):
            temp_g_score = g + 1
            neighbor = (neighbor_row, neighbor_col)
            if neighbor not in g_score or temp_g_score < g_score[neighbor]:
                came_from[neighbor] = (current, neighbor_dir)
                g_score[neighbor] = temp_g_score
                f_score = temp_g_score + heuristic(neighbor_row, neighbor_col)
                heapq.heappush(open_set, (f_score, temp_g_score, neighbor, neighbor_dir, path + [neighbor]))

    return None
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    maze = [
        "11111111",
        "00001001",
        "11101101",
        "11100001",
        "11101101",
        "11101101",
        "11101101",
        "11111101"
    ]
    commands = solve_maze(maze)
    if commands:
        for command in commands:
            print(command)
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
