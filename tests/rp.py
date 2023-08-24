from collections import deque

MAP = [
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'],
    ['#', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '#'],
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '0', '#'],
    ['#', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '#'],
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '0', '#'],
    ['#', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '#'],
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '0', '#'],
    ['#', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '#'],
    ['#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#']
]


def rp_bfs(map_start, map_end, map_data=MAP):
    # define up,right,down,left
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    direction_symbols = ['^', '>', 'v', '<']
    visited = set()
    queue = deque([(map_start[0], map_start[1], [('s', map_start[0], map_start[1])])])  # (row, col, path)

    while queue:
        row, col, path = queue.popleft()

        if (row, col) == tuple(map_end):
            path[-1] = ('e', map_end[0], map_end[1])
            return path

        if (row, col) in visited:
            continue

        visited.add((row, col))

        for direction, symbol in zip(directions, direction_symbols):
            new_row, new_col = row + direction[0], col + direction[1]

            if 0 <= new_row < len(map_data) and 0 <= new_col < len(map_data[0]) and map_data[new_row][new_col] == '0':
                queue.append((new_row, new_col, path + [(symbol, new_row, new_col)]))

    return None


map_sta = [7, 1]
map_sto = [1, 1]
shortest_path = rp_bfs(map_sta, map_sto, map_data=MAP)

if shortest_path:
    print("Shortest path directions:", shortest_path)
    for step in shortest_path:
        if step[0] == 's' or step[0] == 'e':  # at sta or sto
            pass
        else:
            pass
else:
    print("No path found.")
