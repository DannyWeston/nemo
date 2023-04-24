import numpy as np

class OccupancyGrid:
    def __init__(self, size, resolution: int = 3, influence = 1, wall_threshold = 0.4):
        self.size = (size[0] * resolution, size[1] * resolution)

        self.resolution = resolution

        self.wall_threshold = wall_threshold # Anything below 0.25 is treated as a wall

        self.influence = influence

        self.grid = np.full(self.size, 100,  dtype=np.int8)

    def world_to_map(self, x, y):
        return (int(x * self.resolution + (self.size[0] / 2)), int(y * self.resolution + (self.size[1] / 2)))

    def map_to_world(self, x, y):
        return (x / self.resolution, y / self.resolution)

    def visit(self, x, y):
        grid_x, grid_y = self.world_to_map(x, y)

        start_x = max(0, grid_x - self.influence)
        start_y = max(0, grid_y - self.influence)
        end_x = min(self.size[0] - 1, grid_x + self.influence)
        end_y = min(self.size[1] - 1, grid_y + self.influence)

        for a in range(start_x, end_x + 1):
            for b in range(start_y, end_y + 1):
                self.grid[a][b] = 0 # Visited tiles are set to 0

    def flush_to_disk(self, path):
        with open(path, 'w') as file:
            for row in self.grid:
                for y in row:
                    file.write(f'{y},')
                file.write("\n")