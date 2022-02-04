import logging

import numpy as np

logger = logging.getLogger(__name__)


class Cell:
    """Class representing a cell in the grid."""
    cnt_id: int = 0

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float):
        assert x_min < x_max, "<Cell> x_min should be smaller than x_max"
        assert y_min < y_max, "<Cell> y_min should be smaller than y_max"

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.center = np.array([(x_max + x_min) / 2, (y_max + y_min) / 2])

        self.id = Cell.cnt_id
        Cell.cnt_id += 1

    def __repr__(self):
        return f"Cell(id={self.id}, x=[{self.x_min}, {self.x_max}], y=[{self.y_min}, {self.y_max}])"


class Grid:
    """Cartesian grid for discretizing the reachable sets."""

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float, size_grid: float):
        assert x_min < x_max, "<Grid> x_min should be smaller than x_max"
        assert y_min < y_max, "<Grid> y_min should be smaller than y_max"

        self.list_cells = list()
        self.size_grid = size_grid

        self.x_min_cells = np.floor(x_min / size_grid) * size_grid
        self.x_max_cells = np.ceil(x_max / size_grid) * size_grid
        self.y_min_cells = np.floor(y_min / size_grid) * size_grid
        self.y_max_cells = np.ceil(y_max / size_grid) * size_grid

        self.list_x_cells = np.arange(self.x_min_cells, self.x_max_cells + 0.001, self.size_grid)
        self.list_y_cells = np.arange(self.y_min_cells, self.y_max_cells + 0.001, self.size_grid)
        self.num_columns = len(self.list_x_cells) - 1
        self.num_rows = len(self.list_y_cells) - 1

        self._create_cells()

    def _create_cells(self):
        logger.debug(f"\tCreating cartesian grid for x=[{self.x_min_cells},  {self.x_max_cells}], "
                     f"y=[{self.y_min_cells}, {self.y_max_cells}]...")

        for x_min_cell, x_max_cell in zip(self.list_x_cells[:-1], self.list_x_cells[1:]):
            for y_min_cell, y_max_cell in zip(self.list_y_cells[:-1], self.list_y_cells[1:]):
                self.list_cells.append(Cell(x_min_cell, x_max_cell, y_min_cell, y_max_cell))

        logger.debug(f"\t#Cells in grid: {len(self.list_cells)}")

    def __repr__(self):
        return f"Grid(#cols={self.num_columns}, #rows={self.num_rows}, x=[{self.x_min_cells}, {self.x_max_cells}], " \
               f"y=[{self.y_min_cells}, {self.y_max_cells}], size_grid={self.size_grid})"
