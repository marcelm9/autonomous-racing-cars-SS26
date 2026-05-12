import heapq
from typing import List, Tuple

import numpy
from scipy.ndimage import distance_transform_edt
from skimage.morphology.binary import binary_erosion
from skimage.segmentation import flood_fill


def compute_simple_raceline(
    grid: numpy.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    safety: int,
    sparse: int,
    closed_loop: bool,
) -> Tuple[List[Tuple[int, int]], numpy.ndarray]:
    """Computes a simple raceline from a given boolean occupancy grid.

    Arguments
    ---------
    grid: numpy.ndarray
        2D boolean array where True represents a wall.
    origin_x: float
        X-coordinate of the map origin.
    origin_y: float
        Y-coordinate of the map origin.
    resolution: float
        Resolution of the map grid.
    safety: int
        Amount of pixels eroded from the driveable area.
    sparse: int
        1 / sampling ratio for the final raceline.
    closed_loop: bool
        When True, the path is created as a closed loop.

    Returns
    -------
    Tuple[List[Tuple[int, int]], numpy.ndarray]
        A tuple containing:
        - A list of (row, col) tuples representing the sparse path.
        - A 2D array representing the costmap.
    """
    height, width = grid.shape

    # 1. Obtain the driveable area
    xc = int(-origin_x / resolution)
    yr = int(-origin_y / resolution)

    driveable_area = ~numpy.bitwise_and(
        # `flood_fill` returns changed image...
        flood_fill(grid.copy(), (yr, xc), 1),
        # ... so we take only the cells that are changed.
        grid,
    )

    # 2. Obtain the finish line
    finish_left_yr = yr - numpy.argmin(driveable_area[:yr, xc][::-1]) + 1
    finish_right_yr = yr + numpy.argmin(driveable_area[yr:, xc]) - 1

    if not closed_loop:
        prefinish_left_yr = yr - numpy.argmin(driveable_area[:yr, xc - 1][::-1]) + 1
        prefinish_right_yr = yr + numpy.argmin(driveable_area[yr:, xc - 1]) - 1
    else:
        prefinish_left_yr = prefinish_right_yr = yr

    # 3. Erode the driveable_area by n_safety pixels
    save_area = ~binary_erosion(
        image=driveable_area,
        footprint=[
            (numpy.ones((2 * safety + 1, 1)), 1),
            (numpy.ones((1, 2 * safety + 1)), 1),
        ],
    )
    # Note that for save_area, True = wall

    # 4. Calculate distance for each point in the save_area
    costmap = numpy.ones_like(save_area, dtype=float) * numpy.inf

    # Initialize the costmap
    costmap[finish_left_yr : finish_right_yr + 1, xc] = 0
    costmap[prefinish_left_yr : prefinish_right_yr + 1, xc - 1] = 1

    # Initialize start nodes
    nodes = [(_y, xc - 1) for _y in range(prefinish_left_yr, prefinish_right_yr + 1)]

    while nodes:
        _r, _c = nodes.pop(0)
        _cost = costmap[_r, _c]

        for _ri, _ci in [(-1, 0), (0, -1), (0, 1), (1, 0)]:
            _rc = (_r + _ri, _c + _ci)

            # Skip if outside of the map
            if min(_rc) < 0 or _rc[0] >= height or _rc[1] >= width:
                continue

            # Skip if the cell contains a wall.
            if save_area[_rc]:
                continue

            if costmap[_rc] > _cost + 1 and costmap[_rc] > 0:
                nodes.append(_rc)
                costmap[_rc] = _cost + 1

    # 5. Create a pixel path from the starting point
    path = [(yr, xc)]

    # We employ an 8-hood here. Fixed bug: included (1, 0) instead of duplicate (1, -1)
    _area = numpy.asarray(
        [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    )

    for _ in range(2):
        _hood = _area + path[-1]
        _values = costmap[_hood[:, 0], _hood[:, 1]]

        # Throw away all low numbers
        _values[_values < 3] = numpy.nan

        path.append(tuple(_hood[numpy.nanargmin(_values)]))

    # And now we continue further with creating the path.
    current_cost = costmap[path[-1]]

    # Until we reach the "start" of the path...
    while current_cost > 1:
        _hood = _area + path[-1]

        try:
            _values = costmap[_hood[:, 0], _hood[:, 1]]
        except IndexError:
            _hood_list = []

            for _r, _c in _hood.tolist():
                if _r < 0 or _c < 0 or _r >= height or _c >= width:
                    continue
                else:
                    _hood_list.append((_r, _c))

            _hood = numpy.asarray(_hood_list)
            _values = costmap[_hood[:, 0], _hood[:, 1]]

        path.append(tuple(_hood[numpy.argmin(_values)]))
        current_cost = costmap[path[-1]]

    # 6. Increase the point sparsity
    sparse_path = path[::sparse]

    return sparse_path, costmap


def compute_centered_raceline(
    grid: numpy.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    safety: int,
    sparse: int,
    closed_loop: bool,
) -> Tuple[List[Tuple[int, int]], numpy.ndarray]:
    height, width = grid.shape

    # 1. Obtain the driveable area
    xc = int(-origin_x / resolution)
    yr = int(-origin_y / resolution)

    driveable_area = ~numpy.bitwise_and(
        flood_fill(grid.copy(), (yr, xc), 1),
        grid,
    )

    # 2. Obtain the finish line
    finish_left_yr = yr - numpy.argmin(driveable_area[:yr, xc][::-1]) + 1
    finish_right_yr = yr + numpy.argmin(driveable_area[yr:, xc]) - 1

    if not closed_loop:
        prefinish_left_yr = yr - numpy.argmin(driveable_area[:yr, xc - 1][::-1]) + 1
        prefinish_right_yr = yr + numpy.argmin(driveable_area[yr:, xc - 1]) - 1
    else:
        prefinish_left_yr = prefinish_right_yr = yr

    # 3. Erode the driveable_area by n_safety pixels
    if safety > 0:
        save_area = ~binary_erosion(
            image=driveable_area,
            footprint=[
                (numpy.ones((2 * safety + 1, 1)), 1),
                (numpy.ones((1, 2 * safety + 1)), 1),
            ],
        )
    else:
        save_area = ~driveable_area

    # 4. Obstacle Distance Transform for Centering
    # `distance_transform_edt` calculates the distance to the nearest False (wall)
    wall_dist = distance_transform_edt(~save_area)

    # 5. Calculate costmap using a Priority Queue (Dijkstra)
    costmap = numpy.ones_like(save_area, dtype=float) * numpy.inf

    # Initialize the costmap
    costmap[finish_left_yr : finish_right_yr + 1, xc] = 0
    costmap[prefinish_left_yr : prefinish_right_yr + 1, xc - 1] = 1.0

    # Initialize priority queue
    pq = []
    for _y in range(prefinish_left_yr, prefinish_right_yr + 1):
        heapq.heappush(pq, (1.0, (_y, xc - 1)))

    visited = numpy.zeros_like(save_area, dtype=bool)

    while pq:
        _cost, (_r, _c) = heapq.heappop(pq)

        if visited[_r, _c]:
            continue
        visited[_r, _c] = True

        # 8-hood expansion for smoother centered paths
        for _ri, _ci in [
            (-1, 0),
            (0, -1),
            (0, 1),
            (1, 0),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]:
            _rc = (_r + _ri, _c + _ci)

            # Skip if outside of the map
            if _rc[0] < 0 or _rc[0] >= height or _rc[1] < 0 or _rc[1] >= width:
                continue

            # Skip if the cell contains a wall
            if save_area[_rc]:
                continue

            # Base movement cost
            move_dist = 1.0 if (_ri == 0 or _ci == 0) else 1.414

            # Centering Penalty: Higher cost when closer to walls
            w_d = wall_dist[_rc]
            penalty = 20.0 / (w_d + 1e-3)

            new_cost = _cost + move_dist + penalty

            if new_cost < costmap[_rc]:
                costmap[_rc] = new_cost
                heapq.heappush(pq, (new_cost, _rc))

    # 6. Extract Path via Gradient Descent (from End -> Start)
    _area = numpy.asarray(
        [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    )

    if closed_loop:
        # End point is the max cost neighbor touching the finish line
        _hood = _area + (yr, xc)
        valid_coords = [
            (r, c)
            for r, c in _hood
            if 0 <= r < height and 0 <= c < width and not numpy.isinf(costmap[r, c])
        ]
        end_point = (
            max(valid_coords, key=lambda rc: costmap[rc[0], rc[1]])
            if valid_coords
            else (yr, xc - 1)
        )
    else:
        # End point is the absolute highest cost generated on the track
        masked_costmap = numpy.ma.masked_invalid(costmap)
        max_idx = numpy.unravel_index(numpy.ma.argmax(masked_costmap), costmap.shape)
        end_point = tuple(int(x) for x in max_idx)

    # Trace backwards down the shortest-path gradient
    path_rev = [end_point]
    current_cost = costmap[end_point]
    visited_path = set(path_rev)

    while current_cost > 1.0:
        _hood = _area + path_rev[-1]

        valid_coords = [
            (r, c)
            for r, c in _hood
            if 0 <= r < height
            and 0 <= c < width
            and not numpy.isinf(costmap[r, c])
            and (r, c) not in visited_path
            and costmap[r, c] > 0.0  # Prevent stepping on the finish line too early
        ]

        if not valid_coords:
            break

        # Strictly follow the gradient downhill
        best_next = min(valid_coords, key=lambda rc: costmap[rc[0], rc[1]])

        if costmap[best_next] >= current_cost:
            break  # Failsafe

        path_rev.append(best_next)
        visited_path.add(best_next)
        current_cost = costmap[best_next]

    # Reverse to get proper Start -> End driving sequence
    path = path_rev[::-1]

    # Prepend the actual origin to close the link
    if not path or path[0] != (yr, xc):
        path.insert(0, (yr, xc))

    # 7. Increase the point sparsity
    sparse_path = [(int(r), int(c)) for r, c in path[::sparse]]

    return sparse_path, costmap
