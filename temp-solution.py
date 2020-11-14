
class clean_up_on_aisle_7:
  def __init__(self, grid, num_rows, num_cols, charging_stations):
    self.grid = grid
    self.charging_stations = charging_stations
    self.num_rows = num_rows
    self.num_cols = num_cols

  def find_nearest_charging_station(self):
      min_moves_required = num_rows + num_cols + 1
      closeset_charging_station = charging_stations[0]
      for x, y in charging_stations:
          moves_required = abs(pos.first - x) + abs(pos.second - y)
          if moves_required < min_moves_required:
              min_moves_required = moves_required
              closest_charging_station = (x, y)

      return (closest_charging_station, min_moves_required)

  # class method of RobotState

  def should_go_home(self):
    # go home if your fuel is euqal to the amount of fuel needed to go back
    # go home if you are out of fluid
    if not fluid:
      return True
    if fuel > max_fuel/2:
      return False
    _, moves_required = find_nearest_charging_station()
    if moves_required + 1 <= fuel:
      return True
    return False


# things todo:
# generate output list
input_data = {
    "robots": [
        ["jeremy", [10, 10]],
        ["gavin", [11, 10]],
        ["lindsay", [-1, 5]]
    ],
    "actions": [
        ["jeremy", "clean", 30],
        ["gavin", "move", [11, 11]],
        ["lindsay", "clean", 10]
    ]
}

grid = [[32, 15, 65], [33 58 32]]

# Format:
# FluidCap FuelCap
# row col
# grid

# test case 1
# 200 5
# 2 3
#    x
# 32 15 65
# 33 58 32
#    x


@dataclass
class Problem:
  max_fluid: int
  max_fuel: int
  floor: np.ndarray


@dataclass
class RobotState:
  pos: tuple
  fluid: int
  fuel: int
  name: string

  def clean_tile():
    if grid[pos.first][pos.second] and fluid:
      fluid_to_use = min(grid[pos.first][pos.second], fluid)
      action = [name, "clean", fluid_to_use]
    return action
    return None

    def


charging_stations = [(x, y), (x, y)]


def find_nearest_charging_station():
    min_moves_required = num_rows + num_cols + 1
    closeset_charging_station = charging_stations[0]
  	for x, y in charging_stations:
    	moves_required = abs(pos.first - x) + abs(pos.second - y)
        if moves_required < min_moves_required:
        	min_moves_required = moves_required
            closest_charging_station = (x, y)
  	
	return closest_charging_station
