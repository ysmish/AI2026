import search
import utils

id = ["No numbers - I'm special!"]

_MOVE_DELTAS = {
    "UP": (-1, 0),
    "DOWN": (1, 0),
    "LEFT": (0, -1),
    "RIGHT": (0, 1),
}


class WateringProblem(search.Problem):
    """Multi-robot watering problem with taps, plants, and walls."""

    def __init__(self, initial_data):
        self._size = initial_data["Size"]
        self._walls = frozenset(initial_data.get("Walls", set()))
        self._plant_targets = dict(initial_data["Plants"])
        self._robot_capacities = self._extract_robot_capacities(initial_data["Robots"])
        self._tap_index = {}
        self._plant_index = {}

        robot_states = self._build_robot_states(initial_data["Robots"])
        plant_states = self._build_plant_states(initial_data["Plants"])
        tap_states = self._build_tap_states(initial_data["Taps"])

        search.Problem.__init__(self, (robot_states, plant_states, tap_states))

    @staticmethod
    def _extract_robot_capacities(robots_data):
        return {
            robot_id: capacity
            for robot_id, (_, _, _, capacity) in robots_data.items()
        }

    def _build_robot_states(self, robots_data):
        return tuple(
            (robot_id, r, c, load)
            for robot_id, (r, c, load, _) in sorted(robots_data.items())
        )

    def _build_plant_states(self, plants_data):
        states = tuple(
            (r, c, 0)
            for (r, c) in sorted(plants_data.keys())
        )
        self._plant_index = {
            (r, c): idx for idx, (r, c, _) in enumerate(states)
        }
        return states

    def _build_tap_states(self, taps_data):
        states = tuple(
            (r, c, remaining)
            for (r, c), remaining in sorted(taps_data.items())
        )
        self._tap_index = {
            (r, c): idx for idx, (r, c, _) in enumerate(states)
        }
        return states

    def successor(self, state):
        robot_states, plant_states, tap_states = state
        successors = []

        for robot_index, robot_state in enumerate(robot_states):
            robot_id, r, c, load = robot_state

            successors.extend(
                self._get_movement_successors(robot_states, plant_states, tap_states,
                                              robot_index, robot_id, r, c, load)
            )

            successors.extend(
                self._get_load_successor(robot_states, plant_states, tap_states,
                                         robot_index, robot_id, r, c, load)
            )
            successors.extend(
                self._get_pour_successor(robot_states, plant_states, tap_states,
                                         robot_index, robot_id, r, c, load)
            )

        return successors

    def _get_movement_successors(self, robot_states, plant_states, tap_states,
                                 robot_index, robot_id, r, c, load):
        successors = []
        occupied = {
            (robot_r, robot_c)
            for idx, (_, robot_r, robot_c, _) in enumerate(robot_states)
            if idx != robot_index
        }

        for action_name, (dr, dc) in _MOVE_DELTAS.items():
            nr, nc = r + dr, c + dc
            if not self._in_bounds(nr, nc):
                continue
            if (nr, nc) in self._walls:
                continue
            if (nr, nc) in occupied:
                continue

            new_robot_state = (robot_id, nr, nc, load)
            new_robot_states = self._replace_tuple_entry(robot_states, robot_index, new_robot_state)
            new_state = (new_robot_states, plant_states, tap_states)
            successors.append(((robot_id, action_name), new_state))

        return successors

    def _get_load_successor(self, robot_states, plant_states, tap_states,
                            robot_index, robot_id, r, c, load):
        tap_idx = self._tap_index.get((r, c))
        if tap_idx is None:
            return []

        tap_r, tap_c, remaining = tap_states[tap_idx]
        if remaining <= 0:
            return []

        capacity = self._robot_capacities[robot_id]
        if load >= capacity:
            return []

        new_robot_state = (robot_id, r, c, load + 1)
        new_robot_states = self._replace_tuple_entry(robot_states, robot_index, new_robot_state)

        new_tap_state = (tap_r, tap_c, remaining - 1)
        new_tap_states = self._replace_tuple_entry(tap_states, tap_idx, new_tap_state)

        new_state = (new_robot_states, plant_states, new_tap_states)
        return [((robot_id, "LOAD"), new_state)]

    def _get_pour_successor(self, robot_states, plant_states, tap_states,
                            robot_index, robot_id, r, c, load):
        if load <= 0:
            return []

        plant_idx = self._plant_index.get((r, c))
        if plant_idx is None:
            return []

        plant_r, plant_c, poured = plant_states[plant_idx]
        target = self._plant_targets[(plant_r, plant_c)]
        if poured >= target:
            return []

        new_robot_state = (robot_id, r, c, load - 1)
        new_robot_states = self._replace_tuple_entry(robot_states, robot_index, new_robot_state)

        new_plant_state = (plant_r, plant_c, poured + 1)
        new_plant_states = self._replace_tuple_entry(plant_states, plant_idx, new_plant_state)

        new_state = (new_robot_states, new_plant_states, tap_states)
        return [((robot_id, "POUR"), new_state)]

    def _replace_tuple_entry(self, data_tuple, index, new_value):
        return data_tuple[:index] + (new_value,) + data_tuple[index + 1:]

    def _in_bounds(self, r, c):
        rows, cols = self._size
        return 0 <= r < rows and 0 <= c < cols

    def goal_test(self, state):
        _, plant_states, _ = state
        for r, c, poured in plant_states:
            if poured < self._plant_targets[(r, c)]:
                return False
        return True

    def h_astar(self, node):
        return self._remaining_water_need(node.state)

    def h_gbfs(self, node):
        state = node.state
        remaining = self._remaining_water_need(state)
        if remaining == 0:
            return 0

        robot_states, plant_states, _ = state
        unsatisfied_plants = [
            (r, c)
            for r, c, poured in plant_states
            if poured < self._plant_targets[(r, c)]
        ]

        if not unsatisfied_plants:
            return remaining

        min_distance = min(
            abs(pr - rr) + abs(pc - rc)
            for (_, rr, rc, _) in robot_states
            for (pr, pc) in unsatisfied_plants
        )

        return remaining + min_distance

    def _remaining_water_need(self, state):
        _, plant_states, _ = state
        return sum(
            self._plant_targets[(r, c)] - poured
            for r, c, poured in plant_states
        )


def create_watering_problem(game):
    """Factory method retained for checker compatibility."""
    return WateringProblem(game)


if __name__ == '__main__':
    import ex1_check
    ex1_check.main()