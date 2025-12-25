import ext_plant

id = ["216764803"]

class Controller:
    """This class is a controller for the ext_plant game."""

    def __init__(self, game: ext_plant.Game):
        """Initialize controller for given game model."""
        self.original_game = game
        self.rows, self.cols = self.problem["Size"]
        self.walls = frozenset(self.problem.get("Walls", []))
        self.probs = self.problem["robot_chosen_action_prob"]
        self.goal_reward = self.problem.get("goal_reward", 0)


  
    def choose_next_action(self, state):
        """ Choose the next action given a state."""
        raise ValueError("Fill the function")