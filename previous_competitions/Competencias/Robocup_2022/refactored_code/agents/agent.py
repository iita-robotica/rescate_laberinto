import random

class Agent:
    def __init__(self, possible_actions=[]) -> None:
        self.possible_actions = possible_actions

    def predict(self, state: list) -> list:
        raise NotImplementedError

    def get_action(self, state: list) -> str:
        return self.predict(state)


class RandomAgent(Agent):
    def __init__(self, possible_actions=[]) -> None:
        super().__init__(possible_actions)

    def predict(self, state: list) -> str:
        return random.choice(self.possible_actions)