from .behavior import Behavior
from core.motion_controller import DifferentialController
from core.types import Robot, Ball, Vector2

class Defensa(Behavior):
    """
    El defensor se posiciona entre la pelota y el arco propio,
    siguiendo la pelota lateralmente y manteniendo una distancia mínima al arco.
    """

    def __init__(self, controller: DifferentialController | None = None):
        self.controller = controller or DifferentialController()
        self.goal_x = 0.0  # Coordenada X del arco propio (ajusta según tu campo)
        self.goal_y = 0.65 # Centro del arco (ajusta según tu campo)
        self.min_dist_to_goal = 0.15  # Distancia mínima al arco

    def step(self, robot: Robot, ball: Ball | None) -> tuple[float, float]:
        if ball is None:
            return 0.0, 0.0

        # Calcula el punto entre la pelota y el arco propio, manteniendo una distancia mínima
        direction = Vector2(ball.position.x - self.goal_x, ball.position.y - self.goal_y)
        norm = (direction.x**2 + direction.y**2)**0.5
        if norm == 0:
            norm = 1e-6  # Evita división por cero

        # Punto objetivo: entre el arco y la pelota, pero nunca más cerca del arco que min_dist_to_goal
        target_x = self.goal_x + self.min_dist_to_goal * (direction.x / norm)
        target_y = ball.position.y  # Sigue la pelota lateralmente

        target = Vector2(target_x, target_y)
        return self.controller.goto_point_lim(robot.pose, target)

