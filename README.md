# purepersuit
차 모델의 횡,종 방향을 선택하여 목표지점까지 도달

import numpy as n
import matplotlib.patches as pat
import matplotlib.pyplot as mp
import math

class PurePursuitController:
    def __init__(self, waypoints, lookahead_distance) :
        self.waypoints = waypoints
        self.lookahead_distance = lookahead_distance
    
    def find_lookahead_point(self, current_position):
        min_distance = float('inf')
        lookahead_point = None

        for waypoint in self.waypoints:
            distance = n.linalg.norm(n.array(waypoint) - n.array(current_position))

            if distance < min_distance:
                min_distance = distance
                closest_point = waypoint

            if distance > self.lookahead_distance and min_distance < self.lookahead_distance:
                lookahead_point = waypoint
                break
        
        # 미리 보기 점을 찾을 수 없는 경우, 미리 보기 _point가 없음인 경우 마지막 경유점을 반환합니다
        if lookahead_point is None:
            lookahead_point = self.waypoints[-1]

        return lookahead_point
    
    def calculate_steering_angle(self, current_position, current_yaw):
        lookahead_point = self.find_lookahead_point(current_position)

        if lookahead_point is None:
            return 0.0
        
        dx = lookahead_point[0] - current_position[0]
        dy = lookahead_point[1] - current_position[1]

        target_yaw = n.arctan2(dy,dx)
        steering_angle = target_yaw - current_yaw

        return steering_angle     #######19
    
def is_close_to_goal(current_position, goal_position, threshould=0.1):
    distance = n.linalg.norm(n.array(goal_position) - n.array(current_position))
    return distance <= threshould

def simulate_movement(initial_position, initial_yaw, controller, delta_t, velocity, max_iterations = 1000):
    positions = [initial_position]
    yaws = [initial_yaw]
    iterations = 0

    while not is_close_to_goal(positions[-1], controller.waypoints[-1]) and iterations < max_iterations:
        current_position = positions[-1]
        current_yaw = yaws[-1]

        steering_angle = controller.calculate_steering_angle(current_position, current_yaw)

        new_yaw = current_yaw + (velocity / controller.lookahead_distance) * n.tan(steering_angle) * delta_t
        new_position = (
            current_position[0] + velocity * n.cos(new_yaw) * delta_t,
            current_position[1] + velocity * n.sin(new_yaw) * delta_t
        )

        positions.append(new_position)
        yaws.append(new_yaw)
        iterations += 1

    return positions, yaws #######20

def visualize_movement(waypoints, positions, yaws):
    fig, ax = mp.subplots()

    # 경유지 표시
    waypoints_x, waypoints_y = zip(*waypoints)
    ax.plot(waypoints_x, waypoints_y, 'bo-', label='Waypoints')

    # 차량의 위치와 방향을 열거형으로 표시
    for i, (position, yaw) in enumerate(zip(positions, yaws)):
        ax.plot(position[0], position[1], 'ro-', markersize=1)
        ax.arrow(position[0], position[1], 0.1 * n.cos(yaw), 0.1 * n.sin(yaw), head_width=0.1, head_length=0.2, fc='r', ec='r')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_aspect('equal')
    ax.legend()
    mp.grid()
    mp.show() ######22

# 두 번째 사용예시 (2024/04/26: 이 부분 왜 있는건지 왜 적었는지 까먹음;)
x_values = [x * 0.5 for x in range(20)]
waypoints = [(x, math.sin(x)) for x in x_values]

lookahead_distance = 1.0

controller = PurePursuitController(waypoints, lookahead_distance)


#값 바꿔보기
initial_position = (0.5, 0)
initial_yaw = n.deg2rad(45)

delta_t = 0.1
velocity = 1.0
max_iterations = 100

positions, yaws = simulate_movement(initial_position, initial_yaw, controller, delta_t, velocity, max_iterations)
visualize_movement(waypoints, positions, yaws)
