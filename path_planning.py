import math


ACTION_FORWARD = "forward"
ACTION_STEER_LEFT = "steer_left"
ACTION_STEER_RIGHT = "steer_right"
ACTION_TURN_LEFT = "turn_left"
ACTION_TURN_RIGHT = "turn_right"
ACTION_STOP = "stop"


def normalize_angle(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


def valid_distance(distance_cm):
    return distance_cm is not None and distance_cm >= 0


class Bug2Planner:
    def __init__(
        self,
        goal_x,
        goal_y,
        start_x=0.0,
        start_y=0.0,
        front_stop_cm=30,
        side_close_cm=20,
        side_far_cm=45,
        goal_tolerance_m=0.10,
        heading_tolerance_deg=12.0,
        mline_tolerance_m=0.10,
        leave_progress_m=0.10,
    ):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.start_x = start_x
        self.start_y = start_y
        self.front_stop_cm = front_stop_cm
        self.side_close_cm = side_close_cm
        self.side_far_cm = side_far_cm
        self.goal_tolerance_m = goal_tolerance_m
        self.heading_tolerance_rad = math.radians(heading_tolerance_deg)
        self.mline_tolerance_m = mline_tolerance_m
        self.leave_progress_m = leave_progress_m

        self.state = "go_to_goal"
        self.follow_side = "right"
        self.hit_distance_to_goal = None

    def choose_action(self, sensor_data, pose):
        if self.distance_to_goal(pose) <= self.goal_tolerance_m:
            self.state = "done"
            return ACTION_STOP

        if self.state == "done":
            return ACTION_STOP

        if self.state == "go_to_goal":
            return self._go_to_goal(sensor_data, pose)

        return self._follow_boundary(sensor_data, pose)

    def _go_to_goal(self, sensor_data, pose):
        if self._front_blocked(sensor_data):
            self.state = "follow_boundary"
            self.hit_distance_to_goal = self.distance_to_goal(pose)
            return self._pick_turn_direction(sensor_data)

        return self._steer_toward_goal(pose)

    def _follow_boundary(self, sensor_data, pose):
        if self._should_leave_boundary(sensor_data, pose):
            self.state = "go_to_goal"
            return self._steer_toward_goal(pose)

        return self._wall_follow(sensor_data)

    def _steer_toward_goal(self, pose):
        desired_theta = math.atan2(
            self.goal_y - pose["y"],
            self.goal_x - pose["x"]
        )

        heading_error = normalize_angle(desired_theta - pose["theta"])

        if abs(heading_error) <= self.heading_tolerance_rad:
            return ACTION_FORWARD

        if heading_error > 0:
            return ACTION_STEER_LEFT

        return ACTION_STEER_RIGHT

    def _wall_follow(self, sensor_data):
        if self.follow_side == "right":
            return self._follow_right_wall(sensor_data)

        return self._follow_left_wall(sensor_data)

    def _follow_right_wall(self, sensor_data):
        front_left = sensor_data["front_left"]
        front_right = sensor_data["front_right"]
        right = sensor_data["right"]

        if valid_distance(front_right) and front_right <= self.front_stop_cm:
            return ACTION_TURN_LEFT

        if valid_distance(front_left) and front_left <= self.front_stop_cm:
            return ACTION_TURN_RIGHT

        if valid_distance(right) and right <= self.side_close_cm:
            return ACTION_STEER_LEFT

        if not valid_distance(right) or right >= self.side_far_cm:
            return ACTION_STEER_RIGHT

        return ACTION_FORWARD

    def _follow_left_wall(self, sensor_data):
        front_left = sensor_data["front_left"]
        front_right = sensor_data["front_right"]
        left = sensor_data["left"]

        if valid_distance(front_left) and front_left <= self.front_stop_cm:
            return ACTION_TURN_RIGHT

        if valid_distance(front_right) and front_right <= self.front_stop_cm:
            return ACTION_TURN_LEFT

        if valid_distance(left) and left <= self.side_close_cm:
            return ACTION_STEER_RIGHT

        if not valid_distance(left) or left >= self.side_far_cm:
            return ACTION_STEER_LEFT

        return ACTION_FORWARD

    def _pick_turn_direction(self, sensor_data):
        front_left = sensor_data["front_left"]
        front_right = sensor_data["front_right"]

        if valid_distance(front_left) and valid_distance(front_right):
            if front_left < front_right:
                self.follow_side = "left"
                return ACTION_TURN_RIGHT


            if front_right < front_left:
                self.follow_side = "right"
                return ACTION_TURN_LEFT

        left_clearance = self._clearance_score(sensor_data["left"])
        right_clearance = self._clearance_score(sensor_data["right"])

        if left_clearance >= right_clearance:
            self.follow_side = "right"
            return ACTION_TURN_LEFT

        self.follow_side = "left"
        return ACTION_TURN_RIGHT

    def _should_leave_boundary(self, sensor_data, pose):
        if self.hit_distance_to_goal is None or self._front_blocked(sensor_data):
            return False

        closer_to_goal = (
            self.distance_to_goal(pose)
            < self.hit_distance_to_goal - self.leave_progress_m
        )

        return closer_to_goal and self.distance_to_mline(pose) <= self.mline_tolerance_m

    def _front_blocked(self, sensor_data):
        front_left = sensor_data["front_left"]
        front_right = sensor_data["front_right"]

        left_blocked = (
            valid_distance(front_left)
            and front_left <= self.front_stop_cm
        )

        right_blocked = (
            valid_distance(front_right)
            and front_right <= self.front_stop_cm
        )

        return left_blocked or right_blocked

    def _clearance_score(self, distance_cm):
        if not valid_distance(distance_cm):
            return 999

        return distance_cm

    def distance_to_goal(self, pose):
        return math.hypot(
            self.goal_x - pose["x"],
            self.goal_y - pose["y"]
        )

    def distance_to_mline(self, pose):
        line_dx = self.goal_x - self.start_x
        line_dy = self.goal_y - self.start_y
        line_length = math.hypot(line_dx, line_dy)

        if line_length == 0:
            return self.distance_to_goal(pose)

        return abs(
            line_dy * pose["x"]
            - line_dx * pose["y"]
            + self.goal_x * self.start_y
            - self.goal_y * self.start_x
        ) / line_length
