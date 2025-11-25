class Vehicle:
    def __init__(self, transfer_fn, max_wheel_speed=0.22):
        self.tf = transfer_fn
        self.max_speed = max_wheel_speed

    def compute_wheel_speeds(self, left_dist, right_dist, k):
        # uncrossed excitatory: left sensor -> left wheel, right sensor -> right wheel
        v_l = self.tf(left_dist, self.max_speed, k)
        v_r = self.tf(right_dist, self.max_speed, k)
      
        return v_l, v_r

