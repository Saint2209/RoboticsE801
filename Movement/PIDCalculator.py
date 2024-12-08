import numpy as np

class PID:
    def __init__(self, setpoint, k_p, k_i, k_d):
        self.setpoint = setpoint
        self.k_p, self.k_i, self.k_d= k_p, k_i, k_d
        self.e_p, self.e_i, self.e_d = 0.0, 0.0, 0.0
        self.previous_e_p= 0.0
        self.pid = 0.0
        # print("here_func")
    
    def set_proportional_error(self, current):
        return self.setpoint - current

    def set_integral_error(self):
        return self.e_i + self.e_p

    def set_derivative_error(self):
        return self.previous_e_p-self.e_p
        
    def calculate_pid(self, current):
       

        self.e_p = self.set_proportional_error(current)
        # print(f"E_p: {self.e_p}")
        self.e_i = self.set_integral_error()
        self.e_d = self.set_derivative_error()
        self.previous_e_p = self.e_p

        arr = np.array([(self.k_p*self.e_p), (self.k_i*self.e_i), (self.k_d*self.e_d)])
        
        return np.sum(arr)
    