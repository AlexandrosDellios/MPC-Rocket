import numpy as np
import casadi as ca
from typing import Tuple
from control import dlqr


class NmpcCtrl:
    """
    Nonlinear MPC controller.
    get_u should provide this functionality: u0, x_ol, u_ol, t_ol = mpc_z_rob.get_u(t0, x0).
    - x_ol shape: (12, N+1); u_ol shape: (4, N); t_ol shape: (N+1,)
    You are free to modify other parts    
    """



    def __init__(self, rocket, H, xs, us):
        """
        Hint: As in our NMPC exercise, you can evaluate the dynamics of the rocket using 
            CASADI variables x and u via the call rocket.f_symbolic(x,u).
            We create a self.f for you: x_dot = self.f(x,u)
        """      
        self.rocket=rocket
        self.H=H
        self.xs=xs
        self.us=us
        self.Ts=rocket.Ts
        
        self.nx=len(xs)
        self.nu=len(us)

        # symbolic dynamics f(x,u) from rocket
        self.f = lambda x,u: rocket.f_symbolic(x,u)[0]

        # for the terminal cost

        A_cont, B_cont = rocket.linearize(xs, us)

        A_d = np.eye(self.nx) + A_cont * self.Ts
        B_d = B_cont * self.Ts

        # Weights

        Q_diag = np.array([100, 100, 100, 10, 10, 200, 200, 200, 500, 1000, 1000, 1000]) #high on position 
        
        R_diag = np.array([10, 10, 10, 10])
        
        Q_mat = np.diag(Q_diag)
        R_mat = np.diag(R_diag)

        _,self.P_lqr,_ = dlqr(A_d, B_d, Q_mat, R_mat)

        self.Q_weights = Q_diag
        self.R_weights = R_diag

        self._setup_controller()
        
    
    def rk4(self, x, u): # Derived from Exercise 9
        
        k1 = self.f(x, u)
        k2 = self.f(x + self.Ts/2 * k1, u)
        k3 = self.f(x + self.Ts/2 * k2, u)
        k4 = self.f(x + self.Ts * k3, u)
        return x + (self.Ts/6) * (k1 + 2*k2 + 2*k3 + k4)

    def _setup_controller(self) -> None:

        self.opti=ca.Opti()

        self.X = self.opti.variable(self.nx, self.H + 1)
        self.U = self.opti.variable(self.nu, self.H)
        self.P = self.opti.parameter(self.nx)

        # Slack variable

        self.Slack = self.opti.variable(1, self.H)

        x_sym=ca.MX.sym('x',self.nx)
        u_sym=ca.MX.sym('u',self.nu)

        x_next=self.rk4(x_sym,u_sym)

        F = ca.Function('F', [x_sym, u_sym], [x_next])


        # Initial condition

        self.opti.subject_to(self.X[:, 0] == self.P)

        totalcost=0
        rho=1e3

        for k in range(self.H) :
            err_x = self.X[:, k] - self.xs
            err_u = self.U[:, k] - self.us
            totalcost += ca.sumsqr(err_x * np.sqrt(self.Q_weights))
            totalcost += ca.sumsqr(err_u * np.sqrt(self.R_weights))
            totalcost += rho * ca.sumsqr(self.Slack[:, k]) # high cost of the slack variable


            self.opti.subject_to(self.X[:, k+1] == F(self.X[:, k], self.U[:, k]))
            self.opti.subject_to(self.Slack[:, k] >= 0)

            # Constraints
            #self.opti.subject_to(self.opti.bounded(-80*np.pi/180,self.X[3,k],80*np.pi/180))
            self.opti.subject_to(self.opti.bounded(-80*np.pi/180,self.X[4,k],80*np.pi/180))
            self.opti.subject_to(self.X[11,k]>=0)
            self.opti.subject_to(self.opti.bounded(-0.26, self.U[0, k], 0.26))
            self.opti.subject_to(self.opti.bounded(-0.26, self.U[1, k], 0.26))
            self.opti.subject_to(40.0 <=self.U[2, k])
            self.opti.subject_to(self.U[2,k]<=80.0 + self.Slack[:, k])
            self.opti.subject_to(self.opti.bounded(-20.0, self.U[3, k], 20.0))
        # Terminal constraints

        self.opti.subject_to(self.opti.bounded(-80*np.pi/180, self.X[3, self.H], 80*np.pi/180))
        self.opti.subject_to(self.opti.bounded(-80*np.pi/180, self.X[4, self.H], 80*np.pi/180))
        self.opti.subject_to(self.X[11, self.H] >= 0)

        # Terminal cost

        err_term = self.X[:, self.H] - self.xs
        totalcost += ca.mtimes([err_term.T, self.P_lqr, err_term])

        self.opti.minimize(totalcost)
        
        # Options solver
        opts = {'ipopt.print_level': 0,'expand':True, 'print_time': 0, 'ipopt.sb': 'yes', 'ipopt.max_iter': 500}
        self.opti.solver('ipopt', opts)

        
        self.ocp = self.opti

        self.X_guess = np.tile(self.xs.reshape(-1, 1), (1, self.H + 1))
        self.U_guess = np.tile(self.us.reshape(-1, 1), (1, self.H))
        self.Slack_guess = np.ones((1, self.H)) * 5.0

        

    def get_u(self, t0: float, x0: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:

        # Initial state

        self.opti.set_value(self.P, x0)

        self.X_guess[:, 0] = x0

        self.opti.set_initial(self.X, self.X_guess)
        self.opti.set_initial(self.U, self.U_guess)
        self.opti.set_initial(self.Slack, self.Slack_guess)

        
        try:

            sol = self.opti.solve()
            
            
            x_val = sol.value(self.X)
            u_val = sol.value(self.U)
            slack_val = sol.value(self.Slack)

            if slack_val.ndim == 1:
                slack_val = slack_val.reshape(1, -1)
            
            
            self.X_guess = np.hstack((x_val[:, 1:], x_val[:, -1:]))
            self.U_guess = np.hstack((u_val[:, 1:], u_val[:, -1:]))
            self.Slack_guess = np.hstack((slack_val[:, 1:], slack_val[:, -1:]))
            
        except RuntimeError:
        
            print(f"Warning: Solver failed at t={t0}. Using previous guess.")
            u_val = self.U_guess
            x_val = self.X_guess

        
        u0 = u_val[:, 0]
        x_ol = x_val
        u_ol = u_val
        t_ol = t0 + np.arange(self.H + 1) * self.Ts

        return u0, x_ol, u_ol, t_ol
    
    
    