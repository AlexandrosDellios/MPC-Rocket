import numpy as np
import cvxpy as cp
from control import place,dlqr
from mpt4py import Polyhedron

from .MPCControl_base import MPCControl_base


class MPCControl_zvel(MPCControl_base):
    x_ids: np.ndarray = np.array([8])
    u_ids: np.ndarray = np.array([2])

    def _setup_controller(self) -> None:
        #################################################
        # YOUR CODE HERE

        # Augmented matrices construction

        A_aug = np.block([
            [self.A, self.B],
            [np.zeros((1, self.nx)), np.eye(1)]
        ])

        B_aug = np.block([
            [self.B],
            [np.zeros((1, self.nu))]
        ])

        
        

        self.A_aug = A_aug 
        self.B_aug = B_aug

        self.nx_orig = self.nx #Old dimension save
        self.nx = self.A_aug.shape[0] #New dimension

        x_var=cp.Variable((self.nx,self.N+1))
        u_var=cp.Variable((self.nu,self.N))


        

        #State constraints -> None
        

        #Input constraints
        M = np.array([[1], [-1]])
        m = np.array([80-self.us[0],-(40-self.us[0])])
    


        #Initial state
        x0_param=cp.Parameter(self.nx) # With the new size
        x_targ_param=cp.Parameter(self.nx) # With the new size
        u_ref_param=cp.Parameter(self.nu)

        # Costs
        Q_aug = np.zeros((self.nx, self.nx))
        Q_aug[0, 0] = 50.0
        Q_aug[1,1]=10
        Q = Q_aug 

        
        R=np.eye(self.nu)*0.1
        
       
        cost = 0
        for i in range(self.N):
            cost += cp.quad_form(x_var[:, i] - x_targ_param, Q)
            cost += cp.quad_form(u_var[:, i] - u_ref_param, R)
        cost += cp.quad_form(x_var[:, self.N] - x_targ_param, Q)


        constraints = []

        # Initial condition

        constraints.append(x_var[:, 0] == x0_param)

        # System dynamics
        
        constraints.append(x_var[:,1:] == A_aug @ x_var[:,:-1] + B_aug @ u_var)

        # State constraints -> None
        

        # Input constraints
        constraints.append(M @ u_var <= m.reshape(-1, 1))



        self.ocp = cp.Problem(cp.Minimize(cost), constraints)

        # Sauvegardes
        self.x0_param = x0_param 
        self.x=x_var
        self.u=u_var
        self.x_targ=x_targ_param
        self.u_ref=u_ref_param
        self.d_est_values=[]

        #---Luenberger Observer---#

        C_aug=np.array([[1,0]])

        self.C_aug = C_aug

        #Choice of poles
        p_obs = np.array([0.7, 0.75]) 

        #Calculation of the L gain
        try:
            L_transpose = place(self.A_aug.T, self.C_aug.T, p_obs)
            self.L = L_transpose.T 
        except ValueError:
            print("WARNING: Observer poles could not be placed. Using zero gain.")
            self.L = np.array([[0.5], [0.1]])

        self.x_hat = np.zeros((self.nx, 1)) 
        self.u_prev = np.zeros((self.nu, 1))

        # YOUR CODE HERE
        #################################################

    #################################################
    def get_u(
        self, x0: np.ndarray, x_target: np.ndarray = None, u_target: np.ndarray = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        #################################################
        # YOUR CODE HERE

        delta_x0 = (x0 - self.xs).reshape(-1, 1)


        #Prediction 
        x_hat_predicted = self.A_aug @ self.x_hat + self.B_aug @ self.u_prev

        
        
        # Correction
        y_hat_k = self.C_aug @ x_hat_predicted
        innov = delta_x0 - y_hat_k 
        x_aug_estimated = x_hat_predicted + self.L @ innov

           

        # Save
        self.x_hat = x_aug_estimated 

        
        d_est = x_aug_estimated[1, 0].item()

        self.d_est_values.append(d_est)
        
        x_ref = (x_target - self.xs).item() if x_target is not None else 0.0
        
    
        self.x0_param.value = x_aug_estimated.flatten()
        self.x_targ.value = np.array([x_ref, d_est]).flatten()
        
        
        self.u_ref.value = np.array([-d_est])
        
        
        self.ocp.solve(solver=cp.PIQP)
        assert self.ocp.status == cp.OPTIMAL, "Pas reussi a resoudre"

        delta_u0 = self.u.value[:, 0]
        
        
        self.u_prev = delta_u0.reshape(-1, 1) 
        

        u0 = delta_u0 + self.us
        x_traj = self.x.value[:self.nx_orig, :] + self.xs.reshape(-1, 1)
        u_traj = self.u.value + self.us.reshape(-1, 1)

        return u0, x_traj, u_traj

        

        
            
        
        

        

        