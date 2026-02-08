import numpy as np
import cvxpy as cp
from control import dlqr
from mpt4py import Polyhedron

from .MPCControl_base import MPCControl_base


class MPCControl_zvel(MPCControl_base):
    x_ids: np.ndarray = np.array([8])
    u_ids: np.ndarray = np.array([2])

    def _setup_controller(self) -> None:
        #################################################
        # YOUR CODE HERE

        x_var=cp.Variable((self.nx,self.N+1))
        u_var=cp.Variable((self.nu,self.N))

        

        #State constraints -> None
        

        #Input constraints
        M = np.array([[1], [-1]])
        m = np.array([80-self.us[0],-(40-self.us[0])])
        

        #Initial state
        x0_param=cp.Parameter(self.nx)

        # Costs
        Q=np.eye(self.nx)*50
        R=np.eye(self.nu)*10
        K, Qf, _ = dlqr(self.A, self.B, Q, R)
        K = -K

        # Computation of the maximum invariant set

        F_combined=M@K
        f_combined=m
        X=Polyhedron.from_Hrep(F_combined,f_combined)
        O_inf=self.max_invariant_set(self.A+self.B@K,X)

        self.X_f=O_inf

        cost = 0
        for i in range(self.N):
            cost += cp.quad_form(x_var[:,i], Q)
            cost += cp.quad_form(u_var[:,i], R)

        # Terminal cost 

        cost+= cp.quad_form(x_var[:,self.N],Qf)

        constraints = []

        # Initial condition

        constraints.append(x_var[:, 0] == x0_param)

        # System dynamics
        
        constraints.append(x_var[:,1:] == self.A @ x_var[:,:-1] + self.B @ u_var)

        # State constraints -> None
        

        # Input constraints
        constraints.append(M @ u_var <= m.reshape(-1, 1))

        # Terminal Constraints 

        constraints.append(O_inf.A @ x_var[:,-1] <= O_inf.b.reshape(-1,1))


        self.ocp = cp.Problem(cp.Minimize(cost), constraints)

        # Sauvegardes
        self.x0_param = x0_param 
        self.x=x_var
        self.u=u_var

        # YOUR CODE HERE
        #################################################

    def get_u(
        self, x0: np.ndarray, x_target: np.ndarray = None, u_target: np.ndarray = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        #################################################
        # YOUR CODE HERE

        delta_x0=x0-self.xs

        self.x0_param.value=delta_x0


        self.ocp.solve(solver=cp.PIQP)
        assert self.ocp.status == cp.OPTIMAL, "Pas reussi a resoudre"

        x_opt=self.x.value
        u_opt=self.u.value

        delta_u0 = u_opt[:, 0]           
  

        u0 = delta_u0 + self.us
        x_traj = x_opt +self.xs.reshape(-1,1)
        u_traj = u_opt + self.us.reshape(-1,1)

        # YOUR CODE HERE
        #################################################

        return u0, x_traj, u_traj