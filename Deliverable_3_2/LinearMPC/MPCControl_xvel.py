import numpy as np

import cvxpy as cp
from control import dlqr
from mpt4py import Polyhedron

from .MPCControl_base import MPCControl_base


class MPCControl_xvel(MPCControl_base):
    x_ids: np.ndarray = np.array([1, 4, 6]) 
    u_ids: np.ndarray = np.array([1])

    def _setup_controller(self) -> None:
        #################################################
        # YOUR CODE HERE
        x_var=cp.Variable((self.nx,self.N+1))
        u_var=cp.Variable((self.nu,self.N))
        

        #State constraints
        F=np.array([[0,1,0],[0,-1,0]])
        f=np.array([0.1745,0.1745])
        

        #Input constraints
        M = np.array([[1], [-1]])
        m = np.array([0.26,0.26])
        

        #Initial state
        x0_param=cp.Parameter(self.nx)
        x_targ_param=cp.Parameter(self.nx)
        u_ref_param=cp.Parameter(self.nu)

        # Costs
        
        Q=np.eye(self.nx)*10
        R=np.eye(self.nu)*10
        K, Qf, _ = dlqr(self.A, self.B, Q, R)
        K = -K

        # Computation of the maximum invariant set

        F_combined=np.vstack([F,M@K])
        f_combined = np.concatenate((f, m))
        X=Polyhedron.from_Hrep(F_combined,f_combined)
        O_inf=self.max_invariant_set(self.A+self.B@K,X)


        cost = 0
        for i in range(self.N):
            cost += cp.quad_form(x_var[:,i]-x_targ_param, Q)
            cost += cp.quad_form(u_var[:,i]-u_ref_param, R)
            

        # Terminal cost 

        cost+= cp.quad_form(x_var[:,self.N]-x_targ_param,Qf)


        constraints = []

        # Initial condition

        constraints.append(x_var[:, 0] == x0_param)

        # System dynamics
        
        constraints.append(x_var[:,1:] == self.A @ x_var[:,:-1] + self.B @ u_var)

        
        constraints.append(F @ x_var[:, :-1] <= f.reshape(-1, 1))

        # Input constraints
        constraints.append(M @ u_var <= m.reshape(-1, 1))

        # Terminal Constraints 

        constraints.append(O_inf.A @ x_var[:,-1] <= O_inf.b.reshape(-1,1))


        self.ocp = cp.Problem(cp.Minimize(cost), constraints)

        # Sauvegardes
        self.x0_param = x0_param 
        self.x=x_var
        self.u=u_var
        self.x_targ=x_targ_param
        self.u_ref=u_ref_param


        # YOUR CODE HERE
        #################################################

    def get_u(
        self, x0: np.ndarray, x_target: np.ndarray = None, u_target: np.ndarray = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        #################################################
        # YOUR CODE HERE

        
        delta_x0=x0-self.xs

        self.x0_param.value=delta_x0

        if x_target is None :
            x_ref = np.zeros(self.nx)
        else :
            x_ref=x_target-self.xs
        
        self.x_targ.value=x_ref

        # Solving B * u_ref = (I - A) * x_ref for the feedforward

        u_ref= np.linalg.pinv(self.B)@(np.eye(self.nx)-self.A)@x_ref

    
        self.u_ref.value=u_ref

        
        
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