import numpy as np
import cvxpy as cp
from control import dlqr
from mpt4py import Polyhedron

from .MPCControl_base import MPCControl_base


class MPCControl_z(MPCControl_base):
    x_ids: np.ndarray = np.array([8, 11])
    u_ids: np.ndarray = np.array([2])

    # only useful for part 5 of the project
    d_estimate: np.ndarray
    d_gain: float

    def _setup_controller(self) -> None:
        #################################################
        # YOUR CODE HERE

        x_var=cp.Variable((self.nx,self.N+1))
        u_var=cp.Variable((self.nu,self.N))

        # State constraints 
        
        F=np.array([[0,-1]])
        f=np.array([3.0])
        X=Polyhedron.from_Hrep(F,f)

        # Input constraints
        M = np.array([[1], [-1]])
        m = np.array([80-self.us[0],-(40-self.us[0])])
        U=Polyhedron.from_Hrep(M,m)

        # Perturbation space
        F_w=np.array([[1],[-1]])
        f_w=np.array([5,15])
        W =Polyhedron.from_Hrep(F_w,f_w)

        
        # Initial state
        x0_param=cp.Parameter(self.nx)
        x_targ_param=cp.Parameter(self.nx)
        u_ref_param=cp.Parameter(self.nu)

        # Costs
        
        Q=np.eye(self.nx)*100
        Q[0,0]=50
        R=np.eye(self.nu)*0.1
        K, Qf, _ = dlqr(self.A, self.B, Q, R)
        K = -K
    

        # Computation of the Minimal Robust Invariant Set

        E=self.min_robust_invariant_set(self.A+self.B@K,self.B@W)

        # Minkowski substraction

        X_tight=X-E
        U_tight=U-K@E


        # Computation of the maximum invariant set

        O_inf=self.max_invariant_set(self.A+self.B@K,X_tight)

        
        cost = 0
        for i in range(self.N):
            cost += cp.quad_form(x_var[:,i]-x_targ_param, Q)
            cost += cp.quad_form(u_var[:,i]-u_ref_param, R)

        # Terminal cost 

        cost+= cp.quad_form(x_var[:,self.N]-x_targ_param,Qf)

        constraints = []

        # Initial condition

        constraints.append(E.A @ (x0_param - x_var[:, 0]) <= E.b)

        # System dynamics
        
        constraints.append(x_var[:,1:] == self.A @ x_var[:,:-1] + self.B @ u_var)

        # State constraints 

        constraints.append(X_tight.A @ x_var[:, :] <= X_tight.b.reshape(-1, 1))

        
        # Input constraints
        constraints.append(U_tight.A @ u_var <= U_tight.b.reshape(-1, 1))

        # Terminal Constraints 

        constraints.append(O_inf.A @ x_var[:,-1] <= O_inf.b.reshape(-1,1))


        self.ocp = cp.Problem(cp.Minimize(cost), constraints)

        # Sauvegardes
        self.x0_param = x0_param 
        self.x=x_var
        self.u=u_var
        self.x_targ=x_targ_param
        self.u_ref=u_ref_param
        self.K=K

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

        x_nom_opt=self.x.value
        u_nom_opt=self.u.value

        x_nom_opt0=x_nom_opt[:,0]
        u_nom_opt0=u_nom_opt[:,0]  

         


        # Feedback
    
        u_feedback = self.K @ (delta_x0 - x_nom_opt0)
        delta_u0 = u_nom_opt0 + u_feedback
  
        

        u0 = delta_u0 + self.us
        x_traj = x_nom_opt +self.xs.reshape(-1,1)
        u_traj = u_nom_opt + self.us.reshape(-1,1)

        # YOUR CODE HERE
        #################################################

        return u0, x_traj, u_traj

    def setup_estimator(self):
        # FOR PART 5 OF THE PROJECT
        ##################################################
        # YOUR CODE HERE

        self.d_estimate = ...
        self.d_gain = ...

        # YOUR CODE HERE
        ##################################################

    def update_estimator(self, x_data: np.ndarray, u_data: np.ndarray) -> None:
        # FOR PART 5 OF THE PROJECT
        ##################################################
        # YOUR CODE HERE
        self.d_estimate = ...
        # YOUR CODE HERE
        ##################################################
