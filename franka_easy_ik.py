
from copy import deepcopy
from typing import Iterable
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion

class FrankaEasyIK():
    def __init__(self):
        self.robot = rtb.models.Panda()
        self.last_q = None

    def __call__(self, p: Iterable[float], q: Iterable[float] = [1., 0., 0., 0.], verbose=False) -> Iterable[float]:
        """ do custom inverse kinematics

        Args:
            p (Float[3]): Cartesian Position
            q (Float[4]): Absolute Quaternion Orienation w.r.t. robot base
                - quternion notation: x,y,z,w
            verboser (bool): Print results
            
        Raises:
            Exception: When IK not found

        Returns:
            Float[7]: 7 DoF robot joint configuration
        """
        assert len(p) == 3, f"position length: {len(p)} != 3"
        assert len(q) == 4, f"quaternion length: {len(q)} != 4"
        
        p = deepcopy(p)
        q = deepcopy(q)
        q[1], q[2], q[3], q[0] = q[0], q[1], q[2], q[3] 

        sol = self.robot.ikine_LM(SE3.Trans(*p) * UnitQuaternion(np.array(q)).SE3(), q0=self.last_q)
        q1 = sol.q
        succ = sol.success
        reason = sol.reason
        
        if not succ:
            raise Exception(f"IK not found because: {reason}")
        if verbose:
            print("last q before: ", self.last_q)
        self.last_q = q1
        if verbose:
            print("last q: ", self.last_q)
        return q1
