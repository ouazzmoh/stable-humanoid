from typing import Optional, Sequence, Tuple, Union

import numpy as np
import pinocchio as pin

from pink.configuration import Configuration
from pink.tasks.exceptions import TargetNotSet
from pink.tasks.task import Task

class ComTask(Task):
    cost: np.ndarray
    lm_damping: float
    transform_target_to_world: Optional[pin.SE3]
    def __init__(
        self,
        position_cost: Union[float, Sequence[float]],
        lm_damping: float = 1e-6,
    ) -> None:
        r"""Define a new body task.

        Args:
            body: Name of the body frame to move to the target pose.
            position_cost: Contribution of position errors to the normalized
                cost, in :math:`[\mathrm{cost}] / [\mathrm{m}]`. If this is a
                vector, the cost is anisotropic and each coordinate corresponds
                to an axis in the local body frame.
            lm_damping: Levenberg-Marquardt damping (see class attributes). The
                default value is conservatively low.
        """
        self.cost = np.ones(3)
        self.lm_damping = lm_damping
        self.transform_target_to_world = None
        #
        self.set_position_cost(position_cost)

    def set_position_cost(
        self, position_cost: Union[float, Sequence[float]]
    ) -> None:
        r"""Set a new cost for all 3D position coordinates.

        Args:
            position_cost: Contribution of position errors to the normalized
                cost, in :math:`[\mathrm{cost}] / [\mathrm{m}]`. If this is a
                vector, the cost is anisotropic and each coordinate corresponds
                to an axis in the local body frame.
        """
        if isinstance(position_cost, float):
            assert position_cost >= 0.0
        else:  # not isinstance(position_cost, float)
            assert all(cost >= 0.0 for cost in position_cost)
        self.cost[0:3] = position_cost
    def set_target_from_configuration(
        self, configuration: Configuration
    ) -> None:
        """Set task target pose from a robot configuration.

        Args:
            configuration: Robot configuration.
        """
        self.set_target(pin.SE3(np.eye(3), np.array(pin.centerOfMass(configuration.model, configuration.data, configuration.q))))
    
    def set_target(
        self,
        transform_target_to_world: np.ndarray,
    ) -> None:
        """Set task target pose in the world frame.

        Args:
            transform_target_to_world: Transform from the task target frame to
                the world frame.
        """
        self.transform_target_to_world = transform_target_to_world.copy()

    def compute_error(self, configuration: Configuration) -> np.ndarray:
        """Compute COM task error.

        Args:
            configuration: Robot configuration.

        Returns:
            COM task error.
        """
        if self.transform_target_to_world is None:
            raise TargetNotSet("no target position set for COM")
        current_com_position = pin.centerOfMass(configuration.model,
                                                configuration.data,
                                                configuration.q)
        error =  - current_com_position + self.transform_target_to_world.translation
        return error
    
    def compute_jacobian(self, configuration: Configuration) -> np.ndarray:
        return pin.jacobianCenterOfMass(configuration.model, configuration.data, configuration.q)

    def compute_qp_objective(
        self, configuration: Configuration
    ) -> Tuple[np.ndarray, np.ndarray]:
        r"""Compute the matrix-vector pair :math:`(H, c)` of the QP objective.

        This pair is such that the contribution of the task to the QP objective
        of the IK is:

        .. math::

            \| J \Delta q - \alpha e \|_{W}^2
            = \frac{1}{2} \Delta q^T H \Delta q + c^T q

        The weight matrix :math:`W \in \mathbb{R}^{6 \times 6}` combines
        position and orientation costs. The unit of the overall contribution is
        :math:`[\mathrm{cost}]^2`. The configuration displacement
        :math:`\Delta q` is the output of inverse kinematics (we divide it by
        :math:`\Delta t` to get a commanded velocity).

        Args:
            configuration: Robot configuration :math:`q`.

        Returns:
            Pair :math:`(H(q), c(q))` of Hessian matrix and linear vector of
            the QP objective.

        See Also:
            Levenberg-Marquardt damping is described in [Sugihara2011]_. The
            dimensional analysis in this class is our own.
        """
        jacobian = self.compute_jacobian(configuration)
        gain_error = self.gain * self.compute_error(configuration)
        weight = np.diag(self.cost)  # [cost] * [twist]^{-1}
        weighted_jacobian = weight @ jacobian  # [cost]
        weighted_error = weight @ gain_error  # [cost]
        mu = self.lm_damping * weighted_error @ weighted_error  # [cost]^2
        eye_tg = configuration.tangent.eye
        # Our Levenberg-Marquardt damping `mu * eye_tg` is isotropic in the
        # robot's tangent space. If it helps we can add a tangent-space scaling
        # to damp the floating base differently from joint angular velocities.
        H = weighted_jacobian.T @ weighted_jacobian + mu * eye_tg
        c = -weighted_error.T @ weighted_jacobian
        return (H, c)