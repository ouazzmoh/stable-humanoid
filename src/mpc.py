from mpc_qp import MpcQp
from qpsolvers_benchmark.problem import Problem
from qpsolvers_benchmark.tolerance import Tolerance
import numpy as np
import os


class Mpc(MpcQp):
    @property
    def description(self) -> str:
        """Description of the test set."""
        return "Subset of the mpc_qp test set "

    @property
    def title(self) -> str:
        """Test set title."""
        return "mpc subset"

    @property
    def sparse_only(self) -> bool:
        """Test set is dense."""
        return False

    def define_tolerances(self) -> None:
        """Define test set tolerances."""
        self.tolerances = {
            "default": Tolerance(
                cost=1e-4,
                primal=1e-3,
                dual=1e-3,
                gap=1e-3,
                runtime=1,
            ),
            "low_accuracy": Tolerance(
                cost=1e-3,
                primal=1e-3,
                dual=1e-3,
                gap=1e-4,
                runtime=1,
            ),
            "high_accuracy": Tolerance(
                cost=1e-5,
                primal=1e-7,
                dual=1e-7,
                gap=1e-3,
                runtime=1,
            ),
        }

    def yield_sequence_from_file(self, file):
        """
        Yield sequence of problems from a file.

        Args:
            file: The file object from which to yield the problems.

        Yields:
            qpsolvers.Problem : A problem object containing the data from the file.
        """
        group = file["mpc_problems"]
        for problem in list(group.keys()):
            qp_data = {}
            for data_name in group[problem].keys():
                if f"{data_name}/data" in group[problem]:
                    data = group[f"{problem}/{data_name}/data"][:]
                    data[data > 9e19] = +np.inf
                    data[data < -9e19] = -np.inf
                    qp_data[data_name] = data
                else:
                    qp_data[data_name] = None
                qp_data["name"] = (
                    os.path.split(file.filename)[1].replace(".hdf5", "") + "_" + problem
                )
                for array_name in ["A", "b", "lb", "ub"]:
                    if array_name not in qp_data.keys():
                        qp_data[array_name] = None
            qp_problem = Problem(**qp_data)
            qp_problem.optimal_cost = self.optimal_cost
            yield qp_problem
