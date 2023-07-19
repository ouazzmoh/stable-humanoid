from typing import Iterator
import os
import h5py as h5
import numpy as np
import scipy.io as spio
import scipy.sparse as spa
from typing import Dict, Iterator, Union

# from qpsolvers import Problem
from qpsolvers_benchmark.problem import Problem
from qpsolvers_benchmark.test_set import TestSet
from qpsolvers_benchmark.tolerance import Tolerance


class MpcQp(TestSet):
    """
    HDF5 Testset of qp problems related to robots movements.
    """

    @property
    def description(self) -> str:
        """Description of the test set."""
        return "hdf5 test set, contains qp problems related to robots movements"

    def __init__(self):
        """Initialize test set.

        Args:
            data_dir: Path to the benchmark data directory.
        """
        super().__init__()
        data_path = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(data_path, "..", "data")
        self.optimal_cost = 1e-4

    @property
    def title(self) -> str:
        """Test set title."""
        return "mpc and inverse kinematics test set"

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

    @property
    def sparse_only(self) -> bool:
        """Test set is dense."""
        return False

    def yield_sequence_from_file(self, file):
        """
        Yield sequence of problems from a file.

        Args:
            file (h5py.File): The file object from which to yield the problems.

        Yields:
            qpsolvers.Problem : A problem object containing the data from the file.
        """
        for group_name in file:
            group = file[group_name]
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
                        os.path.split(file.filename)[1].replace(".hdf5", "")
                        + "_"
                        + str(group)
                        + problem
                    )
                for array_name in ["A", "b", "lb", "ub"]:
                    if array_name not in qp_data.keys():
                        qp_data[array_name] = None
                qp_problem = Problem(**qp_data)
                qp_problem.optimal_cost = self.optimal_cost
                yield qp_problem

    def __iter__(self) -> Iterator[Problem]:
        """Iterate over test set problems."""
        for filename in os.listdir(self.data_dir):
            if not filename.endswith(".hdf5"):
                continue
            filepath = os.path.join(self.data_dir, filename)
            with h5.File(filepath, "r") as file:
                yield from self.yield_sequence_from_file(file)
