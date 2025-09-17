# baseenv.py
"""
@file baseenv.py
@brief Base class for environment generation used by planning algorithms.

@details
Provides a common interface for environment builders that produce N-dimensional
occupancy grids and associated metadata. Subclasses implement @ref getmap() to
return a grid and start/goal pairs in a consistent format consumed by SafePlan.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Sequence, Tuple
import sys


class BaseEnv(ABC):
    """
    @class BaseEnv
    @brief Abstract base for environment generation.

    @details
    Responsibilities:
    - Defines the @ref getmap() interface used by all map-generating classes.
    - Establishes a common return contract for grids and metadata.

    @note Subclasses must implement @ref getmap().
    """

    def __init__(self) -> None:
        """
        @brief Construct the base environment generator.

        @post Instance is initialized; subclass constructors may call super().
        """
        # Keep lightweight; real work happens in subclass and getmap()
        pass

    @abstractmethod
    def getmap(self) -> Tuple[Any, float, str, str, List[Dict[str, Tuple[int, ...]]]]:
        """
        @brief Build or load an occupancy grid and related metadata.

        @details
        Implementations should return:
        - grid : numpy.ndarray (N-D), with values {0=free, 1=obstacle}
        - cellSize : float — physical size of a grid cell (meters or chosen unit)
        - envName : str — short environment name
        - envDes  : str — brief environment description
        - startGoalPairs : list of dicts, each like:
              {"start": (i, j[, k, ...]), "goal": (i, j[, k, ...])}

        @return tuple
                (grid, cellSize, envName, envDes, startGoalPairs)

        @throws NotImplementedError
                If a subclass does not override this method.
        """
        raise NotImplementedError("Subclasses must implement getmap()")
