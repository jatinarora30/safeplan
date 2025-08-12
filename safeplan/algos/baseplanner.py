
"""
@file baseplanner.py
@brief Base class for grid planning algorithms.

@details
Provides a common interface for planners that compute a path on an
N-D occupancy grid from a start index to a goal index.
"""
from abc import ABC
from typing import Any, List, Sequence, Tuple

Coord=Tuple[int,...]
Path= list(Coord)

class BasePlanner(ABC):
    """
    @class BasePlanner
    @brief Abstract base/registry for planning algorithms

    @details
    Responsibilities:
    - Defines the `plan()` interface used by all planners.
    - Establishes the common return contract `(success, path, info)`.

    @note Subclasses must implement @ref plan().
    """
    
    def __init__(self) -> None:
        
        """
        @brief Construct the base planner.

        @post Instance is initialized; subclass constructors may call super().
        """
        
        print("Intializing Base Planner ....")
        
    def plan(self,
             start:Sequence[int],
             goal: Sequence[int],
             grid: Any) ->Tuple[int, Path, List[str]]:
        """
        A common Plan function needs to be incorporated by every planner returns Path, Sucess, info
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @return success Tells if the path was found( as 1 ) or not ( as 0 )
        @return Path Returns the path from star to goal in the form of a tuple
        @return info Returns list of statements of what may may went wrong in finding path from start to goal
        @throws NotImplementedError If a subclass does not override this method.
        """
        
        raise NotImplementedError("Subclasses must implement plan()")
        
        
        
    
        