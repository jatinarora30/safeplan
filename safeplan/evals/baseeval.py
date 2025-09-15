
"""
@file baseeval.py
@brief Base class for evaluation metrices  of planning algorithms.

@details
Provides a common interface for evaluation from start, goal and grid 
"""
from abc import ABC
from typing import Any, Sequence, Tuple

Coord=Tuple[int,...]
Path= list[Coord]

class BaseEval(ABC):
    """
    @class BaseEval
    @brief Abstract base/registry for  evaluating planning algorithms

    @details
    Responsibilities:
    - Defines the `eval()` interface used by all evaulation metrices classes.
    - Establishes the common return contract `value ` for that metrices.

    @note Subclasses must implement @ref eval().
    """
    
    def __init__(self) -> None:
        
        """
        @brief Construct the base evaluation.

        @post Instance is initialized; subclass constructors may call super().
        """
        
        print("Intializing Base Evaluation ....")
        
    def eval(self,
             start:Sequence[int],
             goal: Sequence[int],
             grid: Any,
             cellSize:float,
             path: Path) ->Any:
        """
        A common eval function needs to be incorporated by every evaluation class
        @param start Takes the n-dimensional start input
        @param goal Takes the n-dimension goal input
        @param grid Takes the N x N dimensional grid
        @param path Tkes the path that was returned by planning algorithm
        @return val Returns  the value of evaluated metric
        @throws NotImplementedError If a subclass does not override this method.
        """
        
        raise NotImplementedError("Subclasses must implement eval()")
        
        
        
    
        