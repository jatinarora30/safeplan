
"""
@file baseenv.py
@brief Base class for Environment making of planning algorithms.

@details
Provides a common interface for environment making 
"""
from abc import ABC
from typing import Any, List, Sequence, Tuple



class BaseEnv(ABC):
    """
    @class BaseEnv
    @brief Abstract base/registry for environment making for planning algorithms

    @details
    Responsibilities:
    - Defines the `getmap()` interface used by all map generating classes
    - Establishes the common return contract `grid ` for that classes

    @note Subclasses must implement @ref getmap().
    """
    
    def __init__(self) -> None:
        
        """
        @brief Construct the base map generation.

        @post Instance is initialized; subclass constructors may call super().
        """
        
        print("Intializing Base Map Generation ....")
        
    def getmap(self) -> Any:
        """
        A common getmap function needs to be incorporated by every map generating class
        @return Grid An n-dimensional grid 
        @throws NotImplementedError If a subclass does not override this method.
        """
        
        raise NotImplementedError("Subclasses must implement getmap()")
        
        
        
    
        