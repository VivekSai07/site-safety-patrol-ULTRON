"""Spatial memory: object appearance re-identification and queryable place memory.

Advisory sidecar inspired by DimOS spatial memory (Apache-2.0,
github.com/dimensionalOS/dimos); no DimOS code is vendored.
"""
from .embedder import load_embedder
from .hook import AssociationAdvisor
from .memory import SpatialConfig, SpatialMemory

__all__ = ["AssociationAdvisor", "SpatialConfig", "SpatialMemory", "load_embedder"]
