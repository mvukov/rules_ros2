from __future__ import annotations

import dataclasses
import pathlib


@dataclasses.dataclass(frozen=True, kw_only=True)
class EnvironmentVariable:
    name: str
    value: str


@dataclasses.dataclass(frozen=True, kw_only=True)
class Process:
    name: str
    executable: str | pathlib.Path
    arguments: list[str] | None = None


@dataclasses.dataclass(frozen=True, kw_only=True)
class RosNode(Process):
    parameters_file: str | pathlib.Path | None = None
    remappings: list[tuple[str, str]] | None = None
    ros_arguments: list[str] | None = None


@dataclasses.dataclass(frozen=True, kw_only=True)
class ParametersFile:
    path: str | pathlib.Path


@dataclasses.dataclass(frozen=True)
class Deployment:
    entities: list[Deployment | EnvironmentVariable | RosNode | ParametersFile]


Entity = Deployment | EnvironmentVariable | Process | RosNode | ParametersFile
