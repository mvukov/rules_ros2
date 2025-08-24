from __future__ import annotations

import dataclasses
import pathlib


@dataclasses.dataclass(frozen=True, kw_only=True)
class EnvironmentVariable:
    name: str
    value: str


@dataclasses.dataclass(frozen=True, kw_only=True)
class Node:
    name: str
    executable: str | pathlib.Path
    parameters_file: str | pathlib.Path | None = None
    remappings: list[tuple[str, str]] | None = None
    ros_arguments: list[str] | None = None
    arguments: list[str] | None = None
    # env: dict[str, str] | None = None


@dataclasses.dataclass(frozen=True, kw_only=True)
class ParametersFile:
    path: str | pathlib.Path


@dataclasses.dataclass(frozen=True)
class Deployment:
    entities: list[Deployment | EnvironmentVariable | Node | ParametersFile]


Entity = Deployment | EnvironmentVariable | Node | ParametersFile
