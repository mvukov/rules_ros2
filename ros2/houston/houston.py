import collections
import dataclasses
import pathlib

import yaml


@dataclasses.dataclass(frozen=True)
class EnvironmentVariable:
    name: str
    value: str

    def validate(self):
        pass


@dataclasses.dataclass(frozen=True)
class Node:
    name: str
    executable: str | pathlib.Path
    parameters_file: str | pathlib.Pathlib | None = None
    remappings: list[tuple[str, str]] | None = None
    ros_arguments: list[str] | None = None
    arguments: list[str] | None = None

    # env: dict[str, str] | None = None

    def validate(self):
        pass


@dataclasses.dataclass(frozen=True)
class ParametersFile:
    path: str | pathlib.Path

    def validate(self):
        pass


@dataclasses.dataclass(frozen=True)
class Deployment:
    Entity = 'Deployment' | EnvironmentVariable | Node | ParametersFile
    entities: list[Entity]


def flatten(src: Deployment) -> Deployment:
    dst_entities: list[Deployment.Entity] = []
    src_entities: collections.deque[Deployment.Entity] = collections.deque(
        src.entities)
    while len(src_entities) > 0:
        match src_entity := src_entities.popleft():
            case Deployment(entities):
                for entity in entities:
                    src_entities.appendleft(entity)
            case EnvironmentVariable() | Node() | ParametersFile():
                dst_entities.append(src_entity)
            case _:
                raise TypeError(
                    f'Got unsupported entity of type {type(entity)}')
    return Deployment(dst_entities)


def validate(deployment: Deployment):
    for entity in deployment.entities:
        match entity:
            case Deployment():
                pass
            case EnvironmentVariable() | Node() | ParametersFile():
                entity.validate()
            case _:
                raise TypeError(
                    f'Got unsupported entity of type {type(entity)}')


def collect_env(deployment: Deployment) -> dict[str, str]:
    env: dict[str, str] = {}
    for entity in deployment.entities:
        match entity:
            case EnvironmentVariable(name, value):
                env[name] = value
            case _:
                pass
    return env


def collect_parameters(deployment: Deployment, dst: pathlib.Path):
    params_files: list[pathlib.Path] = []
    for entity in deployment.entities:
        match entity:
            case Node(parameters_file):
                params_files.append(pathlib.Path(parameters_file))
            case ParametersFile(path):
                params_files.append(pathlib.Path(path))
            case _:
                pass

    # TODO(mvukov) Merge parameter files.
    merged_params = {}
    with open(dst, 'w', encoding='utf-8') as stream:
        yaml.dump(merged_params, stream)


def create_node_command(node: Node, merged_params_file: pathlib.Path) -> str:
    command_parts = [node.executable]
    command_parts.append(f'__name:={node.name}')
    command_parts.extend(['--params-file', merged_params_file])

    for src, dst in node.remappings:
        command_parts.extend(['-r', f'{src}:={dst}'])

    if node.ros_arguments is not None and node.ros_arguments:
        command_parts.extend(node.ros_arguments)
    if node.arguments is not None and node.arguments:
        command_parts.extend(node.arguments)

    return ' '.join(command_parts)
