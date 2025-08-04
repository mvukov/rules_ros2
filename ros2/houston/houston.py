from __future__ import annotations

import collections
import dataclasses
import importlib.util
import os
import pathlib

import deepmerge
import toml
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
    parameters_file: str | pathlib.Path | None = None
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
        if not os.path.exists(self.path):
            raise ValueError(f'{self.path} does not exist')


@dataclasses.dataclass(frozen=True)
class Deployment:
    entities: list[Deployment | EnvironmentVariable | Node | ParametersFile]


Entity = Deployment | EnvironmentVariable | Node | ParametersFile


def flatten(src: Deployment) -> Deployment:
    dst_entities: list[Entity] = []
    src_entities: collections.deque[Entity] = collections.deque(src.entities)
    while len(src_entities) > 0:
        match src_entity := src_entities.pop():
            case Deployment(entities):
                for entity in entities:
                    src_entities.appendleft(entity)
            case EnvironmentVariable() | Node() | ParametersFile():
                dst_entities.append(src_entity)
            case _:
                raise TypeError(
                    f'Got unsupported entity of type {type(entity)}')
    print(dst_entities)
    return Deployment(dst_entities)


def validate(deployment: Deployment):
    for entity in deployment.entities:
        match entity:
            case EnvironmentVariable() | Node() | ParametersFile():
                entity.validate()
            case _:
                pass


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
            case Node():
                params_files.append(pathlib.Path(entity.parameters_file))
            case ParametersFile():
                params_files.append(pathlib.Path(entity.path))
            case _:
                pass

    if not params_files:
        return

    with open(params_files[0], encoding='utf-8') as stream:
        merged_params = yaml.load(stream, yaml.SafeLoader)

    if len(params_files) > 1:
        for params_file in params_files[1:]:
            with open(params_file, encoding='utf-8') as stream:
                merged_params = deepmerge.always_merger.merge(
                    merged_params, yaml.load(stream, yaml.SafeLoader))

    with open(dst, 'w', encoding='utf-8') as stream:
        yaml.dump(merged_params, stream)


@dataclasses.dataclass(frozen=True)
class CommandConfig:
    only_env: list[str] | None
    program: str
    args: list[str]


@dataclasses.dataclass(frozen=True)
class ProcessConfig:
    name: str
    run: CommandConfig
    stop: str


def create_node_process_config(node: Node, merged_params_file: pathlib.Path,
                               only_env: list[str] | None) -> ProcessConfig:
    args = ['--ros-args', '-r', f'__node:={node.name}']
    args.extend(['--params-file', merged_params_file])

    if node.remappings is not None:
        for src, dst in node.remappings:
            args.extend(['-r', f'{src}:={dst}'])

    if node.ros_arguments is not None and node.ros_arguments:
        args.extend(node.ros_arguments)
    if node.arguments is not None and node.arguments:
        args.extend(node.arguments)

    return ProcessConfig(name=node.name,
                         run=CommandConfig(only_env=only_env,
                                           program=node.executable,
                                           args=args),
                         stop='SIGINT')


def collect_process_configs(
        deployment: Deployment,
        merged_params_file: pathlib.Path,
        only_env: list[str] | None = None) -> list[ProcessConfig]:
    process_configs: list[ProcessConfig] = []
    for entity in deployment.entities:
        match entity:
            case Node():
                process_configs.append(
                    create_node_process_config(entity, merged_params_file,
                                               only_env))
            case _:
                pass
    return process_configs


def dump_process_configs_to_groundcontrol_processes(
        process_configs: list[ProcessConfig]) -> list[dict]:
    result = []
    for cfg in process_configs:
        run = {}
        if cfg.run.only_env is not None:
            run['only-env'] = cfg.run.only_env
        run['command'] = (
            f'{cfg.run.program} {" ".join([str(arg) for arg in cfg.run.args])}')

        result.append({
            'name': f'{cfg.name}',
            'run': run,
            'stop': f'{cfg.stop}',
        })
    return result


def create_groundcontrol_config(
    deployments: list[Deployment],
    merged_params_exec_path: pathlib.Path,
    merged_params_root_path: pathlib.Path,
) -> dict:
    deployment = Deployment(deployments)
    validate(deployment)
    flattened_deployment = flatten(deployment)

    env = collect_env(flattened_deployment)
    collect_parameters(flattened_deployment, merged_params_exec_path)
    process_configs = collect_process_configs(flattened_deployment,
                                              merged_params_root_path,
                                              only_env=list(env.keys()))
    if not process_configs:
        raise ValueError('There are no processes to be launched')

    config = {
        'processes':
            dump_process_configs_to_groundcontrol_processes(process_configs),
    }
    if len(env) > 0:
        config['env'] = env
    return config


def generate_groundcontrol_config_file(
        deployment_specs_files: list[pathlib.Path],
        merged_params_exec_path: pathlib.Path,
        merged_params_root_path: pathlib.Path,
        groundcontrol_config_file: pathlib.Path):
    # TODO(mvukov) All whitelist of env vars.

    deployments: list[Deployment] = []
    for idx, deployment_specs_file in enumerate(deployment_specs_files):
        print(deployment_specs_file)
        spec = importlib.util.spec_from_file_location(
            f'deployment_specs_module_{idx}', deployment_specs_file)
        # spec can be None!
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        create_deployment = getattr(module, 'create_deployment')
        deployments.append(create_deployment())

    groundcontrol_config = create_groundcontrol_config(deployments,
                                                       merged_params_exec_path,
                                                       merged_params_root_path)

    with open(groundcontrol_config_file, 'w', encoding='utf-8') as stream:
        toml.dump(groundcontrol_config,
                  stream,
                  encoder=toml.TomlPreserveInlineDictEncoder())
