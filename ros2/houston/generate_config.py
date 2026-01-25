import argparse
import collections
import dataclasses
import importlib.util
import os
import pathlib

import deepmerge
import toml
import yaml

from ros2.houston import entity


def flatten(src: entity.Deployment) -> entity.Deployment:
    dst_entities: list[entity.Entity] = []
    src_entities: collections.deque[entity.Entity] = collections.deque(
        src.entities)
    while src_entities:
        src_entity = src_entities.pop()
        match src_entity:
            case entity.Deployment(entities):
                for current_entity in entities:
                    src_entities.appendleft(current_entity)
            case entity.EnvironmentVariable() | entity.RosNode(
            ) | entity.Process() | entity.ParametersFile():
                dst_entities.append(src_entity)
            case _:
                raise TypeError(
                    f'Got unsupported entity of type {type(entity)}: {entity}')
    return entity.Deployment(dst_entities)


def validate(deployment: entity.Deployment,
             executable_paths: list[pathlib.Path]):
    for current_entity in deployment.entities:
        match current_entity:
            case entity.EnvironmentVariable():
                if not current_entity.name:
                    raise ValueError(
                        'Environment variable name must not be empty')

            case entity.Process() | entity.RosNode():
                if not current_entity.name:
                    raise ValueError('RosNode name must not be empty')
                if pathlib.Path(
                        current_entity.executable) not in executable_paths:
                    raise ValueError(
                        f'Process {current_entity.name} executable '
                        f'{current_entity.executable} is '
                        f'not in the list of known nodes {executable_paths}')

            case entity.ParametersFile():
                if not os.path.exists(current_entity.path):
                    raise ValueError(
                        f'Parameter file {current_entity.path} does not exist')

            case _:
                pass


def collect_env(deployment: entity.Deployment) -> dict[str, str]:
    env: dict[str, str] = {}
    for current_entity in deployment.entities:
        match current_entity:
            case entity.EnvironmentVariable(name=name, value=value):
                env[name] = value
            case _:
                pass
    return env


def collect_parameters(deployment: entity.Deployment, dst: pathlib.Path):
    params_files: list[pathlib.Path] = []
    for current_entity in deployment.entities:
        match current_entity:
            case entity.RosNode():
                if current_entity.parameters_file is None:
                    continue
                params_files.append(pathlib.Path(
                    current_entity.parameters_file))
            case entity.ParametersFile():
                params_files.append(pathlib.Path(current_entity.path))
            case _:
                pass

    if not params_files:
        return

    params_files = params_files[::-1]
    with open(params_files[0], encoding='utf-8') as stream:
        merged_params = yaml.load(stream, yaml.SafeLoader)

    merger = deepmerge.Merger(
        [
            # Defines type-wise strategies.
            (list, 'append_unique'),
            (dict, 'merge'),
            (set, 'union'),
        ],
        ['override'],  # Fallback strategies, applied to all other types.
        ['override'],  # Strategies in the case where the types conflict.
    )

    if len(params_files) > 1:
        for params_file in params_files[1:]:
            with open(params_file, encoding='utf-8') as stream:
                merged_params = merger.merge(merged_params,
                                             yaml.load(stream, yaml.FullLoader))

    with open(dst, 'w', encoding='utf-8') as stream:
        yaml.dump(merged_params, stream)


@dataclasses.dataclass(frozen=True, kw_only=True)
class CommandConfig:
    only_env: list[str] | None
    program: str
    args: list[str]


@dataclasses.dataclass(frozen=True, kw_only=True)
class ProcessConfig:
    name: str
    run: CommandConfig
    stop: str


def create_process_process_config(process: entity.Process,
                                  only_env: list[str] | None) -> ProcessConfig:
    args = process.arguments
    args = args if args is not None else []
    return ProcessConfig(name=process.name,
                         run=CommandConfig(only_env=only_env,
                                           program=process.executable,
                                           args=args),
                         stop='SIGINT')


def create_ros_node_process_config(node: entity.RosNode,
                                   merged_params_file: pathlib.Path,
                                   only_env: list[str] | None) -> ProcessConfig:
    args = ['--ros-args', '-r', f'__node:={node.name}']
    args.extend(['--params-file', merged_params_file])

    if node.remappings is not None:
        for src, dst in node.remappings:
            args.extend(['-r', f'{src}:={dst}'])

    if node.ros_arguments is not None and node.ros_arguments:
        args.extend(node.ros_arguments)
    if node.arguments is not None and node.arguments:
        args.append('--')
        args.extend(node.arguments)

    return ProcessConfig(name=node.name,
                         run=CommandConfig(only_env=only_env,
                                           program=node.executable,
                                           args=args),
                         stop='SIGINT')


def collect_process_configs(
        deployment: entity.Deployment,
        merged_params_file: pathlib.Path,
        only_env: list[str] | None = None) -> list[ProcessConfig]:
    process_configs: list[ProcessConfig] = []
    for current_entity in deployment.entities:
        match current_entity:
            case entity.Process():
                process_configs.append(
                    create_process_process_config(current_entity, only_env))
            case entity.RosNode():
                process_configs.append(
                    create_ros_node_process_config(current_entity,
                                                   merged_params_file,
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
    deployments: list[entity.Deployment],
    executable_paths: list[pathlib.Path],
    merged_params_exec_path: pathlib.Path,
    merged_params_root_path: pathlib.Path,
) -> dict:
    deployment = entity.Deployment(deployments)
    flattened_deployment = flatten(deployment)
    validate(flattened_deployment, executable_paths)

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
        groundcontrol_config_file: pathlib.Path,
        executable_paths: pathlib.Path):
    # TODO(mvukov) All whitelist of env vars.

    deployments: list[entity.Deployment] = []
    for idx, deployment_specs_file in enumerate(deployment_specs_files):
        spec = importlib.util.spec_from_file_location(
            f'deployment_specs_module_{idx}', deployment_specs_file)
        # spec can be None!
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
            create_deployment = getattr(module, 'create_deployment')
        except (AttributeError, SyntaxError) as ex:
            raise RuntimeError(
                f'Failed to process {deployment_specs_file}: {ex}')
        deployments.append(create_deployment())

    groundcontrol_config = create_groundcontrol_config(deployments,
                                                       executable_paths,
                                                       merged_params_exec_path,
                                                       merged_params_root_path)

    with open(groundcontrol_config_file, 'w', encoding='utf-8') as stream:
        toml.dump(groundcontrol_config,
                  stream,
                  encoder=toml.TomlPreserveInlineDictEncoder())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--deployment_specs',
                        nargs='+',
                        type=pathlib.Path,
                        required=True)
    parser.add_argument('--merged_params_exec_path',
                        type=pathlib.Path,
                        required=True)
    parser.add_argument('--merged_params_root_path',
                        type=pathlib.Path,
                        required=True)
    parser.add_argument('--groundcontrol_config',
                        type=pathlib.Path,
                        required=True)
    parser.add_argument('--executable_paths',
                        nargs='+',
                        type=pathlib.Path,
                        required=True)
    args = parser.parse_args()

    generate_groundcontrol_config_file(args.deployment_specs,
                                       args.merged_params_exec_path,
                                       args.merged_params_root_path,
                                       args.groundcontrol_config,
                                       args.executable_paths)
