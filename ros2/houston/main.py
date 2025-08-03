import argparse
import pathlib

from ros2.houston import houston

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
    args = parser.parse_args()

    houston.generate_groundcontrol_config_file(args.deployment_specs,
                                               args.merged_params_exec_path,
                                               args.merged_params_root_path,
                                               args.groundcontrol_config)
