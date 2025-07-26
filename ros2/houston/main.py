import argparse

from ros2.houston import houston

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--deployment_specs_path', required=True)
    parser.add_argument('--merged_params_file', required=True)
    parser.add_argument('--groundcontrol_config_path', required=True)
    args = parser.parse_args()

    houston.generate_groundcontrol_config_file(args.deployment_specs_path,
                                               args.merged_params_file,
                                               args.groundcontrol_config_path)
