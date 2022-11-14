import yaml

MAPPINGS = 'repositories/ros2_repo_mappings.yaml'

with open(MAPPINGS, 'r', encoding='utf-8') as stream:
    repos = yaml.load(stream, Loader=yaml.Loader)['repositories']

repo_names = list(repos.keys())
sorted_repo_names = sorted(repo_names)

assert sorted_repo_names == repo_names, 'Please sort the repo names!'
