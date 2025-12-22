import yaml
import os
import sys

candidates = [
    'repositories/ros2_repo_mappings.yaml',
    'external/rules_ros2+/repositories/ros2_repo_mappings.yaml',
    '../rules_ros2+/repositories/ros2_repo_mappings.yaml',
]

MAPPINGS = None
for c in candidates:
    if os.path.exists(c):
        MAPPINGS = c
        break

if not MAPPINGS:
    # Fallback: look relative to script
    # script is likely in <...>/repositories/private/ros2_repo_mappings_tests.py
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(script_dir, '../ros2_repo_mappings.yaml')
    if os.path.exists(candidate):
        MAPPINGS = candidate

if not MAPPINGS:
    # Debug info
    print(f"CWD: {os.getcwd()}")
    os.system("ls -R ..")
    raise FileNotFoundError("Could not find ros2_repo_mappings.yaml")

with open(MAPPINGS, encoding='utf-8') as stream:
    repos = yaml.load(stream, Loader=yaml.Loader)['repositories']

repo_names = list(repos.keys())
sorted_repo_names = sorted(repo_names)

assert sorted_repo_names == repo_names, 'Please sort the repo names!'