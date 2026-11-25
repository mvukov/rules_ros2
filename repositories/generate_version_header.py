# Copyright 2026 Milan Vukov
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generates a version header from a package.xml.

By default, generates <package>/version.h, an equivalent of
ament_generate_version_header() from ament_cmake_gen_version_h:
https://github.com/ament/ament_cmake/blob/rolling/ament_cmake_gen_version_h/cmake/version.h.in

With --template, expands the given template instead, applying the given
substitutions. Substitution values may use the {name}, {version}, {major},
{minor} and {patch} fields, which are read from the package.xml.
"""
import argparse
import re
import xml.etree.ElementTree as ET

_AMENT_TEMPLATE = """\
#ifndef {P}__VERSION_H_
#define {P}__VERSION_H_
#define {P}_VERSION_MAJOR ({major})
#define {P}_VERSION_MINOR ({minor})
#define {P}_VERSION_PATCH ({patch})
#define {P}_VERSION_STR "{version}"
#define {P}_VERSION_GTE(major, minor, patch) ( \\
     (major < {P}_VERSION_MAJOR) ? true \\
     : ((major > {P}_VERSION_MAJOR) ? false \\
     : ((minor < {P}_VERSION_MINOR) ? true \\
     : ((minor > {P}_VERSION_MINOR) ? false \\
     : ((patch < {P}_VERSION_PATCH) ? true \\
     : ((patch > {P}_VERSION_PATCH) ? false \\
     : true))))))
#endif  // {P}__VERSION_H_
"""

# CMake configure_file(@ONLY)-style placeholders, e.g. @FOO_VERSION@.
_PLACEHOLDER_RE = re.compile(r'@[A-Za-z_][A-Za-z0-9_]*@')


def _get_text(root, tag, package_xml):
    element = root.find(tag)
    if element is None or not element.text or not element.text.strip():
        raise ValueError(f'No <{tag}> found in {package_xml}')
    return element.text.strip()


def read_package_info(package_xml):
    """Returns the name and version fields of the given package.xml."""
    root = ET.parse(package_xml).getroot()
    name = _get_text(root, 'name', package_xml)
    version = _get_text(root, 'version', package_xml)
    match = re.fullmatch(r'(\d+)\.(\d+)\.(\d+)', version)
    if not match:
        raise ValueError(f'Invalid version "{version}" in {package_xml}, '
                         'expected <major>.<minor>.<patch>')
    major, minor, patch = match.groups()
    return {
        'name': name,
        'version': version,
        'major': major,
        'minor': minor,
        'patch': patch,
    }


def generate_ament_version_header(package_info):
    return _AMENT_TEMPLATE.format(P=package_info['name'].upper(),
                                  **package_info)


def expand_template(template, substitutions, package_info):
    for key, value in substitutions.items():
        if key not in template:
            raise ValueError(f'Substitution key "{key}" not found in template')
        template = template.replace(key, value.format(**package_info))
    leftovers = sorted(set(_PLACEHOLDER_RE.findall(template)))
    if leftovers:
        raise ValueError('Unexpanded placeholders in template: ' +
                         ', '.join(leftovers))
    return template


def _parse_substitution(arg):
    key, sep, value = arg.partition('=')
    if not sep or not key:
        raise argparse.ArgumentTypeError(
            f'Expected <key>=<value> substitution, got "{arg}"')
    return key, value


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('package_xml', help='Path to the package.xml.')
    parser.add_argument('output', help='Path to the output header.')
    parser.add_argument('--template',
                        help='Path to a template to expand instead of '
                        'generating an ament version.h.')
    parser.add_argument('--substitution',
                        action='append',
                        default=[],
                        type=_parse_substitution,
                        help='A <key>=<value> template substitution. '
                        'Can be given multiple times.')
    args = parser.parse_args()

    package_info = read_package_info(args.package_xml)
    if args.template:
        with open(args.template, encoding='utf-8') as stream:
            template = stream.read()
        header = expand_template(template, dict(args.substitution),
                                 package_info)
    else:
        if args.substitution:
            parser.error('--substitution requires --template')
        header = generate_ament_version_header(package_info)

    with open(args.output, 'w', encoding='utf-8') as stream:
        stream.write(header)


if __name__ == '__main__':
    main()
