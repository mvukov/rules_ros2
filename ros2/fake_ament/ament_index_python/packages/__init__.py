""" Fakes necessary interfaces of the original library.
"""


class PackageNotFoundError(KeyError):
    pass


def get_package_prefix(*_):
    raise NotImplementedError


def get_package_share_directory(*_):
    raise NotImplementedError
