""" Fakes necessary interfaces of the original library.
"""


class PackageNotFoundError(KeyError):
    pass


def get_package_prefix(*_):
    """ Fix when needed.
    """
    return None


def get_package_share_directory(*_):
    """ Fix when needed.
    """
    return None
