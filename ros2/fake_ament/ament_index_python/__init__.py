""" Fakes necessary interfaces of the original library.
"""
from .packages import *


def get_resource(*_):
    raise NotImplementedError


def has_resource(*_):
    raise NotImplementedError


def get_resources(*_):
    raise NotImplementedError


def get_packages_with_prefixes(*_):
    raise NotImplementedError
