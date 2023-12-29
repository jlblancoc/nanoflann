"""File containing utility functions"""

import os
import sys
import subprocess
from .custom_exceptions import CalledProcessError
import tarfile

import logging
logger = logging.getLogger("clang-format")


def get_base_dir():
    """Get the base directory for the Git Repo.

    This script assumes that it is running in buildscripts/, and uses
    that to find the base directory.
    """
    try:
        return subprocess.check_output(
            ['git', 'rev-parse', '--show-toplevel']).rstrip()
    except subprocess.CalledProcessError:
        # We are not in a valid git directory. Use the script path instead.
        return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


# Copied from python 2.7 version of subprocess.py
def _check_output(*popenargs, **kwargs):
    r"""Run command with arguments and return its output as a byte string.

    If the exit code was non-zero it raises a CalledProcessError.  The
    CalledProcessError object will have the return code in the returncode
    attribute and output in the output attribute.

    The arguments are the same as for the Popen constructor.  Example:

    >>> check_output(["ls", "-l", "/dev/null"])
    'crw-rw-rw- 1 root root 1, 3 Oct 18  2007 /dev/null\n'

    The stdout argument is not allowed as it is used internally.
    To capture standard error in the result, use stderr=STDOUT.

    >>> _check_output(["/bin/sh", "-c",
    ...               "ls -l non_existent_file ; exit 0"],
    ...              stderr=STDOUT)
    'ls: non_existent_file: No such file or directory\n'
    """
    if 'stdout' in kwargs:
        raise ValueError('stdout argument not allowed, it will be overridden.')
    process = subprocess.Popen(stdout=subprocess.PIPE, *popenargs, **kwargs)
    output, unused_err = process.communicate()
    retcode = process.poll()
    if retcode:
        cmd = kwargs.get("args")
        if cmd is None:
            cmd = popenargs[0]
        raise CalledProcessError(retcode, cmd, output)
    return output


def callo(args):
    """Call a program, and capture its output
    """
    return _check_output(args)


def extract_clang_format(tar_path):
    """ Extract just the clang-format binary.
    On OSX, we shell out to tar because tarfile doesn't support xz compression
    """
    if sys.platform == 'darwin':
        subprocess.call(['tar', '-xzf', tar_path, '*clang-format*'])
    # Otherwise we use tarfile because some versions of tar don't support
    # wildcards without a special flag
    else:
        tarfp = tarfile.open(tar_path)
        for name in tarfp.getnames():
            if name.endswith('clang-format'):
                tarfp.extract(name)
        tarfp.close()
