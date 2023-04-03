# Copied from python 2.7 version of subprocess.py
# Exception classes used by this module.


class CalledProcessError(Exception):
    """This exception is raised when a process run by check_call() or
    check_output() returns a non-zero exit status. The exit status will be
    stored in the returncode attribute; check_output() will also store the
    output in the output attribute.
    
    """

    def __init__(self, returncode, cmd, output=None):
        self.returncode = returncode
        self.cmd = cmd
        self.output = output

    def __str__(self):
        return ("Command '%s' returned non-zero exit status %d with output %s"
                % (self.cmd, self.returncode, self.output))


class CustomError(BaseException):
    """Class for implementing custom exceptions"""
    pass


class CommitIDTooShort(CustomError):
    def __init__(self, commit_id, min_len):
        self.commit_id = commit_id
        self.min_len = min_len


    def __str__(self):
        return ("At least %d characters of the commit ID "
                "hash should be provided. Current commit ID: %s"
                % (self.min_len, self.commit_id))

