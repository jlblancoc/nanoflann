# clang_git_format - Python wrapper for bulk reformatting of Git repos

## General Information

Current repository offers an automated solution to the reformatting of entire
projects managed by Git. It was inspired by [the series of
articles](https://engineering.mongodb.com/post/succeeding-with-clangformat-part-1-pitfalls-and-planning/)
of how MongoDB used clang-format to reformat their codebase. As a backbone for
this package, the MongoDB
[clang_format.py](https://github.com/mongodb/mongo/blob/master/buildscripts/clang_format.py)
has been used. Most notably package has the following capabilities:

- Make sure a predetermined (`see clang_git_format/config.py` ) version of
    clang-format script is to be used. If one is not installed, the
    `ClangFormat` object is responsible of downloading and setting one up.
- Run clang-format using the `~/.clang-format` configuration file. A sample
    file is provided along with the python code. Symlink it to your home
    directory if you want to use it.

- Format a code Git repo. User can optionally provide the programming
    language(s) that the repository holds so that clang-format runs on the files
    of that language only. By default C++ is assumed.  Run
    `clang_git_format --help for a list of provided languages`.

- In case of unmerged code in a user (stray) branch users can also use the
    `clang_git_format --reformat_branch <start_commit> <end_commit>`to sync his
    code with the formatting changes of the master branch. For more information
    on how this is done, read [this
    article](https://engineering.mongodb.com/post/succeeding-with-clangformat-part-3-persisting-the-change).

- Validate that a specific set of files complies to the clang-format rules that
    is set. This can be handy in cases when one wants to integrate clang-format
    into a continuous integration (CI) system to verify that all incoming
    pull-requests / commits comply to the repo's coding style.

This package differs to the initial MongoDB script in the following points:

- It's refactored into a sane python package and breaks definition of classes,
    utility methods into separate python files for readability.

- Uses the powerful
    [argparse](https://docs.python.org/dev/library/argparse.html) module instead
    of the deprecated optparse.

- It's more generic (doesn't depend on the MongoDB repo configuration), can be
    used with an arbitrary Git repository and is not tied to the repo language
    (C++, Javascript).


## Usage Instructions

A typical usage of the script would be the following

A list of command-line options is the following. Run `--help` yourself for an
up-to-date list of options:


% TODO - update this

```

$ ./format_code.py --help

usage: format_code.py [-h] [-c CLANG_FORMAT] -g GIT_REPO [-a LANG [LANG ...]]
                      [-x REGEX] [-i DIRS_IN [DIRS_IN ...]]
                      [-o DIRS_OUT [DIRS_OUT ...]]
                      (-l | -L | -p LINT_PATCHES [LINT_PATCHES ...] | -b REFORMAT_BRANCH REFORMAT_BRANCH | -f | -F)

Apply clang-format to a whole Git repository. Execute this script and provide
it with the path to the git repository to operate in. WARNING: You have to run
it from the root of the repo if you want to apply its actions to all the
files.

optional arguments:
  -h, --help            show this help message and exit
  -c CLANG_FORMAT, --clang_format CLANG_FORMAT
                        Path to the clang-format command.
  -g GIT_REPO, --git_repo GIT_REPO
                        Relative path to the root of the git repo that is to
                        be formatted/linted.
  -a LANG [LANG ...], --lang LANG [LANG ...]
                        Languages used in the repository. This is used to
                        determinethe files which clang format runs for.
                        Default langs: {'const': None, 'help': 'Languages used
                        in the repository. This is used to determinethe files
                        which clang format runs for. Default langs: %s.',
                        'option_strings': ['-a', '--lang'], 'dest': 'lang',
                        'required': False, 'nargs': '+', 'choices': None,
                        'default': ['cpp'], 'prog': 'format_code.py',
                        'container': <argparse._ArgumentGroup object at
                        0x7f0a9d484b90>, 'type': 'str', 'metavar': None}.
  -x REGEX, --regex REGEX
                        Custom regular expression to apply to the files that
                        are to be fed to clang-format.
  -i DIRS_IN [DIRS_IN ...], --dirs_in DIRS_IN [DIRS_IN ...]
                        Sequence of directories. If given clang-format is
                        going to run for source exclusively in these
                        directories.
  -o DIRS_OUT [DIRS_OUT ...], --dirs_out DIRS_OUT [DIRS_OUT ...]
                        Sequence of directories. If given clang-format is
                        going to ignore source files in these directories
  -l, --lint            Check if clang-format reports no diffs (clean state).
                        Execute only on files managed by git
  -L, --lint_all        Check if clang-format reports no diffs (clean state).
                        Checked files may or may not be managed by git
  -p LINT_PATCHES [LINT_PATCHES ...], --lint_patches LINT_PATCHES [LINT_PATCHES ...]
                        Check if clang-format reports no diffs (clean state).
                        Check a list of patches, given sequentially after this
                        flag
  -b REFORMAT_BRANCH REFORMAT_BRANCH, --reformat_branch REFORMAT_BRANCH REFORMAT_BRANCH
                        Reformat a branch given the <start> and <end> commits.
  -f, --format          Run clang-format against files managed by git
  -F, --format_all      Run clang-format against files that may or may not be
                        managed by the current git repository```

```
