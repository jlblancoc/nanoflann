#!/usr/bin/env python3
"""
A script that provides:
1. Ability to grab binaries where possible from LLVM.
2. Ability to download binaries from MongoDB cache for clang-format.
3. Validates clang-format is the right version.
4. Supports checking which files are to be checked.
5. Supports validating and updating a set of files to the right coding style.
"""
from __future__ import print_function
# absolute_import

import queue
import re
import os
import sys
import threading
import time
import argparse
from multiprocessing import cpu_count

from clang_git_format import ClangFormat
from clang_git_format import Repo
from clang_git_format import CommitIDTooShort

# setup the logging
import logging
from colorlog import ColoredFormatter

LOG_LEVEL = logging.DEBUG
LOGFORMAT = ("%(log_color)s%(levelname)-5s%(reset)s "
             "| %(log_color)s%(message)s%(reset)s")

logging.root.setLevel(LOG_LEVEL)
formatter = ColoredFormatter(LOGFORMAT)
stream = logging.StreamHandler()
stream.setLevel(LOG_LEVEL)
stream.setFormatter(formatter)

logger = logging.getLogger("clang-format")
logger.setLevel(LOG_LEVEL)
logger.addHandler(stream)


def parallel_process(items, func):
    """Run a set of work items to completion
    """
    try:
        cpus = cpu_count()

    except NotImplementedError:
        cpus = 1

    task_queue = queue.Queue()

    # Use a list so that worker function will capture this variable
    pp_event = threading.Event()
    pp_result = [True]
    pp_lock = threading.Lock()

    def worker():
        """Worker thread to process work items in parallel. """
        while not pp_event.is_set():

            try:
                item = task_queue.get_nowait()
            except queue.Empty:
                # if the queue is empty, exit the worker thread
                pp_event.set()
                return

            try:
                logger.debug("Operating on file: %s" % item)
                ret = func(item)
            finally:
                # Tell the queue we finished with the item
                task_queue.task_done()

            # Return early if we fail, and signal we are done
            if not ret:
                logger.error("clang-format exited with an error!")
                with pp_lock:
                    pp_result[0] = False

                pp_event.set()
                return

    # Enqueue all the work we want to process
    for item in items:
        task_queue.put(item)

    # Process all the work
    threads = []
    for cpu in range(cpus):
        thread = threading.Thread(target=worker)

        thread.daemon = True
        thread.start()
        threads.append(thread)

    # Wait for the threads to finish
    # Loop with a timeout so that we can process Ctrl-C interrupts
    # Note: On Python 2.6 wait always returns None so we check is_set also,
    #  This works because we only set the event once, and never reset it
    while not pp_event.wait(1) and not pp_event.is_set():
        time.sleep(1)

    for thread in threads:
        thread.join()

    return pp_result[0]


class ClangRepoFormatter(object):
    """Class that handles the overall execution."""

    def __init__(self):
        super(ClangRepoFormatter, self).__init__()

    def get_repo(self):
        """Return Repo object for the git repository to be formatted."""
        return self.git_repo

    def run(self):
        """Main entry point """
        logger.info("Initializing script...")

        parser = argparse.ArgumentParser()
        parser.description = ("Apply clang-format to a whole Git repository. "
                              "Execute this script and provide it with the "
                              "path to the git repository "
                              "to operate in. "
                              "WARNING: You have to run it from the root of "
                              "the repo if you want to apply its actions "
                              "to all the files.")

        parser.add_argument(
            "-c",
            "--clang_format",
            default=None,
            type=str,
            help="Path to the clang-format command.")

        parser.add_argument(
            '-g',
            '--git_repo',
            type=str,
            required=True,
            help=("Relative path to the root of the git repo that "
                  "is to be formatted/linted."))

        default_langs = ["cpp"]
        parser.add_argument(
            '-a',
            '--lang',
            type=str,
            nargs='+',
            default=default_langs,
            help=("Languages used in the repository. This is used to determine"
                  "the files which clang format runs for. Default langs: %s."
                  .format(default_langs)))
        parser.add_argument(
            '-x',
            '--regex',
            type=str,
            default="",
            required=False,
            help=("Custom regular expression to apply to the files that are "
                  "to be fed to clang-format."))
        parser.add_argument(
            '-i',
            '--dirs_in',
            type=str,
            nargs="+",
            default=[],
            required=False,
            help=("Sequence of directories. If given clang-format is going to "
                  "run for source exclusively in these directories."))
        parser.add_argument(
            '-o',
            '--dirs_out',
            type=str,
            default=[],
            nargs="+",
            required=False,
            help=("Sequence of directories. If given clang-format is going to "
                  "ignore source files in these directories"))



        # Mutually exclusive, internet-related arguments
        commands_group = parser.add_mutually_exclusive_group(required=True)

        commands_group.add_argument(
            "-d",
            "--lint_files",
            default=[],
            nargs='+',
            help=("Check if clang-format reports no diffs (clean state). "
                  "Execute only on files given sequentially after this flag."))
        commands_group.add_argument(
            "-l",
            "--lint",
            action="store_true",
            help=("Check if clang-format reports no diffs (clean state). "
                  "Execute only on files managed by git"))
        commands_group.add_argument(
            "-L",
            "--lint_all",
            action="store_true",
            help=("Check if clang-format reports no diffs (clean state). "
                  "Checked files may or may not be managed by git"))
        commands_group.add_argument(
            "-p",
            "--lint_patches",
            default=[],
            nargs='+',
            help=("Check if clang-format reports no diffs (clean state). "
                  "Check a list of patches, given sequentially after this "
                  "flag"))
        commands_group.add_argument(
            "-b",
            "--reformat_branch",
            default=[],
            nargs=3,
            help=("Reformat CURRENT branch given the <start-1> and"
                  " <end-2> commits."
                  " Finally rebase it on top of <branch-3> branch"))
        commands_group.add_argument(
            "-f",
            "--format",
            action="store_true",
            help=("Run clang-format against files managed by git"))
        commands_group.add_argument(
            "-F",
            "--format_all",
            action="store_true",
            help=("Run clang-format against files that may or may not "
                  "be managed by the current git repository"))

        parser_args = vars(parser.parse_args())

        # clang-format executabel wrapper
        self.clang_format = ClangFormat(parser_args["clang_format"],
                                        self._get_build_dir())

        # git repo
        self.git_repo = Repo(
            parser_args["git_repo"], custom_regex=parser_args["regex"],
            dirs_in=parser_args["dirs_in"],
            dirs_out=parser_args["dirs_out"],
        )
        self.git_repo.langs_used = parser_args['lang']

        # determine your action based on the user input
        if parser_args["lint"]:
            self.lint()
        elif parser_args["lint_all"]:
            self.lint_all()
        elif parser_args["lint_files"]:
            file_list = parser_args["lint_files"]
            self._lint_files(file_list)
        elif len(parser_args["lint_patches"]):
            file_list = parser_args["lint_patches"]
            self.lint_patches(file_list)
        elif len(parser_args["reformat_branch"]):
            start = parser_args["reformat_branch"][0]
            end = parser_args["reformat_branch"][1]
            main_branch = parser_args["reformat_branch"][2]
            self.reformat_branch(start, end, main_branch)
        elif parser_args["format"]:
            self.format_func()
        elif parser_args["format_all"]:
            self.format_func_all()
        else:
            logger.fatal("Unexpected error in the parsing of command line "
                         "arguments! Exiting.")

    def get_list_from_lines(self, lines):
        """"Convert a string containing a series of lines into a list of strings
        """
        return [line.rstrip() for line in lines.splitlines()]

    def get_files_to_check_working_tree(self):
        """Get a list of files to check from the working tree.
        This will pick up files not managed by git.
        """
        repo = self.get_repo()
        valid_files = repo.get_working_tree_candidates()

        return valid_files

    def get_files_to_check(self):
        """Get a list of files that need to be checked
        based on which files are managed by git.
        """
        repo = self.get_repo()
        valid_files = repo.get_candidates(None)
        return valid_files

    def get_files_to_check_from_patch(self, patches):
        """Take a list of patch files generated by git diff, and scan them for a
        list of files.

        :param patches list of patches to check
        """
        candidates = []

        # Get a list of candidate_files
        check = re.compile(
            r"^diff --git a\/([a-z\/\.\-_0-9]+) b\/[a-z\/\.\-_0-9]+")

        lines = []
        for patch in patches:
            with open(patch, "rb") as infile:
                lines += infile.readlines()

        candidates = [
            check.match(line).group(1) for line in lines if check.match(line)
        ]

        repo = self.get_repo()
        valid_files = repo.get_candidates(candidates)
        return valid_files

    def _get_build_dir(self):
        """Get the location of a build directory in case we need to download
        clang-format

        """
        return os.path.join(os.path.curdir, "build")

    def _lint_files(self, files):
        """Lint a list of files with clang-format
        """
        lint_clean = parallel_process([os.path.abspath(f)
                                       for f in files], self.clang_format.lint)

        if not lint_clean:
            logger.error("Code Style does not match coding style")
            sys.exit(1)

    def lint_patches(self, infile):
        """Lint patch command entry point
        """
        files = self.get_files_to_check_from_patch(infile)

        # Patch may have files that we do not want to check which is fine
        if files:
            self._lint_files(files)

    def lint(self):
        """Lint files command entry point
        """
        files = self.get_files_to_check()
        self._lint_files(files)

        return True

    def lint_all(self):
        """Lint files command entry point based on working tree (some files may
        not be managed by git
        """
        files = self.get_files_to_check_working_tree()
        self._lint_files(files)

        return True

    def _format_files(self, files):
        """Format a list of files with clang-format
        """
        format_clean = parallel_process([os.path.abspath(f) for f in files],
                                        self.clang_format.format_func)

        if not format_clean:
            logger.error("failed to format files")
            sys.exit(1)

    def format_func(self):
        """Format files command entry point
        """
        files = self.get_files_to_check()

        self._format_files(files)

    def format_func_all(self):
        """Format files command entry point
        """
        files = self.get_files_to_check_working_tree()
        self._format_files(files)

    def reformat_branch(self, commit_prior_reformat, commit_after_reformat,
                        main_branch):
        """Reformat a branch made before a clang-format run

        :param str commit_prior_reformat The base commit ID connecting the main
        branch with the branch to be merged
        :param str commit_prior_reformat The last commit ID to be reformatted
        and merged into the main branch
        """

        min_commit_id_len = 5

        # verify given commits lenght
        if (len(commit_prior_reformat) < min_commit_id_len):
            raise CommitIDTooShort(commit_prior_reformat,
                                   min_commit_id_len)
        if (len(commit_after_reformat) < min_commit_id_len):
            raise CommitIDTooShort(commit_after_reformat,
                                   min_commit_id_len)

        old_pwd = os.getcwd()
        os.chdir(self.get_repo().path)

        repo = self.get_repo()

        ###################################################################3
        # Validate that the current state is OK

        # Validate that user passes valid commits
        if not repo.is_commit(commit_prior_reformat):
            raise ValueError(
                "Commit Prior to Reformat '%s' is not "
                "a valid commit in this repo" % commit_prior_reformat)
        if not repo.is_commit(commit_after_reformat):
            raise ValueError(
                "Commit After Reformat '%s' is not a valid commit in this repo"
                % commit_after_reformat)
#         if not repo.is_ancestor(commit_prior_reformat,
                                # commit_after_reformat):
            # raise ValueError(
                # ("Commit Prior to Reformat '%s' is not a valid ancestor "
                 # "of Commit After" + " Reformat '%s' in this repo") %
                # (commit_prior_reformat, commit_after_reformat))

        # Validate the user is on a local branch that has the right merge base
        if repo.is_detached():
            raise ValueError("You must not run this script in a detached "
                             "HEAD state")

        # Validate the user has no pending changes
        if repo.is_working_tree_dirty():
            raise ValueError("Your working tree has pending changes. "
                             "You must have a clean working tree before "
                             "proceeding.")

        # validate that the parent of the stranded commit is the merge base
        merge_base = repo.get_merge_base(commit_prior_reformat)
        if (not merge_base[0:min_commit_id_len] ==
                commit_prior_reformat[0:min_commit_id_len]):
            raise ValueError("Please **rebase** your work to '%s' and resolve all "
                             "conflicts before running this script" %
                             (commit_prior_reformat))

        merge_base = repo.get_merge_base(main_branch)

        # if (not merge_base[0:min_commit_id_len] ==
                # commit_prior_reformat[0:min_commit_id_len]):
            # raise ValueError(("The base commit of the merge (%s) and the "
                              # "start_commit (%s) issued don't match."
                              # % (merge_base, commit_prior_reformat)))

        # End of validations
        ###################################################################3

        # Everything looks good so lets start going through all the commits
        branch_name = repo.get_branch_name()
        new_branch = "%s-reformatted" % branch_name

        # Make sure that this is a new branch
        if repo.does_branch_exist(new_branch):
            raise ValueError("The branch '%s' already exists. "
                             "Please delete the "
                             "branch '%s', or rename the current branch." %
                             (new_branch, new_branch))

        commits = self.get_list_from_lines(
            repo.log([
                "--reverse", "--pretty=format:%H",
                "%s..HEAD" % commit_prior_reformat
            ]))

        previous_commit_base = commit_after_reformat
        files_to_check = self.get_files_to_check()

        # Go through all the commits the user made on the local branch and
        # migrate to a new branch that is based on post_reformat commits instead
        for commit_hash in commits:
            repo.checkout(["--quiet", commit_hash])

            deleted_files = []

            # Format each of the files by checking out just a single commit from
            # the user's branch
            commit_files = self.get_list_from_lines(
                repo.diff(["HEAD~", "--name-only"]))

            for commit_file in commit_files:

                # Format each file needed if it was not deleted
                if not os.path.exists(commit_file):
                    logger.warning("Skipping file '%s' since it has been "
                                   "deleted in commit '%s'"
                                   % (commit_file, commit_hash))
                    deleted_files.append(commit_file)
                    continue

                if commit_file in files_to_check:
                    self.clang_format.format_func(commit_file)
                else:
                    logger.info("Skipping file '%s' since it is not a "
                                "file clang_format should format" % commit_file)

            # Check if anything needed reformatting, and if so amend the commit
            if not repo.is_working_tree_dirty():
                print("Commit %s needed no reformatting" % commit_hash)
            else:
                repo.commit(["--all", "--amend", "--no-edit"])

            # Rebase our new commit on top the post-reformat commit
            previous_commit = repo.rev_parse(["HEAD"])

            # Checkout the new branch with the reformatted commits
            # Note: we will not name as a branch until we are done with all
            # commits on the local branch
            repo.checkout(["--quiet", previous_commit_base])

            # Copy each file from the reformatted commit on top of the post
            # reformat
            diff_files = self.get_list_from_lines(
                repo.diff([
                    "%s~..%s" % (previous_commit, previous_commit),
                    "--name-only"
                ]))

            for diff_file in diff_files:
                # If the file was deleted in the commit we are reformatting, we
                # need to delete it again
                if diff_file in deleted_files:
                    repo.rm([diff_file])
                    continue

                # The file has been added or modified, continue as normal
                file_contents = repo.show(
                    ["%s:%s" % (previous_commit, diff_file)])

                root_dir = os.path.dirname(diff_file)
                if root_dir and not os.path.exists(root_dir):
                    os.makedirs(root_dir)

                with open(diff_file, "w+") as new_file:
                    new_file.write(file_contents)

                repo.add([diff_file])

            # Create a new commit onto clang-formatted branch
            repo.commit(["--reuse-message=%s" % previous_commit])

            previous_commit_base = repo.rev_parse(["HEAD"])


        # Create a new branch to mark the hashes we have been using
        repo.checkout(["-b", new_branch])

        # change back to previous directory
        os.chdir(old_pwd)

        logger.info("reformat_branch is done running.\n"
                    "A copy of your branch has been made named '%s',"
                    " and formatted with clang-format.\n"
                    " The original branch has been left unchanged.\n"
                    " Your new branch should already be rebased on top of %s."
                    % (new_branch))


if __name__ == "__main__":
    crf = ClangRepoFormatter()
    crf.run()
