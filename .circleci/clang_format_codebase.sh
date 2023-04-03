#!/usr/bin/env bash

# Current script runs clang_git_format on multiple files of the MRPT codebase.
# - If no extra arguments are given it runs on the whole MRPT codebase
# - If two git commits are given it runs on all the .h .cpp files included in
# these commits.
# - If multiple filepaths are given it runs on each one of the files
# individually

# Provides a handy wrapper for long clang_git_format commands. Run this from
# the  **root** of the MRPT repository

DIRS_IN="include examples"
DIRS_OUT="doc tests"
LANGS=cpp
FORMAT_CODE_BIN=".circleci/clang_git_format/format_code.py"


# Functions
######################
function commit_exists() {

ret=
if git cat-file -e $1^{commit} 2>/dev/null; then
    ret=0
else
    ret=1
fi
return $ret

}

function show_help() {

printf "Current script runs clang_format on multiple .cpp, .h files of "
printf "the MRPT codebase according to your ~/.clang_format file. See file "
printf "docstring for more.\n"
printf "It should be run from the MRPT root directory.\n"

exit $1
}


function format() {

${FORMAT_CODE_BIN} -f -g . --lang ${LANGS} -o ${DIRS_OUT} -i ${DIRS_IN}
exit $?
}


######################

# Make sure we are running it from the root
if [ ! -f CHANGELOG.md ]
then
	show_help 

	echo "ERROR: Cannot find the file CHANGELOG.md!"
	echo "Change to the root dir and rerun this."
	echo "Exiting..."

	exit 1
fi

# find which files to run on
valid_files=
if [ "$#" -eq 0 ]; then
    format
elif [ "$#" -eq 2 ] &&  $(commit_exists $1) && $(commit_exists $2) ; then
    valid_files=$(git diff --name-only $1..$2)
    valid_files=$(echo ${valid_files} | xargs -d' ' -n 1 | grep -ie ".*\.h$\|.*\.cpp$")
else
    valid_files="${@:2}"
fi


if [ -n valid_files ]; then
    printf "Valid files: ${valid_files}\n"
    $(which clang-format) -i -style=file ${valid_files}
fi

exit 0

