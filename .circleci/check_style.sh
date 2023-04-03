#!/usr/bin/env bash

# Given a set of commits current script checks the style based on clang_format
# on the list of changed files that exist in the commits.
#
# Script should be called from a travis.yml file.

# Functions
######################

# This is run by travis automatically from the root of the project

set +e
set +x

DIRS_IN="include examples"
DIRS_OUT="doc tests"
LANGS=cpp
FORMAT_CODE_BIN=".circleci/clang_git_format/format_code.py"

function lint() {

# Get list of changed files for lint to run on. Grep only .h, .cpp files and
# mind the DIRS_IN, DIRS_OUT vars

# Following command fails if you "git commit --amend" - Works correctly on
# standard commits
echo "commit_range: $TRAVIS_COMMIT_RANGE"
changed_files=$(git diff --name-only $TRAVIS_COMMIT_RANGE)
printf "Summary of changed files:\n\n${changed_files}\n\n"

# include
regexp_in=
for i in $DIRS_IN; do
  if [ -n "${regexp_in}" ]; then
    regexp_in="${regexp_in}\|${i}"
  else
    regexp_in="${i}"
  fi
done

# exclude
regexp_out=
for i in $DIRS_OUT; do
  if [ -n "${regexp_out}" ]; then
    regexp_out="${regexp_out}\|${i}"
  else
    regexp_out="${i}"
  fi
done

echo "regexp_in: ${regexp_in}"
echo "regexp_out: ${regexp_out}"

valid_files=$(echo ${changed_files} \
  | xargs -d' ' -n 1 \
  | grep -i -e "${regexp_in}" \
  | grep -v "${regexp_out}" \
  | grep -ie ".*\.h$\|.*\.cpp")


printf "Valid files for lint:\n\t${valid_files}\n"
if [ -n "${valid_files}" ]; then
  ${FORMAT_CODE_BIN} -g .  --lint_files ${valid_files}
else
  true
fi
exit $?

}

function lint_all() {

${FORMAT_CODE_BIN} -g . --lang ${LANGS} \
  -o ${DIRS_OUT} -i ${DIRS_IN} \
  -l

exit $?
}

lint_all
