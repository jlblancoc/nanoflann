#!/bin/bash
set -e # exit with nonzero exit code if anything fails

SRC_DIR=`pwd`
BUILD_DIR=build

CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

function build ()
{
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake $SRC_DIR
  make -j2
}

function test ()
{
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake $SRC_DIR
  make test
}

function build_docs ()
{
# Do not build docs twice, once for each compiler!!
if [ "$CC" == "clang" ]; then
  return
fi

# clear and re-create the out directory
OUT_DIR=$SRC_DIR/doc/html

rm -rf $OUT_DIR || exit 0;
mkdir -p $OUT_DIR;

doxygen

cd $OUT_DIR
git init

# inside this git repo we'll pretend to be a new user
git config user.name "Travis CI"
git config user.email "joseluisblancoc@gmail.com"

# The first and only commit to this new Git repo contains all the
# files present with the commit message "Deploy to GitHub Pages".
git add .
git commit -m "Deploy to GitHub Pages"

# Force push from the current repo's master branch to the remote
# repo's gh-pages branch. (All previous history on the gh-pages branch
# will be lost, since we are overwriting it.) We redirect any output to
# /dev/null to hide any sensitive credential data that might otherwise be exposed.
git push --force --quiet "https://${GH_TOKEN}@${GH_REF}" master:gh-pages > /dev/null 2>&1

}


case $TASK in
  build ) build;;
  test ) test;;
  docs ) build_docs;;
esac
