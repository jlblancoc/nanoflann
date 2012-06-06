#!/bin/bash
# Copies sources from SVN tree and prepare a release package.
# JLBC, 2008-2011

#set -o verbose # echo on
set +o verbose # echo off

APPEND_SVN_NUM=0
IS_FOR_UBUNTU=0
APPEND_LINUX_DISTRO=""
while getopts "sud:" OPTION
do
     case $OPTION in
         s)
             APPEND_SVN_NUM=1
             ;;
         u)
             IS_FOR_UBUNTU=1
             ;;
         d)
             APPEND_LINUX_DISTRO=$OPTARG
             ;;
         ?)
             echo "Unknown command line argument!"
             exit 1
             ;;
     esac
done

# Checks
# --------------------------------
if [ -f version ];
then
	NANOFLANN_VERSION_STR=`cat version`
	NANOFLANN_VERSION_MAJOR=${NANOFLANN_VERSION_STR:0:1}
	NANOFLANN_VERSION_MINOR=${NANOFLANN_VERSION_STR:2:1}
	NANOFLANN_VERSION_PATCH=${NANOFLANN_VERSION_STR:4:1}
	NANOFLANN_VER_MM="${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}"
	NANOFLANN_VER_MMP="${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}"
	echo "nanoflann version: ${NANOFLANN_VER_MMP}"
else
	echo "ERROR: Run this script from the nanoflann root directory."
	exit 1
fi

NANOFLANNSRC=`pwd`
NANOFLANN_DEB_DIR="$HOME/nanoflann_release"
NANOFLANN_EXTERN_DEBIAN_DIR="$NANOFLANNSRC/packaging/debian/"

if [ -f ${NANOFLANN_EXTERN_DEBIAN_DIR}/control ];
then
	echo "Using debian dir: ${NANOFLANN_EXTERN_DEBIAN_DIR}"
else
	echo "ERROR: Cannot find ${NANOFLANN_EXTERN_DEBIAN_DIR}"
	exit 1
fi

# Prepare a directory for building the debian package:
# 
rm -fR $NANOFLANN_DEB_DIR
mkdir $NANOFLANN_DEB_DIR

# Are we in svn?
NANOFLANN_SVN_VERSION=`svnversion -n`

if [ $NANOFLANN_SVN_VERSION = "exported" ];
then
	echo "Copying sources to $NANOFLANN_DEB_DIR/nanoflann-${NANOFLANN_VERSION_STR}"
	cp -R . $NANOFLANN_DEB_DIR/nanoflann-${NANOFLANN_VERSION_STR}
else
	# Strip the last "M", if any:
	if [ ${NANOFLANN_SVN_VERSION:(-1)} = "M" ];
	then
		NANOFLANN_SVN_VERSION=${NANOFLANN_SVN_VERSION:0:${#NANOFLANN_SVN_VERSION}-1}
	fi

	if [ $APPEND_SVN_NUM == "1" ];
	then
		NANOFLANN_VERSION_STR="${NANOFLANN_VERSION_STR}svn${NANOFLANN_SVN_VERSION}${APPEND_LINUX_DISTRO}"
	else
		NANOFLANN_VERSION_STR="${NANOFLANN_VERSION_STR}${APPEND_LINUX_DISTRO}"
	fi

	echo "Exporting to $NANOFLANN_DEB_DIR/nanoflann-${NANOFLANN_VERSION_STR}"
	svn export . $NANOFLANN_DEB_DIR/nanoflann-${NANOFLANN_VERSION_STR}
fi

cd $NANOFLANN_DEB_DIR/nanoflann-${NANOFLANN_VERSION_STR}

# Orig tarball:
cd ..
echo "Creating orig tarball: nanoflann-${NANOFLANN_VERSION_STR}.tar.gz"
tar czf nanoflann-${NANOFLANN_VERSION_STR}.tar.gz nanoflann-${NANOFLANN_VERSION_STR}

# Create zip with Windows line feeds:
find nanoflann-${NANOFLANN_VERSION_STR} -name '*.hpp' | xargs -I FIL todos FIL
find nanoflann-${NANOFLANN_VERSION_STR} -name '*.txt' | xargs -I FIL todos FIL

zip -r nanoflann-${NANOFLANN_VERSION_STR}.zip nanoflann-${NANOFLANN_VERSION_STR}

exit 0

