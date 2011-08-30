#!/bin/bash
# Copies sources from SVN tree and prepare a Debian package.
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
NANOFLANN_DEB_DIR="$HOME/nanoflann_debian"
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

# Deletions:
rm -fR bin 2>/dev/null
rm -fR packaging

# Orig tarball:
cd ..
echo "Creating orig tarball: nanoflann_${NANOFLANN_VERSION_STR}.orig.tar.gz"
tar czf nanoflann_${NANOFLANN_VERSION_STR}.orig.tar.gz nanoflann-${NANOFLANN_VERSION_STR}

# Copy debian directory:
mkdir nanoflann-${NANOFLANN_VERSION_STR}/debian
cp -r ${NANOFLANN_EXTERN_DEBIAN_DIR}/* nanoflann-${NANOFLANN_VERSION_STR}/debian
cp ${NANOFLANN_EXTERN_DEBIAN_DIR}/copyright nanoflann-${NANOFLANN_VERSION_STR}/copyright

# Strip my custom files...
rm nanoflann-${NANOFLANN_VERSION_STR}/debian/*.new 
# debian/source file issues for old Ubuntu distros:
if [ $IS_FOR_UBUNTU == "1" ];
then
	rm -fr nanoflann-${NANOFLANN_VERSION_STR}/debian/source
fi


# Prepare install files:
cd nanoflann-${NANOFLANN_VERSION_STR}

# Figure out the next Debian version number:
echo "Detecting next Debian version number..."

CHANGELOG_UPSTREAM_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*.*svn.*\)-.*/\1/p' )
CHANGELOG_LAST_DEBIAN_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*\).*-\([0-9]*\).*/\2/p' )

echo " -> PREVIOUS UPSTREAM: $CHANGELOG_UPSTREAM_VER -> New: ${NANOFLANN_VERSION_STR}"
echo " -> PREVIOUS DEBIAN VERSION: $CHANGELOG_LAST_DEBIAN_VER"

# If we have the same upstream versions, increase the Debian version, otherwise create a new entry:
if [ "$CHANGELOG_UPSTREAM_VER" = "$NANOFLANN_VERSION_STR" ];
then
	NEW_DEBIAN_VER=$[$CHANGELOG_LAST_DEBIAN_VER + 1]
	echo "Changing to a new Debian version: ${NANOFLANN_VERSION_STR}-${NEW_DEBIAN_VER}"
	DEBCHANGE_CMD="--newversion ${NANOFLANN_VERSION_STR}-${NEW_DEBIAN_VER}"
else
	DEBCHANGE_CMD="--newversion ${NANOFLANN_VERSION_STR}-1"
fi

echo "Adding a new entry to debian/changelog..."
echo DEBEMAIL="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD --distribution unstable --force-distribution New version of upstream sources.

DEBEMAIL="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD -b --distribution unstable --force-distribution New version of upstream sources.

echo "Copying back the new changelog to a temporary file in: ${NANOFLANN_EXTERN_DEBIAN_DIR}changelog.new"
cp debian/changelog ${NANOFLANN_EXTERN_DEBIAN_DIR}changelog.new

set +o verbose # echo off

echo "Now, you can build the source Deb package with 'debuild -S -sa'"

cd ..
ls -lh


exit 0

