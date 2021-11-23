#!/bin/bash
# Creates a set of packages for each different Ubuntu distribution, with the 
# intention of uploading them to: 
#   https://launchpad.net/~joseluisblancoc/+archive/nanoflann
#
# JLBC, 2010-2011

# Checks
# --------------------------------
if [ -f version ];
then
	NANOFLANN_VERSION_STR=`cat version`
	NANOFLANN_VERSION_MAJOR=${NANOFLANN_VERSION_STR:0:1}
	NANOFLANN_VERSION_MINOR=${NANOFLANN_VERSION_STR:2:1}
	NANOFLANN_VERSION_PATCH=${NANOFLANN_VERSION_STR:4:1}
	AUX_SVN=$(svnversion)
	#Remove the trailing "M":
	NANOFLANN_VERSION_SVN=${AUX_SVN%M}

	NANOFLANN_VER_MM="${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}"
	NANOFLANN_VER_MMP="${NANOFLANN_VERSION_MAJOR}.${NANOFLANN_VERSION_MINOR}.${NANOFLANN_VERSION_PATCH}"
	echo "nanoflann version: ${NANOFLANN_VER_MMP} (SVN: ${NANOFLANN_VERSION_SVN})"
else
	echo "ERROR: Run this script from the nanoflann root directory."
	exit 1
fi

NANOFLANN_UBUNTU_OUT_DIR="$HOME/nanoflann_ubuntu"
NANOFLANNSRC=`pwd`
NANOFLANN_DEB_DIR="$HOME/nanoflann_debian"
NANOFLANN_EXTERN_DEBIAN_DIR="$NANOFLANNSRC/packaging/debian/"
EMAIL4DEB="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>"

# Clean out dirs:
rm -fr $NANOFLANN_UBUNTU_OUT_DIR/

# -------------------------------------------------------------------
# And now create the custom packages for each Ubuntu distribution
# -------------------------------------------------------------------
LST_DISTROS=(bionic focal)

count=${#LST_DISTROS[@]}
IDXS=$(seq 0 $(expr $count - 1))

cp ${NANOFLANN_EXTERN_DEBIAN_DIR}/changelog /tmp/my_changelog


for IDX in ${IDXS};
do
	DEBIAN_DIST=${LST_DISTROS[$IDX]}

	# -------------------------------------------------------------------
	# Call the standard "prepare_debian.sh" script:
	# -------------------------------------------------------------------
	cd ${NANOFLANNSRC}
	bash scripts/prepare_debian.sh  -u -d ${DEBIAN_DIST}   # -s

	echo 
	echo "===== Distribution: ${DEBIAN_DIST}  ========="
	cd ${NANOFLANN_DEB_DIR}/nanoflann-${NANOFLANN_VER_MMP}${DEBIAN_DIST}/debian
	cp /tmp/my_changelog changelog
	DEBCHANGE_CMD="--newversion ${NANOFLANN_VERSION_STR}${DEBIAN_DIST}-1~ppa1~${DEBIAN_DIST}"
	echo "Changing to a new Debian version: ${DEBCHANGE_CMD}"
	echo "Adding a new entry to debian/changelog for distribution ${DEBIAN_DIST}"
	DEBEMAIL=${EMAIL4DEB} debchange $DEBCHANGE_CMD -b --distribution ${DEBIAN_DIST} --force-distribution New version of upstream sources.

	cp changelog /tmp/my_changelog 

	echo "Now, let's build the source Deb package with 'debuild -S -sa':"
	cd ..
	debuild -S -sa
	
	# Make a copy of all these packages:
	cd ..
	mkdir -p $NANOFLANN_UBUNTU_OUT_DIR/$DEBIAN_DIST
	cp nanoflann_* $NANOFLANN_UBUNTU_OUT_DIR/$DEBIAN_DIST/
	echo ">>>>>> Saving packages to: $NANOFLANN_UBUNTU_OUT_DIR/$DEBIAN_DIST/"
done


exit 0

