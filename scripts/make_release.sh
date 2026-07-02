#!/usr/bin/env bash
# Prepares and publishes a new nanoflann release.
#
# Steps:
#   1. Ensure the git working tree is clean and in sync with the remote.
#   2. Bump the version macro in include/nanoflann.hpp and commit it.
#   3. Run catkin_generate_changelog and let the user edit CHANGELOG.rst.
#   4. Commit the changelog.
#   5. Run catkin_prepare_release with the requested version bump.
#
# Usage:
#   scripts/make_release.sh [--bump major|minor|patch]
#
# If --bump is not given, it defaults to "patch", same as
# catkin_prepare_release.

set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

BUMP="patch"

while [ $# -gt 0 ]; do
    case "$1" in
        --bump)
            BUMP="$2"
            shift 2
            ;;
        --bump=*)
            BUMP="${1#--bump=}"
            shift
            ;;
        -h|--help)
            sed -n '2,20p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

case "$BUMP" in
    major|minor|patch) ;;
    *)
        echo "ERROR: --bump must be one of: major, minor, patch (got '$BUMP')" >&2
        exit 1
        ;;
esac

HPP_FILE="include/nanoflann.hpp"
CHANGELOG_FILE="CHANGELOG.rst"

# --------------------------------------------------------------------------
# 1. Ensure git is clean and in sync with the remote.
# --------------------------------------------------------------------------
if [ -n "$(git status --porcelain)" ]; then
    echo "ERROR: working tree is not clean. Commit or stash your changes first." >&2
    exit 1
fi

echo "Fetching and pulling latest changes..."
git fetch origin
git pull --ff-only

echo "Pushing any local commits..."
git push

if [ -n "$(git status --porcelain)" ]; then
    echo "ERROR: working tree became dirty after pull. Aborting." >&2
    exit 1
fi

# --------------------------------------------------------------------------
# 2. Compute the new version and bump the nanoflann.hpp version macro.
# --------------------------------------------------------------------------
CUR_VERSION=$(grep -oP '#define NANOFLANN_VERSION_STRING "\K[0-9]+\.[0-9]+\.[0-9]+' "$HPP_FILE")
IFS='.' read -r CUR_MAJOR CUR_MINOR CUR_PATCH <<< "$CUR_VERSION"

case "$BUMP" in
    major)
        NEW_MAJOR=$((CUR_MAJOR + 1))
        NEW_MINOR=0
        NEW_PATCH=0
        ;;
    minor)
        NEW_MAJOR=$CUR_MAJOR
        NEW_MINOR=$((CUR_MINOR + 1))
        NEW_PATCH=0
        ;;
    patch)
        NEW_MAJOR=$CUR_MAJOR
        NEW_MINOR=$CUR_MINOR
        NEW_PATCH=$((CUR_PATCH + 1))
        ;;
esac

NEW_VERSION="${NEW_MAJOR}.${NEW_MINOR}.${NEW_PATCH}"
NEW_VERSION_HEX=$(printf "0x%02x%02x%02x" "$NEW_MAJOR" "$NEW_MINOR" "$NEW_PATCH")

echo "Bumping version: ${CUR_VERSION} -> ${NEW_VERSION} (${BUMP})"

sed -i \
    -e "s/#define NANOFLANN_VERSION_STRING \"[0-9]\+\.[0-9]\+\.[0-9]\+\"/#define NANOFLANN_VERSION_STRING \"${NEW_VERSION}\"/" \
    -e "s/#define NANOFLANN_VERSION 0x[0-9a-fA-F]\+/#define NANOFLANN_VERSION ${NEW_VERSION_HEX}/" \
    "$HPP_FILE"

git add "$HPP_FILE"
git commit -m "Bump version to ${NEW_VERSION}"

# --------------------------------------------------------------------------
# 3. Generate the changelog and let the user review/edit it.
# --------------------------------------------------------------------------
echo "Generating changelog..."
catkin_generate_changelog -y

echo
echo "Please review/edit '${CHANGELOG_FILE}' now (e.g. in another terminal)."
read -r -p "Press ENTER when you are done editing the changelog to continue... "

if [ -z "$(git status --porcelain -- "$CHANGELOG_FILE")" ]; then
    echo "No changes detected in ${CHANGELOG_FILE}, nothing to commit."
else
    git add "$CHANGELOG_FILE"
    git commit -m "Update changelog for ${NEW_VERSION}"
fi

# --------------------------------------------------------------------------
# 4. Run catkin_prepare_release with the matching version bump.
# --------------------------------------------------------------------------
echo "Running catkin_prepare_release --bump ${BUMP}..."
catkin_prepare_release --bump "$BUMP"
