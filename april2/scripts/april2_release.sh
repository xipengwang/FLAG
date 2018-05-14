#!/bin/bash -e

if [ "$#" -ne 1 ]; then
    echo "Usage: $(basename $0) tag-name"
    echo "tag-name is something like 'v0.1'"
    exit 1
fi

if [ -n "$(git status --porcelain)" ]; then
    echo "Error: working directory not clean"
    echo "This script checks out a clean copy of HEAD for release"
    read -r -p "Do you want to proceed? [y/N] " response
    case $response in
        [yY][eE][sS]|[yY])
            ;;
        *)
            exit 1
            ;;
    esac
fi

BASEDIR=$(cd -P $(dirname $0) && cd .. && pwd)
TMPDIR=$(mktemp -d)
OUTDIR=$(mktemp -d)
REMOTE="git@april.eecs.umich.edu:april2.git"

echo "Exporting a clean copy of the repository..."
cd $BASEDIR
git archive --format=tar HEAD | (cd $TMPDIR && tar xf -)
COMMIT_MSG="Source branch: $(git rev-parse --abbrev-ref HEAD) @ $(git rev-parse --short HEAD)"

echo "Adding license headers..."
cd $TMPDIR
FILES=$(find . -type f -exec echo {} +)
$BASEDIR/bin/licensify -l $BASEDIR/licenses $FILES $OUTDIR
rm -rf $TMPDIR

echo "Creating a commit on the release branch..."
cd $OUTDIR
git init
git remote add origin $REMOTE
git fetch
if git rev-parse --verify origin/master; then
    # Make a dummy commit of the new version
    git checkout -b new
    git add -f .
    git commit -m "tmp"
    # Generate a diff against the previous version
    git checkout -b master --track origin/master
    git diff master new | git apply
else
    git checkout -b master
fi
# Create a new commit on master
git add -f .
git commit -em "$COMMIT_MSG"

# Test build
make -j

echo
echo "Before pushing, please make sure everything looks okay."
echo "Release is staged in: $OUTDIR"
echo "Destination is: $REMOTE"
read -r -p "Do you want to proceed? [y/N] " response
case $response in
    [yY][eE][sS]|[yY])
        ;;
    *)
        echo "Push aborted"
        echo "Leaving files in $OUTDIR for manual intervention."
        exit 1
        ;;
esac

git push origin master
echo "Push successful"
rm -rf $OUTDIR

# Tag this release on internal repo
cd $BASEDIR
git tag -a $1
git push origin $1
echo "Tagged release as $1"
echo "Done"
