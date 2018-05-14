# Install april hooks for april2
# Post problems on the issue tracker

exec 1>&2
#echo "Running "$0
DIR=`cd $(dirname "$0") && pwd`

#echo $DIR
CUSTOM_HOOKS_DIR=$DIR/hooks
echo "Custom hooks located in " $CUSTOM_HOOKS_DIR

ROOT_DIR=`dirname $DIR`
#echo $ROOT_DIR
GIT_HOOKS_DIR=$ROOT_DIR/.git/hooks
echo "Repo git hooks directory is " $GIT_HOOKS_DIR

#echo "Hiding current git hooks directory "
#mv $GIT_HOOKS_DIR $ROOT_DIR/.git/.hooks

#echo "Symlink git hook directory to custom hooks dir "
#ln -s $CUSTOM_HOOKS_DIR $ROOT_DIR/.git/hooks

echo "Copying hooks to git hooks directory"
cp $CUSTOM_HOOKS_DIR/* $GIT_HOOKS_DIR
