# Get the root directory of the git repo
REPO_ROOT=$(git rev-parse --show-toplevel)

# Run the archive command relative to that root
git -C "$REPO_ROOT" archive --format=zip HEAD -o "$REPO_ROOT/code.zip"
