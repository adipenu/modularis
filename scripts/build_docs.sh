#! /bin/bash

# Store the file path for the repository
REPO_DIR="$(realpath $(dirname $BASH_SOURCE)/..)"

# Build the docs and place files in the repository.
sphinx-build -b html $REPO_DIR/docs $REPO_DIR/docs/_build/html
