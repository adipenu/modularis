#! /bin/bash

# Store the file path for the repository
DOCS_DIR="$(realpath $(dirname $BASH_SOURCE)/../docs/_build/html)"

# Convert path to a URL that can be opened by browser
DOCS_URL="file://$(realpath $DOCS_DIR/index.html)"

# Try to open the program set to $BROWSER
if [[ ! -z ${BROWSER:-} ]]; then
  $BROWSER $DOCS_URL
  exit
fi

# If $BROWSER wasn't set, use the default x-www-browser
if which x-www-browser; then
  x-www-browser $DOCS_URL
else
  echo "Please set \$DISPLAY to your preferred browser"
fi


