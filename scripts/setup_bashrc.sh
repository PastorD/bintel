#!/bin/bash

# use this file to add common definitions to the bashrc file. This file is loaded every time a terminal starts. 

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
PARENT="$(dirname "$DIR")"
cat "$PARENT/add_to_bashrc.txt" >> "$HOME/.bashrc"
