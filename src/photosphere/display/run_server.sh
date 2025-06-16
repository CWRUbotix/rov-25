#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "Running server from $SCRIPT_DIR..."

python3 -m http.server -d $SCRIPT_DIR