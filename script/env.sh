#!/bin/sh
# NvBREP development environment (Linux/macOS).
# Usage: . script/env.sh [OCCT root]
# Adds the NvBREP build output and the OCCT runtime libraries to the loader path.
# The default OCCT location is the sibling checkout ../OCCT
# (relative to the NvBREP project root).

# When sourced, $0 refers to the caller's shell; BASH_SOURCE (bash/zsh) points to
# this script. ${BASH_SOURCE:-$0} is correct for both sourcing and direct execution.
NV_SCRIPT="${BASH_SOURCE:-$0}"
NV_ROOT="$(cd "$(dirname "$NV_SCRIPT")/.." && pwd)"

if [ -n "$1" ]; then
  OCCT_ROOT="$1"
elif [ -z "$OCCT_ROOT" ]; then
  OCCT_ROOT="$NV_ROOT/../OCCT"
fi

if [ -d "$OCCT_ROOT/install/lib" ]; then
  OCCT_LIB="$OCCT_ROOT/install/lib"
else
  OCCT_LIB="$OCCT_ROOT/lib"
fi

if [ "$(uname)" = "Darwin" ]; then
  export DYLD_LIBRARY_PATH="$NV_ROOT/build/lib:$OCCT_LIB${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="$NV_ROOT/build/lib:$OCCT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

echo "NvBREP root : $NV_ROOT"
echo "OCCT root     : $OCCT_ROOT"
echo "OCCT lib dir  : $OCCT_LIB"
