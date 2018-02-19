#!/bin/bash
clang-format -i $(find . -name '*.cpp' -o -name '*.h')

# Check we have no typos.
which misspell 2>/dev/null >/dev/null
if [[ $? -eq 0 ]]; then
    misspell -error `find . -name '*.c' -or -name '*.h'`
fi
