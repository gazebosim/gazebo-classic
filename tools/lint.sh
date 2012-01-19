#!/bin/sh
find ./src ./test ./plugins ./player ./tools -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py
