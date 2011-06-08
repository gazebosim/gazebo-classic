#!/bin/sh

find . -name "*.cc" -exec cppcheck --enable=all -q -I ./src {} \;
