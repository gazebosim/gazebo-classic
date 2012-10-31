#!/usr/bin/env python

# Convert output from Google's cpplint.py to the cppcheck XML format for
# consumption by the Jenkins cppcheck plugin.

# Reads from stdin and writes to stderr (to mimic cppcheck)


import sys
import re

def parse():
    # TODO: do this properly, using the xml module.
    # Write header
    sys.stderr.write('''<?xml version="1.0" encoding="UTF-8"?>''')
    sys.stderr.write('''<results>''')

    # Do line-by-line conversion
    r=re.compile('([^:]*):([0-9]*):  ([^\[]*)\[([^\]]*)\] \[([0-9]*)\].*')
    

    # Write footer
    sys.stderr.write('''</results>''')


if __name__ == '__main__':
    parse()
