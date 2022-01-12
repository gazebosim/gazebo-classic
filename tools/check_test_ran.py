#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: check_test_ran.py 16671 2012-04-27 16:15:28Z dthomas $

"""
Writes a test failure out to test file if it doesn't exist or is empty.
"""

# Adapted from rosunit/check_test_ran.py

from __future__ import print_function
NAME="check_test_ran.py"

import os
import re
import string
import sys
import subprocess

def usage():
    print("""Usage:
\t%s test-file.xml
"""%(NAME), file=sys.stderr)
    print(sys.argv)
    sys.exit(getattr(os, 'EX_USAGE', 1))

def file_contains_string(filename, string):
    with open(filename, 'r') as f:
        if string in f.read():
            return True
    return False

def run_xsltproc(stylesheet, document):
    test_ran_message = ''
    try:
        process = subprocess.Popen(['xsltproc', stylesheet, document], stdout=subprocess.PIPE)
        stdout, stderr = process.communicate()
        if process.returncode != 0:
            test_ran_message = "Error while converting QTest XML to junit with xsltproc."
        # Overwrite same document
        open(document, 'w').write(stdout.decode())
    except OSError as err:
        test_ran_message = "Unable to find xsltproc. Can not parse output test for QTest suite."
    if len(test_ran_message) > 0:
        test_name = os.path.basename(document)
        f = open(document, 'w')
        d = {'test': test_name, 'test_file': document, 'test_no_xml': test_name.replace('.xml', ''), 'message': test_ran_message}
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="%(test_no_xml)s">
    <failure message="%(message)s" type=""/>
  </testcase>
</testsuite>"""%d)
        sys.exit(getattr(os, 'EX_USAGE', 1))

def check_main():
    if len(sys.argv) < 2:
        usage()
    test_file = sys.argv[1]
        
    # kill rogue gzservers from INTEGRATION_world_clone test
    # https://github.com/osrf/gazebo/issues/1299
    if 'INTEGRATION_world_clone' in test_file:
        process = subprocess.Popen(['ps', 'ax'], stdout=subprocess.PIPE)
        stdout, stderr = process.communicate()
        for line in string.split(stdout, '\n'):
            if line.find('gzserver /tmp/clone.11347.world') >= 0:
                print(line)
                m = re.search('^ *([0-9]+) .*', line)
                if m == None:
                    print("Error identifying process id, not killing any gzserver's")
                else:
                    pid = m.group(1)
                    print("killing gzserver with pid %s" % (pid))
                    subprocess.call(["kill", "-9", "%s" % (pid)])

    print("Checking for test results in %s"%test_file)
    
    if not os.path.exists(test_file) or os.path.getsize(test_file) == 0:
        if not os.path.exists(os.path.dirname(test_file)):
            os.makedirs(os.path.dirname(test_file))
            
        print("Cannot find results, writing failure results to", test_file)
        
        with open(test_file, 'w') as f:
            test_name = os.path.basename(test_file)
            d = {'test': test_name, 'test_file': test_file , 'test_no_xml': test_name.replace('.xml', '')}
            f.write("""<?xml version="1.0" encoding="UTF-8"?>
<testsuite tests="1" failures="1" time="1" errors="0" name="%(test)s">
  <testcase name="test_ran" status="run" time="1" classname="%(test_no_xml)s">
    <failure message="Unable to find test results for %(test)s, test did not run.\nExpected results in %(test_file)s" type=""/>
  </testcase>
</testsuite>"""%d)
        sys.exit(getattr(os, 'EX_USAGE', 1))

    # Checking if test is a QTest file
    if file_contains_string(test_file, "QtVersion"):
        print("Detect QTest xml file. Converting to JUNIT ...")
        stylesheet = os.path.dirname(os.path.abspath(__file__)) + "/qtest_to_junit.xslt"
        run_xsltproc(stylesheet, test_file)

if __name__ == '__main__':
    check_main()
