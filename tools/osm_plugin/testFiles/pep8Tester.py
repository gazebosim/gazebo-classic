#!/usr/bin/env python
# Copyright 2014 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Tashwin Khurana
# Description: Program to test pep8 conformance for the files in the package

import pep8
import os

def test_pep8_conformance():
    """Test source code for PEP8 conformance"""
    pep8style = pep8.StyleGuide()
    report = pep8style.options.report
    report.start()
    pep8style.input_dir(os.path.join(os.path.dirname(__file__),
                                     '../', './'))
    report.stop()
    assert report.total_errors == 0,\
        "Found '{0}' code style errors" +\
        " (and warnings).".format(report.total_errors)

test_pep8_conformance()
