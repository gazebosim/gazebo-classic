#!/usr/bin/env python
##############################################################################
#Author: Tashwin Khurana
#Version: 1.0
#Package: gazebo_osm
#
#Description:
#            Program to test pep8 conformance for the files in the package
##############################################################################

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
