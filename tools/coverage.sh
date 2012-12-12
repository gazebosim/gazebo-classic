#!/bin/sh
rm /tmp/gazebo_all.info
rm /tmp/gazebo.info
rm -rf /tmp/coverage

lcov -d build/gazebo -c -o /tmp/gazebo_all.info
lcov -r /tmp/gazebo_all.info "/usr/*" -r /tmp/gazebo_all.info "*/include/*" -r /tmp/gazebo_all.info "*build*" -r /tmp/gazebo_all.info "*.hh" -o /tmp/gazebo.info
genhtml  /tmp/gazebo.info -o /tmp/coverage
scp -r /tmp/coverage root@gazebosim.org:/www/live/htdocs
