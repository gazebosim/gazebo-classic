GAZEBO_ROOT=/home/jrivero/code/gazebo

for f in $(ls *.hh | grep -v Private); do 
    # exclusions
    case ${f} in
	Plugin.hh)
	    echo "Exception! leave as it"
	    continue
    esac

    no_ext=${f%.hh} 
    new=${no_ext}-fwd.hh 
    echo "Generating $new"
    cp ${f} ${new}
    # Update guard for the new file
    sed -i -e "s:_HH_:FWD_HH_:g" ${new}
    # TODO: set iofwd for iostreams. Its own PRs?
    # TODO: investigate what's going on if there are templates in the
    # header file. Probably need as it.

    # Need to respect: class, struct, enum, templated classes,
    #                 typedef outside class.
    # TODO: fix class comment to set: forward declaration doxygen
    # TODO: Remove all comments
    # TODO: Remove public, private or protected lines


    # Add file to mercurial
    hg add ${new}

    header_list="${header_list} ${new}"
    cmake_list="${full_list}\n${new}"

    # Change the include in the headers by the fowarded declaration
    # except for the own .cc file
    for h in $(find ${GAZEBO_ROOT} -name '*.hh' | grep -v ${no_ext}.cc); do
      if [[ -n $(grep -nH "^#include.*${f}" ${h}) ]]; then
	sed -i -e "s:${f}:${new}:g" ${h}
      fi
    done
done

echo "Remember to add this list to install headers in the CMAkeLists.txt"
echo -e ${cmake_list}
