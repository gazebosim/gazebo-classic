GAZEBO_ROOT=/home/jrivero/code/gazebo
# TODO: set iofwd for iostreams. Its own PRs?

for f in Color.hh Timer.hh Event.hh; do
    # exclusions
    case ${f} in
	Plugin.hh)
	Assert
	    echo "Exception! leave as it"
	    continue
    esac

    no_ext=${f%.hh}
    new=${no_ext}-fwd.hh
    echo "Generating $new"
    cp ${f} ${new}
    # Update guard for the new file
    sed -i -e "s:_HH_:FWD_HH_:g" ${new}
    # TODO: investigate what's going on if there are templates in the
    # header file. Probably need as it.

    # Need to respect: class, struct, enum, templated classes,
    #                 typedef outside class.
    # TODO: fix class comment to set: forward declaration doxygen
    # TODO: Remove all comments
    # TODO: Remove public, private or protected lines
    sed -i -e /.*\brief.*/d ${new}
    sed -i -e /.*\param.*/d ${new}
    sed -i -e /.*\return.*/d ${new}
    sed -i -e /.*\addtogroup.*/d ${new}
    sed -i -e /.*\private.*/d ${new}
    sed -i -e /.*\public.*/d ${new}
    sed -i -e /.*\protected.*/d ${new}

    sed -i -e s:GZ_.*_VISIBLE\ ::g ${new}

    # Add file to mercurial
    hg add ${new}

    # Change the include in the headers by the fowarded declaration
    # except for the own .cc file
    for h in $(find ${GAZEBO_ROOT} -name '*.hh' | grep -v ${no_ext}.cc); do
      if [[ -n $(grep -nH "^#include.*${f}" ${h}) ]]; then
	sed -i -e "s:${f}:${new}:g" ${h}
      fi
    done

    # Updatte CMakeLists.txt
    sed -i -e "/${f}/a  ${new}" CMakeLists.txt
done
