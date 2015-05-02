WORLD=$1
PRESET=$2
PREFIX=${WORLD}_${PRESET}
DIAG=$HOME/.gazebo/diagnostics
STDOUT=${DIAG}/stdout.txt
/usr/bin/time -a -p -o ${DIAG}/stdout.txt \
  gzserver --verbose $HOME/ws/gazebo_sdformat/src/gazebo/test/worlds/${WORLD}.world -o ${PRESET} \
  --iters 100000 > ${STDOUT} 2>&1
cat ${STDOUT}
SUFFIX=`ls -t ${DIAG} | grep ^201 | head -1`
mv ${STDOUT} ${DIAG}/${SUFFIX}/
mv ${DIAG}/${SUFFIX} ${DIAG}/${PREFIX}_${SUFFIX}
