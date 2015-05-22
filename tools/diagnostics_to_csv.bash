# csv header
grep -rnI ODEPhysics::UpdatePhysics `ls | sort` \
  | sed -e 's@_2015.*Physics@@' \
        -e 's@ [0-9][0-9\.eE\-]* @  @g' \
        -e 's@^[a-zA-Z0-9_\-]*@name@' \
        -e 's@  *$@@' \
        -e 's@  *@,@g' \
  | head -1
# csv rows
grep -rnI ODEPhysics::UpdatePhysics `ls | sort` \
  | sed -e 's@_2015.*Physics@@' \
        -e 's@ maxAbs @  @' \
        -e 's@ mean @  @' \
        -e 's@ min @  @' \
        -e 's@ var @  @' \
        -e 's@  *$@@' \
        -e 's@  *@,@g'
