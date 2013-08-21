#!/bin/bash

hg_root=`hg root`
while read line
do
  id=`echo $line | awk '{print $1}'`
  src=`echo $line | awk '{print $2}'`
  dest=`echo $line | awk '{print $3}'`
  branch=`echo $line | awk '{print $4}'`

  msg="$id $branch"
  if ! hg log -r $dest &> /dev/null
  then
    echo $msg Unknown revision $dest, try: hg pull
    break
  fi

  if ! hg log -r $src &> /dev/null
  then
    echo $msg Unknown revision $src, try: hg pull "(it could also be a fork)"
    break
  fi

  ancestor=`hg log -r "ancestor($src,$dest)" | head -1 | sed -e 's@.*:@@'`
  
  if [ $dest != $ancestor ]
  then
    msg="$msg need to merge branch $branch with $dest"
  fi

  echo $msg

  files=`$hg_root/tools/bitbucket_pullrequests -f $id | grep '\.[ch]*$'`
  tmp=`mktemp -t asdfXXXXXXXXXX`
  for f in $files
  do
    prefix=`basename $f | sed -e 's@\..*$@@'`
    ext=`echo $f | sed -e 's@^.*\.@@'`
    tmp2="$tmp.$ext"
    tmp2base=`basename "$tmp"`
    hg cat -r $src $hg_root/$f > $tmp2
    python $hg_root/tools/cpplint.py $tmp2 2>&1 \
      | sed -e "s@$tmp2@$f@g" -e "s@$tmp2base@$prefix@g" \
      | grep -v 'Total errors found: 0'
    cppcheck -q --enable=style,performance,portability,information $tmp2 2>&1 \
      | sed -e "s@$tmp2@$f@g" \
      | grep -v 'use --check-config for details'
    rm $tmp2
  done
  rm $tmp
done
