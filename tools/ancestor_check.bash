#!/bin/bash

hg_root=`hg root`
$hg_root/tools/bitbucket_pullrequests | \
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
    echo $msg Unknown revision $src, is this from a fork?
    break
  fi

  ancestor=`hg log -r "ancestor($src,$dest)" | head -1 | sed -e 's@.*:@@'`
  
  if [ $dest != $ancestor ]
  then
    msg="$msg need to merge branch $branch with $dest"
  fi

  echo $msg
done
