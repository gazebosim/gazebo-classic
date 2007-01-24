#!/usr/bin/env bash

#USAGE="USAGE: upload_www.sh <dest> <name>"
#if [ $# -ne 2 ]; then
#  echo $USAGE
#  exit
#fi

# Figure out the SF user name; define SFUSER in your startup scripts
# is this is different from your username.
if [ -n "$SFUSER" ]; then 
  _SFUSER=$SFUSER
else
  _SFUSER=$USER
fi
echo "assuming your SourceForge username is $_SFUSER..."

# Document destination and name
DEST=$1
NAME=$1

# Create a tarball with the correct name and layout
rm -rf $NAME
cp -r html $NAME
tar cvzf $NAME.tgz $NAME

# Copy tarball to website
scp $NAME.tgz $SFUSER@shell.sourceforge.net:/home/groups/p/pl/playerstage/htdocs/doc/

# Untar the file and re-jig the permissions
ssh $_SFUSER@shell.sourceforge.net 'cd /home/groups/p/pl/playerstage/htdocs/doc/; tar xvzf '$NAME'.tgz; find . -type d | xargs chmod -fR 2775; find . -type f | xargs chmod -fR 664'
