#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MODELS_DIR=$DIR

# optional argument giving a directory
if [ "$1" != "" ]; then
  MODELS_DIR=$1
fi

for dir in $MODELS_DIR/*/
do
  dir=${dir%*/}
  echo -e "\e[93mCreating thumbnails for [${dir##*/}]...\e[39m"
  rm -rf $MODELS_DIR/${dir##*/}/thumbnails
  if [[ -f $MODELS_DIR/${dir##*/}/model.sdf ]]; then
	  # generate thumbnails with green bg
    gzserver -s libModelPropShop.so $DIR/green.world --propshop-save "$MODELS_DIR/${dir##*/}/thumbnails" --propshop-model "$MODELS_DIR/${dir##*/}/model.sdf"
    for thumb in $MODELS_DIR/${dir##*/}/thumbnails/*.png
    do
      # make green bg transparent
      convert $thumb -fuzz 30% -transparent '#00ff00' $thumb

      # crop transparent ends
      convert $thumb -trim $thumb

      # add shadow
      convert -background none\
              -fill black \
              $thumb \
              \( +clone -background black  -shadow 100x10+0+0 \) +swap \
              -background none   -layers merge +repage  $thumb

    done
  fi
done
