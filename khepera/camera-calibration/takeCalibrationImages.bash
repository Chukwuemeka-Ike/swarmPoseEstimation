#!/bin/bash


# Set the pipes to allow you take a photo
echo "Setting the pipes to allow capture"
media-pipes.sh
media-formats.sh 752 480

echo "Taking photos at 1 second intervals"
mkdir -p images
for((i=1; i<=80; i++))
do
	v4l2grab -d /dev/video6 -o ./images/image$i.jpg -W 752 -H 480 -I -1
	echo "Photo $i"
	sleep 1
done
echo "Done taking photos!"