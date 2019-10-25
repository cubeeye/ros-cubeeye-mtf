#!/bin/bash

# meerecompany driver install.
# product VID, PID setting
# MTF_API 2.0ver
# root authority command 'mtfinstall.sh'

CURDIR=`pwd`
echo "Current directory is $CURDIR. MTF driver will be installed..."
A=`whoami`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   exit 1;
fi

# Copy rules file
echo "Copy rules file."
cp cube-eye.rules /etc/udev/rules.d/216-cube-eye.rules

echo " MTF driver install finish."
