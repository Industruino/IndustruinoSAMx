#!/bin/bash

if [ "x${1}" == "xsamd" ]; then
   cat platform-license.txt platform-samd.txt platform-common.txt > platform.txt
elif [ "x${1}" == "xsaml" ]; then
   cat platform-license.txt platform-saml.txt platform-common.txt > platform.txt
elif [ "x${1}" == "xall" ]; then
   cat platform-license.txt platform-samd-saml.txt platform-common.txt > platform.txt
elif [ "x${1}" == "xclean" ]; then
   rm platform.txt
else
   echo Unknown option \'${1}\'
   exit 1
fi

