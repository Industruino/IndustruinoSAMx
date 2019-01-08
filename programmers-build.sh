#!/bin/bash

if [ "x${1}" == "xsamd" ]; then
   cat programmers-license.txt programmers-samd.txt > programmers.txt
elif [ "x${1}" == "xsaml" ]; then
   cat programmers-license.txt programmers-saml.txt > programmers.txt
elif [ "x${1}" == "xall" ]; then
   cat programmers-license.txt programmers-samd-saml.txt > programmers.txt
elif [ "x${1}" == "xclean" ]; then
   rm programmers.txt
else
   echo Unknown option \'${1}\'
   exit 1
fi

