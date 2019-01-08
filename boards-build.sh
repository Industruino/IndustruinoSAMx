#!/bin/bash

if [ "x${1}" == "xsamd" ]; then
   cat boards-license.txt boards-samd.txt > boards.txt
elif [ "x${1}" == "xsaml" ]; then
   cat boards-license.txt boards-saml-menu.txt boards-saml.txt > boards.txt
elif [ "x${1}" == "xall" ]; then
   cat boards-license.txt boards-saml-menu.txt boards-samd.txt boards-saml.txt > boards.txt
elif [ "x${1}" == "xclean" ]; then
   rm boards.txt
else
   echo Unknown option \'${1}\'
   exit 1
fi

