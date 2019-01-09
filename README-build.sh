#!/bin/bash

if [ "x${1}" == "xsamd" ]; then
   cp README-samd.md README.md
elif [ "x${1}" == "xsaml" ]; then
   cp README-saml.md README.md
elif [ "x${1}" == "xclean" ]; then
   rm README.md
else
   echo Unknown option \'${1}\'
   exit 1
fi
