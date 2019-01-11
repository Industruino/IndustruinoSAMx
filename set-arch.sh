#!/bin/bash

if [ "x${1}" == "xsamd" ]; then
   ./README-build.sh samd
   ./boards-build.sh samd
   ./platform-build.sh samd
   ./programmers-build.sh samd
elif [ "x${1}" == "xsaml" ]; then
   ./README-build.sh saml
   ./boards-build.sh saml
   ./platform-build.sh saml
   ./programmers-build.sh saml
elif [ "x${1}" == "xall" ]; then
   ./boards-build.sh all
   ./platform-build.sh all
   ./programmers-build.sh all
elif [ "x${1}" == "xclean" ]; then
   ./README-build.sh clean
   ./boards-build.sh clean
   ./platform-build.sh clean
   ./programmers-build.sh clean
elif [ "x${1}" == "xpurge" ]; then
   rm docs/README.md
   rmdir docs 2> /dev/null
   rm README-*
   rm boards-*
   rm platform-*
   rm programmers-*
else
   echo Unknown option \'${1}\'
   exit 1
fi
