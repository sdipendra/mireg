#!/bin/bash

if [ $# -eq 2 ]
then
	./gicp "${1}" "${2}" "variance"
	./gicp "${1}" "${2}" "reflectivity"
	./gicp "${1}" "${2}" "grayscale"
	./gicp "${1}" "${2}" "normal"
	./gicp "${1}" "${2}" "all"
else
	echo "usage: ./exp1.sh path_to_scan1 path_to_scan2"
fi
