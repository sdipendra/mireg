#!/bin/bash

if [ $# -eq 2 ]
then
	printf -v j "%04d" $1
	for i in $(seq -f "%04g" $(($1+1)) $(($1+15)))
	do
		./gicp "${2}SCAN_normal_refc_gray_${i}.txt" "${2}SCAN_normal_refc_gray_${j}.txt" "all"
#		echo "./gicp ${2}SCAN_normal_refc_gray_${i}.txt ${2}SCAN_normal_refc_gray_${j}.txt all"
	done
else
	echo "usage: ./exp2.sh scan_no_reference path_to_scans"
fi
