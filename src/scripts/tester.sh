#!/bin/bash

if [ $# -eq 2 ]
then
	cases=${1}
	./test_generator ${cases}
	
	path=${2}
	
	while IFS='' read -r line || [[ -n "$line" ]]; do
		arr=($line)
		./exp1.sh "${path}SCAN_normal_refc_gray_${arr[0]}.txt" "${path}SCAN_normal_refc_gray_${arr[1]}.txt" &
	done < scan_pairs.txt
	wait
else
	echo "usage: ./tester.sh #cases path_to_scans"
fi
