#!/bin/bash

if [ $# -eq 3 ]
then
	./test_generator ${1}
	
	while IFS='' read -r line || [[ -n "$line" ]]; do
		arr=($line)
		./exp3.sh "${3}" "${arr[0]}" "${arr[1]}" "${2}" &
	done < scan_pairs.txt
	wait
else
	echo "usage: ./tester_exp3.sh #cases #inputs path_to_scans"
fi
