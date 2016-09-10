#!/bin/bash

if [ $# -eq 2 ]
then
	./test_generator_exp2 ${1}
	
	while IFS='' read -r line || [[ -n "$line" ]]; do
		./exp2.sh "${line}" "${2}" &
	done < reference_scans_exp2.txt
	wait
else
	echo "usage: ./tester_exp2.sh #cases path_to_scans"
fi
