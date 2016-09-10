#!/bin/bash

if [ $# -eq 4 ]
then
	./random_input "${2}" "${3}" "${4}"
	
	count=1;
	while IFS='' read -r line || [[ -n "$line" ]]; do
		arr=($line)
		./gicp3 "${1}SCAN_normal_refc_gray_${2}.txt" "${1}SCAN_normal_refc_gray_${3}.txt" "${arr[0]}" "${arr[1]}" "${arr[2]}" "${arr[3]}" "${arr[4]}" "${arr[5]}" "${count}"
#		echo "./gicp3 ${1}SCAN_normal_refc_gray_${2}.txt ${1}SCAN_normal_refc_gray_${3}.txt ${arr[0]} ${arr[1]} ${arr[2]} ${arr[3]} ${arr[4]} ${arr[5]} ${count}"
		((count++))
	done < "${2}_${3}_${4}_input.txt"
	wait
else
	echo "usage: ./exp3.sh path_to_scans scan1 scan2 #inputs"
fi
