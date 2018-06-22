#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./DLite-RANDOM E DLite-RANDOM_E.csv
	i=$(($i+1))
	sleep 1
done
echo 'DLite-RANDOM_E done'
