#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./DLite-MST E DLite-MST_E.csv
	i=$(($i+1))
	sleep 1
done
echo 'DLite-MST_E done'
