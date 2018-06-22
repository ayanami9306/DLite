#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./DLite-MST H DLite-MST_H.csv
	i=$(($i+1))
	sleep 1
done
echo 'DLite-MST_H done'
