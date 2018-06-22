#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./DLite-RANDOM H DLite-RANDOM_H.csv
	i=$(($i+1))
	sleep 1
done
echo 'DLite-RANDOM_H done'
