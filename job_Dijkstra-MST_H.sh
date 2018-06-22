#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./Dijkstra-MST H Dijkstra-MST_H.csv
	i=$(($i+1))
	sleep 1
done
echo 'Dijkstra-MST_H done'
