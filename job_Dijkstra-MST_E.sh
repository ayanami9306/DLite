#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./Dijkstra-MST E Dijkstra-MST_E.csv
	i=$(($i+1))
	sleep 1
done
echo 'Dijkstra-MST_E done'
