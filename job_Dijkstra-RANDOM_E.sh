#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./Dijkstra-RANDOM E Dijkstra-RANDOM_E.csv
	i=$(($i+1))
	sleep 1
done
echo 'Dijkstra-RANDOM_E done'
