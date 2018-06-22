#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./Dijkstra-RANDOM H Dijkstra-RANDOM_H.csv
	i=$(($i+1))
	sleep 1
done
echo 'Dijkstra-RANDOM_H done'
