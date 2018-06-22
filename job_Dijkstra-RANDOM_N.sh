#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./Dijkstra-RANDOM N Dijkstra-RANDOM_N.csv
	i=$(($i+1))
	sleep 1
done
echo 'Dijkstra-RANDOM_N done'
