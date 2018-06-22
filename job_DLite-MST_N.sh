#! /bin/ksh

i=0
while [ $i -lt 1000 ]
do
	./DLite-MST N DLite-MST_N.csv
	i=$(($i+1))
	sleep 1
done
echo 'DLite-MST_N done'
