sed -n '20p' Variables.h
sed -n '17p' Variables.h
sed -n '18p' Variables.h
echo -e "\n"
sed -i "32s/.*/#define DLite/g" Variables.h
sed -n '32p' Variables.h
sed -i "34s/.*/#define MST_ALLOC/g" Variables.h
sed -n '34p' Variables.h
echo -e "Use DLite, MST ALLOC : DLite-MST\n"
g++ -O2 -o DLite-MST Dstar.cpp FloodFill.cpp main.cpp MST.cpp RobotClass.cpp Task_Allocation.cpp User_Defined_Function.cpp -lm -std=gnu++11
sed -i "32s/.*/#define DLite/g" Variables.h
sed -n '32p' Variables.h
sed -i "34s/.*/\/\/#define MST_ALLOC/g" Variables.h
sed -n '34p' Variables.h
echo -e "Use DLite, Random ALLOC : DLite-RANDOM\n"
g++ -O2 -o DLite-RANDOM Dstar.cpp FloodFill.cpp main.cpp MST.cpp RobotClass.cpp Task_Allocation.cpp User_Defined_Function.cpp -lm -std=gnu++11
sed -i "32s/.*/\/\/#define DLite/g" Variables.h
sed -n '32p' Variables.h
sed -i "34s/.*/#define MST_ALLOC/g" Variables.h
sed -n '34p' Variables.h
echo -e "Use Dijkstra, MST ALLOC : Dijkstra-MST\n"
g++ -O2 -o Dijkstra-MST Dstar.cpp FloodFill.cpp main.cpp MST.cpp RobotClass.cpp Task_Allocation.cpp User_Defined_Function.cpp -lm -std=gnu++11
sed -i "32s/.*/\/\/#define DLite/g" Variables.h
sed -n '32p' Variables.h
sed -i "34s/.*/\/\/#define MST_ALLOC/g" Variables.h
sed -n '34p' Variables.h
echo -e "Use Dijkstra, Random ALLOC : Dijkstra-RANDOM\n"
g++ -O2 -o Dijkstra-RANDOM Dstar.cpp FloodFill.cpp main.cpp MST.cpp RobotClass.cpp Task_Allocation.cpp User_Defined_Function.cpp -lm -std=gnu++11











