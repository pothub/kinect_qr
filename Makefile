out:	main.cpp
	@ g++ main.cpp -std=c++11 -o out `pkg-config opencv --cflags --libs` `pkg-config freenect2 --cflags --libs` `pkg-config --cflags zbar` `pkg-config --libs zbar`
	@ echo "----------program start!----------"
	@ ./out
	@ echo "-----------program end!-----------"
