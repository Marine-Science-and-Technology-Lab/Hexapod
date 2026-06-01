all: postrack.o jog.o vector.o ipassigner.o message.o motioncomplete.o commands.o recordposition.o contour.o remoteclient.o remoteserver.o

postrack.o: examples.h
	g++ position_tracking.cpp position_tracking_example.cpp -lgclib -lgclibo -o position_tracking_example.out
jog.o: examples.h
	g++ jog.cpp jog_example.cpp -lncurses -lgclib -lgclibo -o jog_example.out
vector.o: examples.h
	g++ vector.cpp vector_example.cpp -lgclib -lgclibo -o vector_example.out
ipassigner.o: examples.h
	g++ ip_assigner.cpp ip_assigner_example.cpp -lgclib -lgclibo -o ip_assigner_example.out
message.o: examples.h
	g++ message.cpp message_example.cpp -lgclib -lgclibo -o message_example.out
commands.o: examples.h
	g++ commands.cpp commands_example.cpp -lgclib -lgclibo -o commands_example.out
motioncomplete.o: examples.h
	g++ motion_complete.cpp motion_complete_example.cpp -lgclib -lgclibo -o motion_complete_example.out
recordposition.o: examples.h
	g++ record_position.cpp record_position_example.cpp -lgclib -lgclibo -o record_position_example.out
contour.o: examples.h 
	g++ record_position.cpp contour.cpp contour_example.cpp -lgclib -lgclibo -o contour_example.out
remoteclient.o: examples.h
	g++ remote_client.cpp remote_client_example.cpp -lgclib -lgclibo -lncurses -o remote_client_example.out
remoteserver.o: examples.h
	g++ remote_server.cpp remote_server_example.cpp -lgclib -lgclibo -lncurses -o remote_server_example.out
