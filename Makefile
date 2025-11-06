build:
	cmake --build ./build --parallel

# build:
# 	cmake --build ./build --parallel
	# make -C ./build
.PHONY : build

run:
	./build/main --robot ./robot_configs/sim_bot.json --waypoints ./waypoints/PT_ping_pong.json

controller:
	./build/controller

wheelchair:
	./build/wheelchair

go1:
	./build/go1

main:
	./build/main

# .PHONY : run
