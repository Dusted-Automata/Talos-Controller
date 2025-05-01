build:
	cmake --build ./build --parallel
	# make -C ./build
.PHONY : build

run:
	./build/sim

controller:
	./build/controller

traj_run:
	./build/trajectory_run

.PHONY : run
