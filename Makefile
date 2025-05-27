build:
	cmake --build ./build --parallel
	# make -C ./build
.PHONY : build

run:
	./build/sim

controller:
	./build/controller

wheelchair:
	./build/wheelchair

.PHONY : run
