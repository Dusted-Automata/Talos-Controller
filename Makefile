build:
	make -C ./build
.PHONY : build

run:
	./build/sim

controller:
	./build/controller

.PHONY : run
