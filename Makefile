build:
	cmake --build ./build --parallel

# build:
# 	cmake --build ./build --parallel
	# make -C ./build
.PHONY : build

run:
	./build/sim

controller:
	./build/controller

wheelchair:
	./build/wheelchair

go1:
	./build/go1

.PHONY : run
