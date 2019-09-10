SHELL := /bin/bash

define run_docker
        nvidia-docker run --rm -it -u $$(id -u):$$(id -g) -v $$(pwd):$$(pwd) -w $$(pwd) \
				          -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
						  carlosgalvezp/sfnd:latest $(1)
endef

.PHONY: clean
clean:
	rm -rf build

.PHONY: build
build:
	mkdir -p build
	$(call run_docker, \
		/bin/bash -c   \
	    'cd build &&    \
	     cmake .. &&    \
	     make -j8'       \
	)

.PHONY: environment
environment: build
	$(call run_docker, ./build/environment)
