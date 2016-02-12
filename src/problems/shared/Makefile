MODULE_NAME = shared
CHILD_DIRS := geometry

# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-g -c -std=c++11

PYLIBPATH = $(shell python-config --exec-prefix)/lib

INCLUDEFLAGS=-I/usr/local/include/fcl/ -I/usr/include/python2.7/

override INCLUDEFLAGS += -I/usr/include/eigen3 -I/usr/local/include/openrave-0.9 -I/usr/include/boost -I/usr/include/jsoncpp/
override INCLUDEFLAGS += -L/usr/lib/ -L/usr/local/lib/

LDFLAGS= -L$(PYLIBPATH) $(shell python-config --libs) -lfcl -lopenrave0.9 -lopenrave0.9-core
OPTS = $(shell python-config --include) -O2
BOOSTFLAGS= -lboost_system -lboost_program_options -lboost_python

ifdef HAS_ROOT_MAKEFILE

include .make/template.mk
include .make/build.mk

else
REDIRECT=$(MODULE_NAME)
.PHONY: $(MAKECMDGOALS) call-upwards
$(MAKECMDGOALS): all call-upwards ;

all: kin.so util.so obstacle.so environment.so copy clean

kin.so: Kinematics.o
	$(CC) $(INCLUDEFLAGS) -Wl,-rpath,$(PYLIBPATH) -shared $< -o $@ $(LDFLAGS) $(BOOSTFLAGS)
		
Kinematics.o: Kinematics.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
util.so: Kinematics.o Terrain.o Obstacle.o Utils.o
	$(CC) $(INCLUDEFLAGS) -Wl,-rpath,$(PYLIBPATH) -shared $^ -o $@ $(LDFLAGS) $(BOOSTFLAGS)
	
Utils.o: utils.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
obstacle.so: Kinematics.o Utils.o Terrain.o Obstacle.o
	$(CC) $(INCLUDEFLAGS) -Wl,-rpath,$(PYLIBPATH) -shared $^ -o $@ $(LDFLAGS) $(BOOSTFLAGS)
	
Obstacle.o: Obstacle.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
Terrain.o: Terrain.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
environment.so: Environment.o
	$(CC) $(INCLUDEFLAGS) -Wl,-rpath,$(PYLIBPATH) -shared $^ -o $@ $(LDFLAGS) $(BOOSTFLAGS)
	
Environment.o: Environment.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) -fPIC $< -o $@
	
	
copy:
	cp *.so ../../../python/
clean:
	rm *.o *.so
call-upwards:
	@$(MAKE) --no-print-directory -C .. $(MAKECMDGOALS) REDIRECT=$(REDIRECT)
endif
