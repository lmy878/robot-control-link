CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra -Wpedantic -pthread -Iinclude
LDFLAGS  ?=

BUILD := build

LIB_SRCS := $(wildcard src/*.cpp)
LIB_OBJS := $(patsubst src/%.cpp,$(BUILD)/%.o,$(LIB_SRCS))

.PHONY: all clean

all: $(BUILD)/sender $(BUILD)/receiver $(BUILD)/controller $(BUILD)/robot_node

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/sender: apps/sender.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

$(BUILD)/receiver: apps/receiver.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

$(BUILD)/controller: apps/controller.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

$(BUILD)/robot_node: apps/robot_node.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD)
