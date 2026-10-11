CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra -Wpedantic -pthread -Iinclude
LDFLAGS  ?=

BUILD := build

LIB_SRCS := $(wildcard src/*.cpp)
LIB_OBJS := $(patsubst src/%.cpp,$(BUILD)/%.o,$(LIB_SRCS))

.PHONY: all clean test

all: $(BUILD)/sender $(BUILD)/receiver $(BUILD)/test_link

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/sender: apps/sender.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

$(BUILD)/receiver: apps/receiver.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

$(BUILD)/test_link: tests/test_link.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

test: $(BUILD)/test_link
	$(BUILD)/test_link

clean:
	rm -rf $(BUILD)
