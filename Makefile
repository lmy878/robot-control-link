CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra -Wpedantic -pthread -Iinclude
LDFLAGS  ?=

BUILD := build

LIB_SRCS := $(wildcard src/*.cpp)
LIB_OBJS := $(patsubst src/%.cpp,$(BUILD)/%.o,$(LIB_SRCS))

TEST_BUILD := $(BUILD)/tests
PROBE_BINS := $(patsubst tests/probe/%.cpp,$(TEST_BUILD)/%,$(wildcard tests/probe/*.cpp))
UNIT_BINS  := $(patsubst tests/unit/%.cpp,$(TEST_BUILD)/%,$(wildcard tests/unit/*.cpp))

.PHONY: all clean test

all: $(BUILD)/sender $(BUILD)/receiver $(BUILD)/controller $(BUILD)/robot_node $(BUILD)/monitor

# Build the test probes and unit-test binaries (run via tests/run_tests.py).
test: $(PROBE_BINS) $(UNIT_BINS)

$(TEST_BUILD):
	mkdir -p $(TEST_BUILD)

$(PROBE_BINS): $(TEST_BUILD)/%: tests/probe/%.cpp $(LIB_OBJS) | $(TEST_BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

$(UNIT_BINS): $(TEST_BUILD)/%: tests/unit/%.cpp $(LIB_OBJS) | $(TEST_BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

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

$(BUILD)/monitor: apps/monitor.cpp $(LIB_OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $< $(LIB_OBJS) -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD)
