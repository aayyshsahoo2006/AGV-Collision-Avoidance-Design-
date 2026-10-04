CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude -pthread

SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
BIN_DIR = bin

CORE_SRCS = $(wildcard $(SRC_DIR)/*/*.cpp)
CORE_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(CORE_SRCS))

MAIN_SRC = $(SRC_DIR)/main.cpp
MAIN_OBJ = $(BUILD_DIR)/main.o

TEST_SRC = tests/TestRunner.cpp
TEST_OBJ = $(BUILD_DIR)/tests/TestRunner.o

TARGET_APP = $(BIN_DIR)/agv_sim
TARGET_TEST = $(BIN_DIR)/agv_tests

.PHONY: all clean test run directories

all: directories $(TARGET_APP) $(TARGET_TEST)

directories:
	@mkdir -p $(BUILD_DIR)/simulation $(BUILD_DIR)/sensor $(BUILD_DIR)/collision $(BUILD_DIR)/controller $(BUILD_DIR)/logging $(BUILD_DIR)/tests $(BIN_DIR) output

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/tests/%.o: tests/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET_APP): directories $(CORE_OBJS) $(MAIN_OBJ)
	$(CXX) $(CXXFLAGS) $(CORE_OBJS) $(MAIN_OBJ) -o $@

$(TARGET_TEST): directories $(CORE_OBJS) $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) $(CORE_OBJS) $(TEST_OBJ) -o $@

test: $(TARGET_TEST)
	@./$(TARGET_TEST)

run: $(TARGET_APP)
	@./$(TARGET_APP)

clean:
	rm -rf $(BUILD_DIR) $(BIN_DIR)
