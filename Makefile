# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -g

# Source files and output
SRCS = main.cpp utils.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = my_program

# Default rule
all: 
	python3 flashing.py

# Linking
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compilation
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJS) $(TARGET)

# Phony targets
.PHONY: all clean


# # The default target
# all:
# 	python3 flashing.py

# # Clean target (optional)
# clean:
# 	echo "Nothing to clean"

# .PHONY: all clean
