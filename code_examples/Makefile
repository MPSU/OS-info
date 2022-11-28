.PHONY: all clean

CC := gcc
CFLAGS := -Wall
LDFLAGS := 
TARGET := prog

MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
PROJECT_DIR := $(patsubst %/,%,$(dir $(MAKEFILE_PATH)))

SRC_DIR := $(PROJECT_DIR)/src
OBJ_DIR := $(CURDIR)/obj
SRC_FILES := $(wildcard $(SRC_DIR)/*.c)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC_FILES))

all: create_obj_dir $(TARGET)

create_obj_dir:
	mkdir -p $(OBJ_DIR)

$(TARGET): $(OBJ_FILES)
	g++ $(LD_FLAGS) -o $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	g++ $(CFLAGS) -c -o $@ $<

clean:
	rm -rf $(OBJ_DIR)