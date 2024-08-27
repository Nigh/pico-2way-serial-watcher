

BUILD_DIR = build


.PHONY: clean default build rebuild

default: build

build:
	cmake . -G Ninja -B$(BUILD_DIR) -S.
	ninja -C $(BUILD_DIR)

clean:
	@if [ -d "./$(BUILD_DIR)" ]; then rm ./$(BUILD_DIR) -rf; fi

rebuild: clean build

format:
	bash format.sh
