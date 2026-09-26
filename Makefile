CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-missing-field-initializers -I src -I third_party -MMD -MP
LDFLAGS ?=
PREFIX ?= /usr/local

SRCS = \
  src/scene/scene.cpp \
  src/scene/edit.cpp \
  src/scene/rig.cpp \
  src/ops/ops.cpp \
  src/agent/agent.cpp \
  src/io/gltf.cpp \
  src/io/gltf_import.cpp \
  src/render/software.cpp \
  src/rpc/rpc.cpp \
  src/app.cpp \
  src/main.cpp

OBJS = $(SRCS:.cpp=.o)
DEPS = $(OBJS:.o=.d)
BIN = kiln

.PHONY: all clean install demo rpc

all: $(BIN)

$(BIN): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(DEPS) $(BIN)

# Header dependencies: touching a .hpp recompiles everything that includes it.
-include $(DEPS)

install: $(BIN)
	install -d $(PREFIX)/bin
	install -m 755 $(BIN) $(PREFIX)/bin/kiln

# `make rpc` drives the JSON-RPC stdio mode end to end: ops in, structured
# results out, then a clean shutdown.
rpc: $(BIN)
	printf '%s\n' \
	  '{"jsonrpc":"2.0","id":1,"method":"op","params":{"op":"create","primitive":"cube","name":"Box"}}' \
	  '{"jsonrpc":"2.0","id":2,"method":"describe"}' \
	  '{"jsonrpc":"2.0","id":3,"method":"shutdown"}' \
	  | ./$(BIN) --rpc

demo: $(BIN)
	./$(BIN) --headless --preview examples/demo.png --export examples/demo.glb \
	  "add a red cube" \
	  "create plane Ground size 12 1 12" \
	  "color Ground gray" \
	  "create sphere Ball size 0.7 0.7 0.7 at 1.6 0.35 0.2" \
	  "color Ball gold" \
	  "lookat 5.2 3.2 6.4  0 0.4 0"
