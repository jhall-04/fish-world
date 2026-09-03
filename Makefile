# Point this at your raylib checkout
RAYLIB_SRC ?= ../raylib/src

# Where the React app serves static files from
WEB_OUT ?= ./public/boids

# ---- Web: emits boids.js + boids.wasm only, no shell page ----
web:
	mkdir -p $(WEB_OUT)
	em++ -o $(WEB_OUT)/boids.js boid2d.cpp -std=c++17 -Os -Wall \
	  $(RAYLIB_SRC)/libraylib.web.a \
	  -I$(RAYLIB_SRC) \
	  -s USE_GLFW=3 \
	  -s MODULARIZE=1 -s EXPORT_ES6=0 -s EXPORT_NAME=createBoids \
	  -s EXPORTED_FUNCTIONS=_main,_StopSim \
	  -s ALLOW_MEMORY_GROWTH=1 \
	  -s INVOKE_RUN=1 \
	  -DPLATFORM_WEB

# ---- Desktop ----
desktop:
	g++ -o boids boid2d.cpp -std=c++17 -O2 -Wall \
	  -I$(RAYLIB_SRC) $(RAYLIB_SRC)/libraylib.a \
	  -lGL -lm -lpthread -ldl -lrt -lX11

mac:
	clang++ -o boids boid2d.cpp -std=c++17 -O2 -Wall \
	  -I$(RAYLIB_SRC) $(RAYLIB_SRC)/libraylib.a \
	  -framework IOKit -framework Cocoa -framework OpenGL

clean:
	rm -f boids
	rm -rf $(WEB_OUT)

.PHONY: web desktop mac clean