CC = gcc
CFLAGS = -std=gnu99 -Wall -Wextra -Wno-unused-parameter
CFLAGS += -include src/common.h
CFLAGS += -fno-pie
LDFLAGS = -no-pie
LIBS = -lm

# SDL2
CFLAGS += `sdl2-config --cflags`
LIBS += `sdl2-config --libs`

# Control the build verbosity
ifeq ("$(VERBOSE)","1")
    Q :=
    VECHO = @true
else
    Q := @
    VECHO = @printf
endif

OUT ?= build
SHELL_HACK := $(shell mkdir -p $(OUT))

BIN = build/jitboy
OBJS = core.o gbz80.o lcd.o memory.o emit.o interrupt.o main.o optimize.o audio.o save.o interpreter.o
OBJS := $(addprefix $(OUT)/, $(OBJS))
deps := $(OBJS:%.o=%.o.d)
GIT_HOOKS := .git/hooks/applied

all: CFLAGS += -O3
all: LDFLAGS += -O3
all: $(BIN) $(GIT_HOOKS)

sanitizer: CFLAGS += -Og -g -fsanitize=thread
sanitizer: LDFLAGS += -fsanitize=thread
sanitizer: $(BIN)

debug: CFLAGS += -g -D DEBUG
debug: DYNASMFLAGS += -D DEBUG
debug: $(BIN)

#interpreter only
onlyint: CFLAGS += -D INTERPRETER_ONLY
onlyint: $(BIN)

$(GIT_HOOKS):
	@scripts/install-git-hooks
	@echo

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $+ $(LIBS)
	
$(OUT)/%.o: src/%.c
	$(CC) -o $@ -c $(CFLAGS) $< -MMD -MF $@.d

$(OUT)/%.o: $(OUT)/%.c
	$(CC) -o $@ -c $(CFLAGS) $< -MMD -MF $@.d

LuaJIT/src/host/minilua.c:
	git submodule update --init
	touch $@

$(OUT)/minilua: LuaJIT/src/host/minilua.c
	$(CC) -o $@ $^ -lm

$(OUT)/emit.c: src/emit.dasc $(OUT)/minilua src/dasm_macros.inc
	$(OUT)/minilua LuaJIT/dynasm/dynasm.lua $(DYNASMFLAGS) -I src -o $@ $<
	
clean:
	$(RM) $(BIN) $(OBJS) $(deps)
	$(RM) $(OUT)/minilua $(OUT)/emit.c

-include $(deps)
