#
# TODO: Move `libmongoclient.a` to /usr/local/lib so this can work on production servers
#

CC := g++ # This is the main compiler
#CC := clang --analyze # and comment out the linker last line for sanity
SRCDIR := src
TESTDIR := tests
BUILDDIR := build
BIN := bin
TARGET := testDLP

SRCEXT := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
CFLAGS := -g # -Wall
LIB := -lm -lglpk
INC := -I include -I eigen-3.3.3

# build of DLP class, testDLP
$(TARGET): $(BUILDDIR)/dlp.o $(BUILDDIR)/testDLP.o
	@echo " Linking..."
	@mkdir -p $(BIN)
	@echo " $(CC) $^ -o $(BIN)/$(TARGET) $(LIB)"; $(CC) $^ -o $(BIN)/$(TARGET) $(LIB)

$(BUILDDIR)/dlp.o: $(SRCDIR)/dlp.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILDDIR)/testDLP.o: $(SRCDIR)/testDLP.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<
# build of tests insinde tests folder
# 1- testEigen
testEigen: $(BUILDDIR)/testEigen.o
	@echo " Linking..."
	@mkdir -p $(BIN)
	@echo " $(CC) $^ -o $(BIN)/testEigen"; $(CC) $^ -o $(BIN)/testEigen
$(BUILDDIR)/testEigen.o: $(TESTDIR)/testEigen.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<


clean:
	@echo " Cleaning...";
	@echo " $(RM) -r $(BUILDDIR) $(BIN)"; $(RM) -r $(BUILDDIR) $(BIN)

.PHONY: clean
