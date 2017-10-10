#
# TODO: Move `libmongoclient.a` to /usr/local/lib so this can work on production servers
#

CC := g++ # This is the main compiler
#CC := clang --analyze # and comment out the linker last line for sanity
SRCDIR := src
TESTDIR := tests
BUILDDIR := build
BIN := bin
TEST_DLP := testDLP
TEST_EIG := testEigen
TEST_MATHSET := testMathSet
TEST_COLLISION := exhaustiveCollisionTest
TEST_GEN_DATA := generate_data
SRCEXT := cpp
SOURCES := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))
CFLAGS := -g # -Wall
LIB := -lm -lglpk
INC := -I include/* -I eigen-3.3.3

# build all
.PHONY: all
all : msg $(TEST_DLP)  $(TEST_EIG) $(TEST_MATHSET) $(TEST_COLLISION) $(TEST_GEN_DATA)
msg:
	@echo "################## Building all test files ##################"

# build of DLP class, testDLP
$(TEST_DLP): $(BUILDDIR)/dlp.o $(BUILDDIR)/testDLP.o
	@echo " Linking..."
	@mkdir -p $(BIN)
	@echo " $(CC) $^ -o $(BIN)/$(TEST_DLP) $(LIB)"; $(CC) $^ -o $(BIN)/$(TEST_DLP) $(LIB)

$(BUILDDIR)/dlp.o: $(SRCDIR)/dlp.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILDDIR)/testDLP.o: $(TESTDIR)/testDLP.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

# build testEigen
$(TEST_EIG): $(BUILDDIR)/testEigen.o
	@echo " Linking..."
	@mkdir -p $(BIN)
	@echo " $(CC) $^ -o $(BIN)/$(TEST_EIG)"; $(CC) $^ -o $(BIN)/$(TEST_EIG)
$(BUILDDIR)/testEigen.o: $(TESTDIR)/testEigen.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<
# testMathSet
$(TEST_MATHSET): $(BUILDDIR)/testMathSet.o
	@echo " Linking..."
	@mkdir -p $(BIN)
	@echo " $(CC) $^ -o $(BIN)/$(TEST_MATHSET)"; $(CC) $^ -o $(BIN)/$(TEST_MATHSET)
$(BUILDDIR)/testMathSet.o: $(TESTDIR)/testMathSet.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<
# exhaustiveCollisionTest
$(TEST_COLLISION): $(BUILDDIR)/dlp.o $(BUILDDIR)/exhaustiveCollisionTest.o
	@echo " Linking..."
	@mkdir -p $(BIN)
	@echo " $(CC) $^ -o $(BIN)/$(TEST_COLLISION) $(LIB)"; $(CC) $^ -o $(BIN)/$(TEST_COLLISION) $(LIB)
$(BUILDDIR)/exhaustiveCollisionTest.o: $(TESTDIR)/exhaustiveCollisionTest.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<
# generate_date
$(TEST_GEN_DATA): $(BUILDDIR)/dlp.o $(BUILDDIR)/generate_data.o
	@echo " Linking..."
	@mkdir -p $(BIN)
	@echo " $(CC) $^ -o $(BIN)/$(TEST_GEN_DATA) $(LIB)"; $(CC) $^ -o $(BIN)/$(TEST_GEN_DATA) $(LIB)
$(BUILDDIR)/generate_data.o: $(TESTDIR)/generate_data.cpp
	@mkdir -p $(BUILDDIR)
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<


.PHONY: clean
clean:
	@echo " Cleaning...";
	@echo " $(RM) -r $(BUILDDIR) $(BIN)"; $(RM) -r $(BUILDDIR) $(BIN)
