# ProKey Makefile
# Automatische Erkennung des Betriebssystems
ifdef OS
   RM = del /Q
   FixPath = $(subst /,\,$1)
   SUFFIX = .exe
   LIB_EXT = .dll
   CFLAGS_EXTRA = -DBUILD_PROKEY_DLL
   LDFLAGS = -shared
else
   ifeq ($(shell uname), Linux)
      RM = rm -f
      FixPath = $1
      SUFFIX = 
      LIB_EXT = .so
      CFLAGS_EXTRA = -mrdrnd -fPIC
      LDFLAGS = -shared
   endif
endif

CC = gcc
CXX = g++
CFLAGS = -std=c99 -O2 -Wall $(CFLAGS_EXTRA)
CXXFLAGS = -std=c++11 -O2 -Wall $(CFLAGS_EXTRA)

# Dateinamen
TARGET_LIB = prokey$(LIB_EXT)
TARGET_TEST = test_suite$(SUFFIX)
TARGET_CPP = cpp_demo$(SUFFIX)

SRCS = prokey.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean test

all: $(TARGET_LIB) $(TARGET_TEST) $(TARGET_CPP)

# 1. Die Shared Library (DLL / SO) bauen
$(TARGET_LIB): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

# 2. Den C Test bauen (Linkt gegen die DLL/SO wenn nötig, oder statisch)
# Hier linken wir einfach direkt die C-Files für den Test, um Pfadprobleme zu vermeiden
$(TARGET_TEST): test_prokey.c prokey.c
	$(CC) $(CFLAGS) -o $@ $^

# 3. Den C++ Wrapper Demo bauen
$(TARGET_CPP): main_cpp.cpp prokey.c
	$(CXX) $(CXXFLAGS) -o $@ $^

# Objektdateien kompilieren
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(call FixPath, $(OBJS))
	$(RM) $(call FixPath, $(TARGET_LIB))
	$(RM) $(call FixPath, $(TARGET_TEST))
	$(RM) $(call FixPath, $(TARGET_CPP))
	$(RM) $(call FixPath, *.lib)
	$(RM) $(call FixPath, *.exp)
	$(RM) $(call FixPath, *.dump)
	$(RM) $(call FixPath, *.txt)

test: $(TARGET_TEST)
	./$(TARGET_TEST)
