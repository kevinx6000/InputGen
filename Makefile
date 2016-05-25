F = 
CC = g++
CFLAGS = -c -Wall $(F)
SRCS = main.cpp genFunction.cpp
OBJS = $(SRCS:.cpp=.o)
EXEC = main
TESTCASE = input.txt

all: $(SRCS) $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) -o $@ $<

clean:
	@rm -rf $(EXEC) $(OBJS) *~

run:
	@./$(EXEC) < $(TESTCASE)
