CC=gcc
RM=rm -vf

EDCFLAGS:= -O2 -Wall -DUNIT_TEST_INA219 -I include/ -I ./ -I drivers/ $(CFLAGS)
EDLDFLAGS:= -lpthread -lm $(LDFLAGS)

EDCFLAGS+=

TARGET=ina219_test.out

COBJS=ina219.o \
	drivers/i2cbus/i2cbus.o

all: $(TARGET)

$(TARGET): $(COBJS)
	$(CC) $(EDCFLAGS) -o $@ $(COBJS) $(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	$(RM) $(TARGET)
	$(RM) $(COBJS)

