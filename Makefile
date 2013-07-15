CC=gcc
CFLAGS=-c `pkg-config --cflags glib-2.0` `pkg-config --cflags rest-0.7` -Wall -DRFM12B
LFLAGS=`pkg-config --libs glib-2.0` `pkg-config --libs rest-0.7`

all: ws1093_rf

ws1093_rf: ws1093_rf.o bcm2835.o rfm.o bmp085.o
	$(CC) $(LFLAGS) -lm ws1093_rf.o bcm2835.o rfm.o bmp085.o -o ws1093_rf

ws1093_rf.o: ws1093_rf.c rfm.h rfm_commands.h
	$(CC) $(CFLAGS) ws1093_rf.c

bcm2835.o: bcm2835.c
	$(CC) $(CFLAGS) bcm2835.c

rfm.o: rfm.c rfm.h rfm_commands.h
	$(CC) $(CFLAGS) rfm.c

bmp085.o: bmp085.c
	$(CC) $(CFLAGS) bmp085.c

clean:
	rm -f ws1093_rf.o bcm2835.o ws_bcm2835.o rfm.o bmp085.o ws1093_rf

install: all
	cp ws1093_rf /usr/local/bin
	cp receiver.sh /usr/local/bin
