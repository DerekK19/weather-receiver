CC=gcc
CFLAGS=-c `pkg-config --cflags glib-2.0` `pkg-config --cflags rest-0.7` -Wall -DRFM12B
LFLAGS=`pkg-config --libs glib-2.0` `pkg-config --libs rest-0.7`

all: ws1093_rf wh1080_rf

ws1093_rf: ws1093_rf.o fo_bcm2835.o rfm.o bmp085.o
	$(CC) $(LFLAGS) -lm ws1093_rf.o fo_bcm2835.o rfm.o bmp085.o -lbcm2835 -o ws1093_rf

ws1093_rf.o: ws1093_rf.c rfm.h rfm_commands.h
	$(CC) $(CFLAGS) ws1093_rf.c

wh1080_rf: wh1080_rf.o bcm2835.o rfm.o bmp085.o
	$(CC) $(LFLAGS) -lm wh1080_rf.o bcm2835.o rfm.o bmp085.o -o wh1080_rf

wh1080_rf.o: wh1080_rf.c wh1080_rf.h rfm01.h bcm2835.h
	$(CC) $(CFLAGS) wh1080_rf.c

rfm.o: rfm.c rfm.h rfm_commands.h
	$(CC) $(CFLAGS) rfm.c

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f *.o ws1093_rf wh1080_rf

install: all
	cp ws1093_rf /usr/local/bin
	cp receiver.sh /usr/local/bin
