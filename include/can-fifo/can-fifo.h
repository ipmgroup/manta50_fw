#ifndef INCLUDE_CAN_FIFO_CAN_FIFO_H_
#define INCLUDE_CAN_FIFO_CAN_FIFO_H_

#ifndef uint16_t
#define uint16_t unsigned int
#endif
#ifndef uint8_t
#define uint8_t unsigned char
#endif
#ifndef NULL
#define NULL (void*)0
#endif

#define BUFFER_SIZE 5
#define OVERWRITE_OLD_WHEN_FULL 0

typedef struct canMessage{
	uint16_t id;
	uint8_t msg[8];
	uint8_t len;
	uint8_t valid;
}canMessage;

void initCanBuffer();
canMessage* getHeadAdv();
void releaseHead();
canMessage* getTailAdv();
canMessage* writeToBuffer(canMessage *msg);
canMessage* readFromBuffer(canMessage *msg);
uint16_t bufferElemCount();

#endif /* INCLUDE_CAN_FIFO_CAN_FIFO_H_ */
