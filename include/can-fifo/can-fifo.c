#include "can-fifo.h"
#include <stdlib.h>
#include <string.h>

canMessage canBuffer[BUFFER_SIZE];

uint16_t head = 0;
uint16_t tail = 0;
uint16_t msgCount = 0;

// Call once before using the buffer.
void initCanBuffer(){
	memset(canBuffer, '\0', sizeof(canBuffer));
}

// Returns the next free spot in the buffer and advances the counter.
canMessage* getHeadAdv(){
	uint16_t newHead = (head+1) % BUFFER_SIZE;
#if !OVERWRITE_OLD_WHEN_FULL
	if((newHead == tail) && (msgCount >= BUFFER_SIZE)){
		return NULL; // Buffer is full.
	}
#endif
	head = newHead;
	if(msgCount < BUFFER_SIZE){
		msgCount++; // Message about to be added.
	}
	return &canBuffer[head];
}

// TODO: rename/refactoring
void releaseHead(){
	uint16_t newHead = (head + BUFFER_SIZE - 1) % BUFFER_SIZE;
	if((newHead == tail) && (msgCount == 0)){
		return; // Buffer is full.
	}
	head = newHead;
	msgCount--;
	return;
}

// Returns the oldest message in the buffer and advances the counter.
canMessage* getTailAdv(){
	if(msgCount == 0){
		return NULL; // Buffer empty.
	}
	tail = (tail+1) % BUFFER_SIZE;
	msgCount--; // Message read and can be disregarded.
	return &canBuffer[tail];
}

// Copies the contents of the passed message to the buffer. Returns NULL if fails.
canMessage* writeToBuffer(canMessage *msg){
	canMessage *bufHead = getHeadAdv();
	if(bufHead != NULL){
		memcpy(bufHead, msg, sizeof(canMessage));
	}
	return bufHead;
}

// Copies a message from the buffer to the given pointer. Returns NULL if fails.
canMessage* readFromBuffer(canMessage *msg){
	canMessage *bufTail = getHeadAdv();
	if(bufTail != NULL){
		memcpy(msg, getTailAdv(), sizeof(canMessage));
	}
	return bufTail;
}

uint16_t bufferElemCount(){
	return msgCount;
}
