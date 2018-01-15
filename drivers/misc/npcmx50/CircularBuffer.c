/* CircularBuffer_t.c */
#if 0
	#include <linux/types.h> /* size_t */
	//#include <mach/hal.h>
	#define  PRINTF printk



#include "CircularBuffer.h"

void cbInit(CircularBuffer_t *cb, u32_t size, void *startOfBuffer ,
		u32_t elementSize,copy_func_t	fCopyOnWrite ,copy_func_t	fCopyOnRead)
{
    cb->size  = size; /* include empty elem */
    cb->write_next_pos = 1;
    cb->read_last_pos  = 0;
    cb->elementSize=elementSize;
    cb->startOfBuffer=startOfBuffer;
    cb->fCopyOnRead=fCopyOnRead;
    cb->fCopyOnWrite=fCopyOnWrite;
}


u32_t cbIsFull(CircularBuffer_t *cb)
{
    return cb->write_next_pos == cb->read_last_pos;
}

u32_t cbIsEmpty(CircularBuffer_t *cb)
{
    return  cb->write_next_pos == ( (cb->read_last_pos + 1 ) % cb->size );
}


u32_t cbGetNextReadPos(CircularBuffer_t *cb)
{
    return  ( (cb->read_last_pos + 1 ) % cb->size );
}


u32_t cbGetNumOfElements(CircularBuffer_t *cb)
{
	u32_t numOfElements;
	if(cb->write_next_pos > cb->read_last_pos)
	{
		numOfElements = cb->write_next_pos - cb->read_last_pos -1;
	}
	else
	{
        numOfElements = (cb->size + cb->write_next_pos) - cb->read_last_pos -1;
	}
	return  numOfElements;
}

/* Write an elements */
u32_t cbWrite(CircularBuffer_t *cb, void *buffer, u32_t max_num_to_write )
{
	u32_t num_to_write =0 ;
	u32_t num_was_written =0 ;
	u32_t read_last_pos;

	read_last_pos=cb->read_last_pos;
	while (max_num_to_write && (read_last_pos != cb->write_next_pos) ) // second test is for fullness of buffer
	{

		if(cb->write_next_pos > read_last_pos)
		{
			num_to_write = cb->size - cb->write_next_pos ;
		}
		else
		{
			num_to_write = read_last_pos - cb->write_next_pos ;
		}


		if(num_to_write > max_num_to_write)
		{
			num_to_write = max_num_to_write;
		}

		cb->fCopyOnWrite(((u8_t*)cb->startOfBuffer) + (cb->write_next_pos * cb->elementSize),
				 buffer, num_to_write * cb->elementSize);
		buffer = ((u8_t*)buffer) + num_to_write * cb->elementSize;
		max_num_to_write -= num_to_write;
		num_was_written += num_to_write;
		cb->write_next_pos = (cb->write_next_pos + num_to_write ) % cb->size;
	}
    return num_was_written;
}

/* Read oldest element. */
u32_t cbRead(CircularBuffer_t *cb, void *buffer, u32_t max_num_to_read  )
{
	u32_t num_to_read = 0 ;
	u32_t startOfReadPos ;
	u32_t num_was_read =0 ;
	u32_t write_next_pos;

	write_next_pos = cb->write_next_pos;
	startOfReadPos=((cb->read_last_pos+1) % cb->size);
	while (max_num_to_read && (write_next_pos != startOfReadPos) ) // second test is for emptiness of buffer
	{

		if(write_next_pos > startOfReadPos)
		{
			num_to_read = ( write_next_pos - startOfReadPos ) ;
		}
		else
		{
			num_to_read = (cb->size - startOfReadPos ) ;
		}


		if(max_num_to_read < num_to_read)
		{
			num_to_read = max_num_to_read;
		}

		cb->fCopyOnRead(buffer, ((u8_t*)cb->startOfBuffer) + (startOfReadPos * cb->elementSize),
				num_to_read * cb->elementSize);
		buffer = ((u8_t*)buffer) + num_to_read * cb->elementSize;
		max_num_to_read -= num_to_read;
		num_was_read += num_to_read;
		cb->read_last_pos = startOfReadPos + num_to_read - 1;

		startOfReadPos = ((cb->read_last_pos+1) % cb->size);

	}

    return num_was_read;
}

#endif
