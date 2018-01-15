/********************************************************
 *
 * file : CircularBuffer.h
 *
 *
 ****************************************************/

#ifndef _CircularBuffer_H_
#define _CircularBuffer_H_

typedef void *(*copy_func_t)(void *dest, const void *src ,size_t n);

/* Circular buffer object */
typedef struct {
    u32_t         size;   /* maximum number of elements           */
    u32_t         read_last_pos;  /* index of last read element              */
    u32_t         write_next_pos;    /* index at which to write new element  */
    void			*startOfBuffer;
    u32_t		elementSize;
    copy_func_t		fCopyOnWrite;
    copy_func_t		fCopyOnRead;
} CircularBuffer_t;

void cbInit(CircularBuffer_t *cb, u32_t size, void *startOfBuffer ,
		u32_t elementSize,copy_func_t	fCopyOnWrite ,copy_func_t	fCopyOnRead) ;
u32_t cbWrite(CircularBuffer_t *cb, void *buffer , u32_t max_num_to_write );
u32_t cbRead(CircularBuffer_t *cb, void *buffer , u32_t max_num_to_read );
u32_t cbGetNextReadPos(CircularBuffer_t *cb);
u32_t cbGetNumOfElements(CircularBuffer_t *cb);


#else
#pragma message( "warning : this header file had already been included" )
#endif
