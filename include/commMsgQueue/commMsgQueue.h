#ifndef __MSGQUEUE__H_
#define __MSGQUEUE__H_

#include <stdlib.h>
#include <string.h>


template <class T> class CMsgQueue
{
public:
	CMsgQueue(int iMaxsize = 2)
	{
		packetNodes = NULL;
		isize = iMaxsize;
		QueueInitQueue();
	}
	~CMsgQueue()
	{
		QueueDelQueue();
	}

public:
	int  QueueSetEmpty()	//队列置空
	{
		int iResult = 0;

		ifront = iback = 0;

		return iResult;
	}
	int  QueueAddElement(T* element)	//队列尾部添加一个元素
	{
		int iResult = 0;

		if(element == NULL)
		{
			iResult = -1;
			return iResult;
		}
		if((iResult = QueueIsFull()) == 0)//如果缓存队列满了，那么就会丢弃最后一包
		{
			ifront = (ifront + 1) % isize;
		}

		*packetNodes[iback] = *element;
		iback = (iback + 1) % isize;

		return iResult;
	}
/*
 * 成功 返回0，失败返回-1
 */
	int  QueueDelElement(T* element)	//队列删除头部的元素
	{
		int iResult = 0;

		if(element == NULL || QueueIsEmpty() == 0 )
		{
			iResult = -1;
			return iResult;
		}

		*element = *packetNodes[ifront];
		ifront = (ifront + 1) % isize;

		return iResult;
	}
	int  QueueGetFront(T* element)	//队列获取当前头元素
	{
		int iResult = 0;

		if(QueueIsEmpty() == 0 || element == NULL)
		{
			iResult = -1;
			return iResult;
		}
		else
		{
			*element = *packetNodes[ifront];
		}

		return iResult;
	}
	int QueueIsEmpty()//队列判空
	{
		int iResult = 0;

		if(ifront != iback)
		{
			iResult = -1;
		}

		return iResult;
	}
	int QueueIsFull()	//队列判断当前是否已满
	{
		int iResult =0;

		if((iback + 1) % isize != ifront)
		{
			iResult = -1;
		}

		return iResult;
	}
	int QueueDelQueue()
	{
		int iResult = 0;

		if(packetNodes)
		{
			for(int i = 0; i < isize; i++)
			{
				if(packetNodes[i] != NULL)
				{
					delete (packetNodes[i]);
				}
			}
			delete []packetNodes;
			packetNodes = NULL;
		}


		return iResult;
	}
private:
	int QueueInitQueue()
	{
		int iResult = 0;

		QueueDelQueue();


		packetNodes = new T *[isize];
		for(int i = 0; i < isize; i++)
		{
			packetNodes[i] = new T;
		}

		QueueSetEmpty();
		return iResult;
	}
private:
	int isize,ifront,iback;
	T** packetNodes;
}; 

#endif