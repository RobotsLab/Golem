/** @file Stream.h
 * 
 * Interface for data serialisation.
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_SYS_STREAM_H_
#define _GOLEM_SYS_STREAM_H_

#include <fstream>
#include <list>
#include <Golem/Sys/Message.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgStream, Message)
MESSAGE_DEF(MsgStreamRead, MsgStream)
MESSAGE_DEF(MsgStreamWrite, MsgStream)
MESSAGE_DEF(MsgStreamFileOpenFail, MsgStream)
MESSAGE_DEF(MsgStreamDirCreateFail, MsgStream)

//------------------------------------------------------------------------------

/** Interface for data serialisation.
*/
class Stream {
public:
	/** Flags */
	enum Flags {
		/** Default */
		FLAG_DEFAULT = 0,
		/** 32 bit pointer */
		FLAG_PTR32 = (1<<0),
		/** 64 bit pointer */
		FLAG_PTR64 = (1<<1),
	};

	/** Handler */
	class Handler {
	public:
		/** Destructor */
		virtual ~Handler() {}
	};

	/** Stream position */
	typedef intptr_t Position;

	/** Constructor */
	Stream(U32 flags = FLAG_DEFAULT, Handler* handler = NULL);
	/** Destructor */
	virtual ~Stream();

	/** User data */
	U32 getFlags() const {
		return flags;
	}
	/** User data */
	void setFlags(U32 flags) const {
		this->flags = flags;
	}
	
	/** User handler */
	Handler* getHandler() const {
		return handler;
	}
	/** User handler */
	void setHandler(Handler* handler) const {
		this->handler = handler;
	}

	/** Resets the input stream */
	virtual Position getRead() const;
	/** Resets the input stream */
	virtual void resetRead(Position pos = 0) const;

	/** Resets the input stream */
	virtual Position getWrite() const;
	/** Resets the output stream */
	virtual void resetWrite(Position pos = 0);

	/** Returns last read/written number of bytes */
	virtual size_t lastBytes() const;

	/** Loads data of type Type from the input stream  */
	template <typename Type> void read(Type& data) const {
		readBytes(&data, sizeof(Type));
	}
	/** Loads data of type Type from the input stream  */
	template <typename Type> void read(Type* data, size_t length) const {
		readBytes(data, sizeof(Type)*length);
	}
	/** Stores data of type Type to the output stream  */
	template <typename Type> void write(const Type& data) {
		writeBytes(&data, sizeof(Type));
	}
	/** Stores data of type Type to the output stream  */
	template <typename Type> void write(const Type* data, size_t length) {
		writeBytes(data, sizeof(Type)*length);
	}
	/** Loads STL collection from the stream */
	template <typename Stl, typename Iter> void read(Stl& stl, Iter iter, const typename Stl::value_type& dflt = typename Stl::value_type()) const {
		const U32 flags = getFlags();
		size_t size = 0;
		if (flags == FLAG_DEFAULT)
			read(size);
		else if (flags & FLAG_PTR32) {
			U32 size32 = 0;
			read(size32);
			size = (size_t)size32;
		}
		else if (flags & FLAG_PTR64) {
			U64 size64 = 0;
			read(size64);
			size = (size_t)size64;
		}

		for (size_t i = 0; i < size; ++i) {
			typename Stl::value_type val = dflt;
			read(val);
			iter = ++stl.insert(iter, val);
		}
	}
	/** Stores STL collection in the stream  */
	template <typename Iter> void write(Iter begin, Iter end) {
		const U32 flags = getFlags();
		const size_t size = std::distance(begin, end);
		if (flags == FLAG_DEFAULT)
			write(size);
		else if (flags & FLAG_PTR32)
			write(U32(size));
		else if (flags & FLAG_PTR64)
			write(U64(size));

		while (begin != end)
			write(*begin++);
	}

	/** Input stream operator */
	template <typename Type> friend const Stream& operator >> (const Stream& stream, Type& data);
	/** Output stream operator */
	template <typename Type> friend Stream& operator << (Stream& stream, const Type& data);

protected:
	/** Stream flags */
	mutable U32 flags;
	/** Stream handler */
	mutable Handler* handler;

	/** Read size bytes from the input stream */
	virtual void readBytes(void *dst, size_t size) const;
	/** Write size bytes to the output stream */
	virtual void writeBytes(const void *src, size_t size);
};

/** Input stream operator */
template <typename Type> const Stream& operator >> (const Stream& stream, Type& data) {
	stream.read(data);
	return stream;
}

/** Output stream operator */
template <typename Type> Stream& operator << (Stream& stream, const Type& data) {
	stream.write(data);
	return stream;
}

//------------------------------------------------------------------------------
// String support.

/** Loads data of type std::string from the stream  */
template <> void Stream::read(std::string* str, size_t length) const;

/** Loads data of type std::string from the stream  */
template <> void Stream::read(std::string& str) const;

/** Stores data of type std::string to the stream  */
template <> void Stream::write(const std::string* str, size_t length);

/** Stores data of type std::string to the stream  */
template <> void Stream::write(const std::string& str);

//------------------------------------------------------------------------------

/** Serialization interface */
class Serializable {
public:
	/** Virtual destructor */
	virtual ~Serializable() {}
	
	/** Loads data from the specified stream */
	virtual void load(const Stream &stream) = 0;
	
	/** Stores data to the specified stream */
	virtual void store(Stream &stream) const = 0;
};

/** Input stream operator */
template <> const Stream& operator >> (const Stream& stream, Serializable& serializable);

/** Output stream operator */
template <> Stream& operator << (Stream& stream, const Serializable& serializable);

//------------------------------------------------------------------------------

/** Memory stream.
*/
class MemoryStream : public Stream {
public:
	typedef void* Pointer;
	typedef std::list<Pointer> Buffer;

public:
	/** Returns buffer */
	virtual const Buffer &get() const = 0;
};


/** Memory write stream.
*/
class MemoryWriteStream : public MemoryStream {
protected:
	Buffer buffer;

	virtual void writeBytes(const void *src, size_t size);

public:
	/** Destructor */
	virtual ~MemoryWriteStream();
	
	/** Returns buffer */
	virtual const Buffer &get() const {
		return buffer;
	}

	/** Resets the stream */
	virtual void resetWrite();
};


/** Memory read stream.
*/
class MemoryReadStream : public MemoryStream {
protected:
	const Buffer &buffer;
	mutable Buffer::const_iterator ptr;

	virtual void readBytes(void *dst, size_t size) const;

public:
	/** Constructor */
	MemoryReadStream(const Buffer &buffer);
	
	/** Returns buffer */
	virtual const Buffer &get() const {
		return buffer;
	}

	/** Resets the stream */
	virtual void resetRead() const;
};

//------------------------------------------------------------------------------

/** Creates a hierarchy of directories with names separated by / or \\.
* If the hierarchy does not end with / or \\ the last directory is skipped.
*/
void mkdir(const char *dir);

//------------------------------------------------------------------------------

/** File stream.
*/
class FileStream : public Stream {
protected:
	mutable std::fstream filestream;
	const char* const filePath;

	virtual void readBytes(void *dst, size_t size) const;
	virtual void writeBytes(const void *src, size_t size);

public:
	/** Constructor */
	FileStream(const char *filePath, std::ios::openmode mode = std::ios::in | std::ios::out | std::ios::binary);

	/** Destructor */
	virtual ~FileStream();
	
	/** Resets the input stream */
	virtual Position getRead() const;
	/** Resets the stream */
	virtual void resetRead(Position pos = 0) const;
	
	/** Resets the input stream */
	virtual Position getWrite() const;
	/** Resets the stream */
	virtual void resetWrite(Position pos = 0);

	/** Returns last read/written number of items */
	virtual size_t lastBytes() const;

	/** Returns file stream */
	virtual const std::fstream &get() const {
		return filestream;
	}
	virtual std::fstream &get() {
		return filestream;
	}
};


/** File write stream.
*/
class FileWriteStream : public FileStream {
protected:
	std::ofstream filestream;

	virtual void readBytes(void *dst, size_t size) const;

public:
	/** Constructor */
	FileWriteStream(const char *filePath);
};


/** File read stream.
*/
class FileReadStream : public FileStream {
protected:
	virtual void writeBytes(const void *src, size_t size);

public:
	/** Constructor */
	FileReadStream(const char *filePath);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_STREAM_H_*/
