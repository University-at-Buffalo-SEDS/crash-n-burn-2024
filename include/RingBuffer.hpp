#pragma once

#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>

inline static size_t wrapping_add(size_t val, size_t increment, size_t max) {
	return (val + increment) % max;
}

template <typename T, unsigned int Cap>
class RingBuffer {
	size_t head, tail, size;
	T buf[Cap];

public:
	RingBuffer() : head(0), tail(0), size(0), buf() {}

	size_t available() { return Cap - size; }
	size_t used() { return size; }

	// Adds a single element to the end of the ring buffer.
	// Returns whether there was enough space to add the
	// new element without overwriting old elements.
	[[nodiscard]] bool push(const T &val, bool overwrite);

	// Pops a single element from the ring buffer and sets val to its value.
	// Returns whether there was a value to pop.
	[[nodiscard]] bool pop(T *val);

	// Pushes an array of data into the ring buffer.
	// If overwrite is false and there is not enough space to push
	// all of the items, this will return false and do nothing.
	// Otherwise, returns whether there was enough space to add the
	// items without overwriting old data.
	[[nodiscard]] bool push(const T *data, size_t count, bool overwrite);

	// Pops an array of data from the ring buffer.
	// If the array does not have count items to pop this
	// immediately returns false and does nothing.
	// Otherwise, this copies count elements into data and returns true.
	[[nodiscard]] bool pop(T *data, size_t count);
};

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::push(const T &val, bool overwrite)
{
	if (size == Cap && !overwrite) {
		return false;
	}

	buf[tail] = val;

	tail = wrapping_add(tail, 1, Cap);

	if (size < Cap) {
		++size;
		return true;
	}

	head = wrapping_add(head, 1, Cap);
	return false;
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::pop(T *val) {
	if (size == 0) {
		return false;
	}
	*val = buf[head];
	--size;
	head = wrapping_add(head, 1, Cap);
	return true;
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::push(const T *data, size_t count, bool overwrite) {
	if (size + count > Cap && !overwrite) {
		return false;
	}
	if (count > Cap) {
		data += count - Cap;
		count = Cap;
	}
	if (tail + count > Cap) {
		// Range wraps around, copy the start of the data from
		// tail to the limit of the buffer, then copy the rest
		// after the start of the buffer.
		size_t count_to_tail = Cap - tail;
		size_t remainder = count - count_to_tail;
		memcpy(buf + tail, data, sizeof(T) * count_to_tail);
		memcpy(buf, data + count_to_tail, sizeof(T) * remainder);
	} else {
		// We have enough space after the tail before the limit of the buffer.
		// Simply copy the data to the tail.
		memcpy(buf + tail, data, sizeof(T) * count);
	}
	tail = wrapping_add(tail, count, Cap);
	if (size + count > Cap) {
		// We've advanced past the head, set it to the tail
		head = tail;
	}
	size += count;
	if (size > Cap) {
		size = Cap;
		return false;
	}
	return true;
}

template <typename T, unsigned int Cap>
bool RingBuffer<T, Cap>::pop(T *data, size_t count) {
	if (size < count) {
		return false;
	}
	if (head + count > Cap) {
		size_t count_from_head = Cap - head;
		size_t remainder = count - count_from_head;
		memcpy(data, buf + head, sizeof(T) * count_from_head);
		memcpy(data + count_from_head, buf, sizeof(T) * remainder);
	} else {
		memcpy(data, buf + head, sizeof(T) * count);
	}
	size -= count;
	head = wrapping_add(head, count, Cap);
	return true;
}