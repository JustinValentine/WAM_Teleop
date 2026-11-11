#pragma once

/* The teleop's own recording: one row per control cycle, written by the
 * control loop, without ROS.
 *
 * The ROS recording (background_state_publisher.h + rosbag) samples the arm
 * from a normal-priority thread: three separate locked reads of position,
 * velocity and torque, 500 times a second, each of which can hold the
 * execution manager's lock while the control loop needs it, and none of which
 * is tied to a control cycle (1-2 % of the bags' samples are late, doubled or
 * missing). It also records none of the controller's own signals.
 *
 * Here the loop copies one fixed row of doubles into a ring buffer (no lock,
 * no allocation, no system call); a normal-priority thread writes the ring to
 * a file. If the disk stalls until the ring is full, rows are dropped and
 * counted, and the loop is never delayed. The row holds exactly what the
 * controller used that cycle, so the file is the controller's view.
 *
 * File: three text lines, then little-endian float64 rows.
 *   TELEOPLOG 1 <columns>
 *   <column names, comma separated>
 *   <free text: program, settings, start time>
 * scripts/teleop_log.py reads it (summary, CSV, NPZ). */

#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class TeleopRecorder {
public:
	/* capacity: rows the ring holds (a power of two); 32768 is 65 s at 500 Hz. */
	TeleopRecorder(const std::string& path, const std::vector<std::string>& columns, const std::string& note,
	               size_t capacity = 32768)
		: path(path), width(columns.size()), capacity(capacity), ring(capacity * columns.size(), 0.0),
		  head(0), tail(0), droppedRows(0), writtenRows(0), failed(false), stopping(false), file(NULL) {
		if (width == 0 || capacity == 0 || (capacity & (capacity - 1)) != 0) {
			throw std::invalid_argument("TeleopRecorder: needs columns and a power-of-two capacity");
		}
		file = std::fopen(path.c_str(), "wb");
		if (file == NULL) {
			throw std::runtime_error("cannot write " + path + ": " + std::strerror(errno));
		}
		std::string names;
		for (size_t i = 0; i < columns.size(); ++i) {
			names += (i ? "," : "") + columns[i];
		}
		std::string oneLine(note);
		for (size_t i = 0; i < oneLine.size(); ++i) {
			if (oneLine[i] == '\n') oneLine[i] = ' ';
		}
		std::fprintf(file, "TELEOPLOG 1 %zu\n%s\n%s\n", width, names.c_str(), oneLine.c_str());
		std::fflush(file);
		writer = std::thread(&TeleopRecorder::run, this);
	}

	~TeleopRecorder() { close(); }

	/* Control loop: the next row to fill (columns() doubles), or NULL when the
	 * ring is full (the row is dropped and counted). Then commit(). */
	double* claim() {
		const size_t h = head.load(std::memory_order_relaxed);
		if (h - tail.load(std::memory_order_acquire) >= capacity) {
			droppedRows.fetch_add(1, std::memory_order_relaxed);
			return NULL;
		}
		return &ring[(h & (capacity - 1)) * width];
	}
	void commit() { head.store(head.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

	size_t columns() const { return width; }
	unsigned long dropped() const { return droppedRows.load(std::memory_order_relaxed); }
	unsigned long written() const { return writtenRows.load(std::memory_order_relaxed); }
	/* The disk refused a write (full, removed): rows are no longer saved. */
	bool writeFailed() const { return failed.load(std::memory_order_relaxed); }
	const std::string& getPath() const { return path; }

	/* Writes what is left and closes the file. */
	void close() {
		if (file == NULL) {
			return;
		}
		stopping = true;
		if (writer.joinable()) {
			writer.join();
		}
		std::fclose(file);
		file = NULL;
	}

private:
	void run() {
		int idle = 0;
		for (;;) {
			const size_t t = tail.load(std::memory_order_relaxed);
			const size_t h = head.load(std::memory_order_acquire);
			if (t == h) {
				if (stopping) {
					break;
				}
				if (++idle >= 100) {  // about half a second: a crash loses no more
					std::fflush(file);
					idle = 0;
				}
				usleep(5000);
				continue;
			}
			// the rows up to the end of the ring, in one write
			const size_t first = t & (capacity - 1);
			size_t rows = h - t;
			if (rows > capacity - first) {
				rows = capacity - first;
			}
			if (!failed) {
				if (std::fwrite(&ring[first * width], sizeof(double) * width, rows, file) != rows) {
					failed = true;  // keep draining so the loop's rows are counted, not blocked
				} else {
					writtenRows.fetch_add(rows, std::memory_order_relaxed);
				}
			}
			tail.store(t + rows, std::memory_order_release);
		}
		std::fflush(file);
	}

	const std::string path;
	const size_t width, capacity;
	std::vector<double> ring;
	std::atomic<size_t> head, tail;
	std::atomic<unsigned long> droppedRows, writtenRows;
	std::atomic<bool> failed, stopping;
	FILE* file;
	std::thread writer;

	TeleopRecorder(const TeleopRecorder&);
	TeleopRecorder& operator=(const TeleopRecorder&);
};

/* Fills one row in order; the arm systems build their rows with it so that
 * the column list and the values cannot drift apart silently. */
class RecorderRow {
public:
	RecorderRow(double* row, size_t width) : row(row), width(width), n(0) {}
	void put(double v) {
		if (n < width) row[n] = v;
		++n;
	}
	template <typename V>
	void put(const V& v, int count) {
		for (int i = 0; i < count; ++i) put(v[i]);
	}
	bool complete() const { return n == width; }

private:
	double* row;
	size_t width, n;
};
