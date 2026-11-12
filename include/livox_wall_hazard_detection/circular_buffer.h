#include <vector>
#include <optional>
#include <mutex>

template <typename T, size_t Capacity>
class CircularBuffer {
    std::vector<T> buffer;
    size_t head = 0;
    size_t tail = 0;
    size_t sz = 0;
    mutable std::mutex mtx;

public:
    CircularBuffer() {
        buffer.resize(Capacity);
    }

    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mtx);
        buffer[tail] = item;
        tail = (tail + 1) % Capacity;
        if (sz < Capacity) {
            sz++;
        } else {
            head = (head + 1) % Capacity;
        }
    }

    std::optional<T> pop() {
        std::lock_guard<std::mutex> lock(mtx);
        if (sz == 0) return std::nullopt;
        T item = buffer[head];
        head = (head + 1) % Capacity;
        sz--;
        return item;
    }

    // Thread-protected copy into an external vector of the same capacity
    void copy_to(std::vector<T>& dest) const {
        std::lock_guard<std::mutex> lock(mtx);
        for (size_t i = 0; i < sz; ++i) {
            dest[i] = buffer[(head + i) % Capacity];
        }
    }

    std::vector<T> get_buffer() const {
        std::lock_guard<std::mutex> lock(mtx);
        std::vector<T> snapshot(sz); //TODO should we already define a vector and resize here for no heap allocation
        for (size_t i = 0; i < sz; ++i) {
            snapshot[i] = buffer[(head + i) % Capacity];
        }
        return snapshot; // Moved efficiently via NRVO / move semantics
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx);
        return sz;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mtx);
        return sz == 0;
    }
};
