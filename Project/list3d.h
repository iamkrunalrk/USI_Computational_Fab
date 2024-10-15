#include <vector>
#include <stdio.h>

template <typename T>
class List3d {
public:
    List3d(size_t d1 = 0, size_t d2 = 0, size_t d3 = 0)
        : d1(d1), d2(d2), d3(d3), data(d1 * d2 * d3) {}

    T& operator()(size_t i, size_t j, size_t k) {
        return data[i * d2 * d3 + j * d3 + k];
    }

    const T& operator()(size_t i, size_t j, size_t k) const {
        return data[i * d2 * d3 + j * d3 + k];
    }

    void resize(size_t d1, size_t d2, size_t d3) {
        this->d1 = d1;
        this->d2 = d2;
        this->d3 = d3;
        data.resize(d1 * d2 * d3);
    }

private:
    size_t d1, d2, d3;         // dimensions of 3D array
    std::vector<T> data;       // 1D vector to store 3D array
};
