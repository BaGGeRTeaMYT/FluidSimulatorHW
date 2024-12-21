#ifndef FieldHEADER
#define FieldHEADER

#include <vector>

class Field {
    public:
    constexpr Field(size_t width, size_t height, std::vector<std::vector<char>>& field);

    private:
    size_t m_width;
    size_t m_height;
    std::vector<std::vector<char>> m_field;
};

#endif FieldHEADER