#ifndef IMAGE_H
#define IMAGE_H

class CImage {
private:
    int m_width, m_height, m_nChannels;
    unsigned char* m_data;

public:
    CImage(const int width, const int height, const int nChannels);
    ~CImage();
    int getWidth() const;
    int getHeight() const;
    int getNChannels() const;
    const unsigned char* getData() const;
    unsigned char* getPixelPtr(const int x, const int y) const;
    void init(const unsigned char r, const unsigned char g, const unsigned char b);
};

#endif