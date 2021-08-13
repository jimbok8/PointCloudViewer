#include "CImage.h"

CImage::CImage(const int width, const int height, const int nChannels) :
	m_width(width),
	m_height(height),
	m_nChannels(nChannels) {
	m_data = new unsigned char[width * height * nChannels];
}

CImage::~CImage() {
    delete[] m_data;
}

int CImage::getWidth() const {
    return m_width;
}

int CImage::getHeight() const {
    return m_height;
}

int CImage::getNChannels() const {
    return m_nChannels;
}

const unsigned char* CImage::getData() const {
    return m_data;
}

unsigned char* CImage::getPixelPtr(const int x, const int y) const {
    return m_data + ((m_height - y - 1) * m_width + x) * m_nChannels;
}

void CImage::init(const unsigned char r, const unsigned char g, const unsigned char b) {
	if (m_nChannels < 3)
		return;

	int index = 0;
	for (int i = 0; i < m_height; i++)
		for (int j = 0; j < m_width; j++) {
			unsigned char* p = getPixelPtr(j, i);
			*p = r;
			*(p + 1) = g;
			*(p + 2) = b;
		}
}