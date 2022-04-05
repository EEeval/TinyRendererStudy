
#include "tgaimage.h"

//const TGAColor white = TGAColor(255,255,255,255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);

void line(int x0,int y0,int x1,int y1,TGAImage& image,TGAColor color)
{
	
	//�ĸ���ֵ�ľ���ֵ��Ͷ��ĸ������������Ͳ�����ַ�϶
	bool steep = false;
	if (std::abs(x0 - x1) < std::abs(y0 - y1)) { // if the line is steep, we transpose the image 
		std::swap(x0, y0);
		std::swap(x1, y1);
		steep = true;
	}
	//���ǵı����Ǵ�x0��x1�����Ҫ��֤x0����С��x1
	if (x0 > x1) { 
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	for (int x = x0; x <= x1; x++) {
		float t = (x - x0) / (float)(x1 - x0);
		int y = y1 * t - y0 * t + y0;
		if (steep) {
			image.set(y, x, color); // if transposed, detransport
		}
		else {
			image.set(x, y, color);
		}
	}
}


int main(int argc, char** argv) {
	//���߲���
	TGAImage image(1000, 1000, TGAImage::RGB);
	line(130, 200, 800, 400, image, TGAColor(255, 0, 0, 255));
	line(200, 130, 400, 800, image, green);
	
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");

	return 0;
}