#include <vector>
#include <cmath>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
Model* model = NULL;
Vec3f light_dir(0, 0, -1);
const int width = 400;
const int height = 400;

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

//Line Sweeping ɨ�߷����������
void triangle_LineSweeping(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage& image, TGAColor color) {
	// sort the vertices, t0, t1, t2 lower_to_upper (bubble sort)
	if (t0.y > t1.y) std::swap(t0, t1);
	if (t0.y > t2.y) std::swap(t0, t2);
	if (t1.y > t2.y) std::swap(t1, t2);
	int total_height = t2.y - t0.y;
	for (int y = t0.y; y <= t1.y; y++) {
		int segment_height = t1.y - t0.y + 1;
		float alpha = (float)(y - t0.y) / total_height;//�߶ε�����ʽ
		float beta = (float)(y - t0.y) / segment_height; // be careful with divisions by zero 
		Vec2i A = t0 + (t2 - t0) * alpha;
		Vec2i B = t0 + (t1 - t0) * beta;
		if (A.x > B.x) std::swap(A, B);
		for (int j = A.x; j <= B.x; j++) {
			image.set(j, y, color); // attention, due to int casts t0.y+i != A.y 
		}
	}
	for (int y = t1.y; y <= t2.y; y++) {
		int segment_height = t2.y - t1.y + 1;
		float alpha = (float)(y - t0.y) / total_height;
		float beta = (float)(y - t1.y) / segment_height; // be careful with divisions by zero 
		Vec2i A = t0 + (t2 - t0) * alpha;
		Vec2i B = t1 + (t2 - t1) * beta;
		if (A.x > B.x) std::swap(A, B);
		for (int j = A.x; j <= B.x; j++) {
			image.set(j, y, color); // attention, due to int casts t0.y+i != A.y 
		}
	}
}

// �������귨��������� barycentric coordinates
//triangle(vec2 points[3]) {
//	vec2 bbox[2] = find_bounding_box(points);
//	for (each pixel in the bounding box) {
//		if (inside(points, pixel)) {
//			put_pixel(pixel);
//		}
//	}
//ʹ�������귨�������������ֵ
Vec3f barycentric(Vec3f& A, Vec3f& B, Vec3f& C, Vec3f P) {
	float dx_pb = P.x - B.x;
	float dy_pb = P.y - B.y;
	float dx_cb = C.x - B.x;
	float dy_cb = C.y - B.y;
	float dx_ab = A.x - B.x;
	float dy_ab = A.y - B.y;
	float dx_pc = P.x - C.x;
	float dy_pc = P.y - C.y;
	float dx_ac = A.x - C.x;
	float dy_ac = A.y - C.y;
	float alpha = (-dx_pb * dy_cb + dy_pb * dx_cb) / (-dx_ab * dy_cb + dy_ab * dx_cb);
	float beta = (-dx_pc * dy_ac + dy_pc * dx_ac) / (dx_cb * dy_ac + (-dy_cb * dx_ac));
	float gamma = 1 - alpha - beta;
	return Vec3f(alpha, beta, gamma);
}

//void triangle(Vec3f* pts,Vec3f* uvs, float* zbuffer, TGAImage& image, TGAColor color,float intensity) {
//	Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
//	Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
//	Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
//	//ȷ��boundingbox��Χ
//	for (int i = 0; i < 3; i++) {
//		bboxmin.x = std::max(0.f, std::min(bboxmin.x, pts[i].x));
//		bboxmin.y = std::max(0.f, std::min(bboxmin.y, pts[i].y));
//
//		bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, pts[i].x));
//		bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, pts[i].y));
//	}
//	Vec3f P;
//	Vec3f uvP;
//	
// //�������������ֵ
//	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
//		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
//			Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
//			Vec3f uv_screen = barycentric(uvs[0], uvs[1], uvs[2], P);
//			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
//			P.z = 0;
//			uvP.x = 0;
//			uvP.y = 0;
//			for (int i = 0; i < 3; i++) 
//			{
//				P.z += pts[i][2] * bc_screen[i];//���������ֵ�����ÿһ�������Ӧ�ĸ�����Ļ���ص�zֵ
//				uvP.x += uvs[i].x * uv_screen[i];
//				uvP.y += uvs[i].y * uv_screen[i];
//			} 
//			//��ģ��zֵ��zbuffer�е�ֵ���Ƚϣ���ʾ��ý��ģ�����-z��������ý���zֵ��
//			int idx = P.x + P.y * width;
//			if (zbuffer[idx] < P.z)
//			{
//				zbuffer[idx] = P.z;
//				TGAColor color = model->diffuse(uvP);
//				image.set(P.x,P.y,TGAColor(color.r*intensity,color.g*intensity,color.b*intensity,255));
//			}
//			
//		}
//		
//	}
//} 
void triangle(Vec3i t0, Vec3i t1, Vec3i t2, Vec2i uv0, Vec2i uv1, Vec2i uv2, TGAImage& image, float intensity, float* zbuffer) {
	if (t0.y == t1.y && t0.y == t2.y) return;
	//�ָ������������
	if (t0.y > t1.y) { std::swap(t0, t1); std::swap(uv0, uv1); }
	if (t0.y > t2.y) { std::swap(t0, t2); std::swap(uv0, uv2); }
	if (t1.y > t2.y) { std::swap(t1, t2); std::swap(uv1, uv2); }
	//�ø߶���ѭ������
	int total_height = t2.y - t0.y;
	for (int i = 0; i < total_height; i++) {
		//�ж�������һ������ȷ���߶�
		bool second_half = i > t1.y - t0.y || t1.y == t0.y;
		int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
		//���㵱ǰ�ı���
		float alpha = (float)i / total_height;
		float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height; // be careful: with above conditions no division by zero here
		//A��ʾt0��t2֮��ĵ�
		//B��ʾt0��t1֮��ĵ�
		Vec3i A = t0 + Vec3f(t2 - t0) * alpha;
		Vec3i B = second_half ? t1 + Vec3f(t2 - t1) * beta : t0 + Vec3f(t1 - t0) * beta;
		//����UV
		Vec2i uvA = uv0 + (uv2 - uv0) * alpha;
		Vec2i uvB = second_half ? uv1 + (uv2 - uv1) * beta : uv0 + (uv1 - uv0) * beta;
		//��֤B��A���ұ�
		if (A.x > B.x) { std::swap(A, B); }// std::swap(uvA, uvB);}
		//�ú�������Ϊѭ�����ƣ�����һ�н�����ɫ
		for (int j = A.x; j <= B.x; j++) {
			//���㵱ǰ����AB֮��ı���
			float phi = B.x == A.x ? 1. : (float)(j - A.x) / (float)(B.x - A.x);
			//�������ǰ�������,A��B������z����Ϣ
			Vec3i   P = Vec3f(A) + Vec3f(B - A) * phi;
			Vec2i uvP = uvA + (uvB - uvA) * phi;
			if (P.x < width && P.y < height)
			{
				//���㵱ǰzbuffer�±�=P.x+P.y*width
				int idx = P.x + P.y * width;
				//��ǰ���z����zbuffer��Ϣ�����ǵ���������zbuffer
				if (zbuffer[idx] < P.z) {
					zbuffer[idx] = P.z;
					TGAColor color = model->diffuse(uvP);
					image.set(P.x, P.y, TGAColor(color.r * intensity, color.g * intensity, color.b * intensity,255));
				}
			}
		}
	}
}
struct Point {
	int x;
	int y;
};
static bool PointIsInTriangle(const Point* tri, const Point target)
{
	TGAImage image(width, height, TGAImage::RGB);

	bool bResult = false;

	image.set(target.x,target.y, TGAColor(255, 0, 0, 255));

	for (int32_t i = 0; i < 3; ++i)
	{
		const Point& point0 = tri[i];       //0 1 2
		const Point& point1 = tri[(i + 1) % 3]; //1 2 0
		line(point0.x,point0.y, point1.x,point1.y, image, TGAColor(255, 0, 0, 255));
		//���˳��ڸ߶��Ͽ����������ཻ�ı�
		if ((point0.y < target.y && target.y <= point1.y) || (point1.y < target.y && target.y <= point0.y))
		{
			float slop = (point1.x - point0.x) / (point1.y - point0.y);
			//��point0��point1�������߶εĵ�бʽ����
			float x = slop * (target.y - point0.y) + point0.x;
			//�жϴ�Ŀ������ҷ��������Ƿ�����������н���
			if (target.x < x)
			{
				bResult = !bResult;
			}
		}
	}
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");

	return bResult;
}


int main(int argc, char** argv) {
	
	Point p0;
	p0.x = 100;
	p0.y = 100;
	Point p1;
	p1.x = 300;
	p1.y = 100;
	Point p2;
	p2.x = 200;
	p2.y = 200;
	std::vector<Point> points;
	points.push_back(p0);
	points.push_back(p1);
	points.push_back(p2);
	Point target;
	target.x = 220;
	target.y = 220;
	if (PointIsInTriangle(&points[0], target))
	{
		std::cout << "point is in triangle!" << std::endl;
	}
	else
	{
		std::cout << "point is out of the triangle!" << std::endl;
	}
	//PointIsInTriangle(&points[0],target);
	////��ȡģ���ļ�
	//if (2 == argc) {
	//	model = new Model(argv[1]);
	//}
	//else {
	//	model = new Model("obj/african_head.obj");
	//}
	////�û�����С��һά����洢zbuffer
	//float* zbuffer = new float[width * height];
	//for (int i = 0; i < width * height; i++) 
	//{
	//	zbuffer[i] = -std::numeric_limits<float>::max();//����-z������ֵԽС���ԽԶ
	//}
	//	
	//TGAImage image(width, height, TGAImage::RGB);
	////����ÿ��������
	//for (int i = 0; i < model->nfaces(); i++) {
	//	std::vector<int> face = model->face(i);
	//	Vec3f screen_coords[3];
	//	Vec3f world_coords[3];
	//	for (int j = 0; j < 3; j++) {
	//		Vec3f v = model->vert(face[j]);
	//		screen_coords[j] = Vec3f(int((v.x + 1.) * width / 2.+.5), int((v.y + 1.) * height / 2.+.5),v.z);
	//		world_coords[j] = v;
	//	}
	//	//������Ƭ������
	//	Vec3f n = (world_coords[2] - world_coords[0])^(world_coords[1] - world_coords[0]);
	//	n.normalize();
	//	//������ ���ո�����*���߷���*cos�� ����ʾ����ǿ��
	//	float intensity = n * light_dir;
	//	//�����޳�
	//	if (intensity > 0) {//���ո�����ͷ��߷���ļнǴ���90��ʱ��intensity<0,���������������˱����޳�
	//		Vec2i uv[3];
	//		for (int k = 0; k < 3; k++) {
	//			uv[k] = model->uv(i, k);
	//		}
	//		triangle(screen_coords[0], screen_coords[1], screen_coords[2], uv[0], uv[1], uv[2], image, intensity,zbuffer);
	//	}
	//}
	/*TGAImage image(width, height, TGAImage::RGB);
	line(100, 100, 300, 100, image, TGAColor(255, 0, 0, 255));
	line(100, 100, 200, 200, image, TGAColor(255, 0, 0, 255));
	line(200, 200, 300, 100, image, TGAColor(255, 0, 0, 255));
	for (int i = 220;i<210;i++)
	{
		image.set(220, i, TGAColor(255, 0, 0, 255));
		image.set(220, 221, TGAColor(255, 0, 0, 255));
	}*/
	
			
	//image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	//image.write_tga_file("output.tga");
	//

	//{ // dump z-buffer (debugging purposes only)
	//	TGAImage zbimage(width, height, TGAImage::GRAYSCALE);
	//	for (int i = 0; i < width; i++) {
	//		for (int j = 0; j < height; j++) {
	//			zbimage.set(i, j, TGAColor(zbuffer[i + j * width], 1));
	//		}
	//	}
	//	zbimage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	//	zbimage.write_tga_file("zbuffer.tga");
	//}
	//delete model;
	//delete[] zbuffer;

	
	return 0;
}