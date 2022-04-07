#include <vector>
#include <cmath>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
Model* model = NULL;
Vec3f light_dir(0, 0, -1);
const int width = 800;
const int height = 800;

void line(int x0,int y0,int x1,int y1,TGAImage& image,TGAColor color)
{
	//哪个差值的绝对值大就对哪个遍历，这样就不会出现缝隙
	bool steep = false;
	if (std::abs(x0 - x1) < std::abs(y0 - y1)) { // if the line is steep, we transpose the image 
		std::swap(x0, y0);
		std::swap(x1, y1);
		steep = true;
	}
	//我们的遍历是从x0到x1，因此要保证x0总是小于x1
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

//Line Sweeping 扫线法填充三角形
void triangle_LineSweeping(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage& image, TGAColor color) {
	// sort the vertices, t0, t1, t2 lower_to_upper (bubble sort)
	if (t0.y > t1.y) std::swap(t0, t1);
	if (t0.y > t2.y) std::swap(t0, t2);
	if (t1.y > t2.y) std::swap(t1, t2);
	int total_height = t2.y - t0.y;
	for (int y = t0.y; y <= t1.y; y++) {
		int segment_height = t1.y - t0.y + 1;
		float alpha = (float)(y - t0.y) / total_height;//线段的两点式
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

// 重心坐标法填充三角形 barycentric coordinates
//triangle(vec2 points[3]) {
//	vec2 bbox[2] = find_bounding_box(points);
//	for (each pixel in the bounding box) {
//		if (inside(points, pixel)) {
//			put_pixel(pixel);
//		}
//	}
//使用了坐标法计算重心坐标的值
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
	float beta = (-dx_pc*dy_ac+dy_pc*dx_ac) / (dx_cb*dy_ac+(-dy_cb*dx_ac));
	float gamma = 1 - alpha - beta;
	return Vec3f(alpha, beta, gamma);
}

void triangle(Vec3f* pts,float* zbuffer, TGAImage& image, TGAColor color) {
	Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
	Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
	//确定boundingbox范围
	for (int i = 0; i < 3; i++) {
		bboxmin.x = std::max(0.f, std::min(bboxmin.x, pts[i].x));
		bboxmin.y = std::max(0.f, std::min(bboxmin.y, pts[i].y));

		bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, pts[i].x));
		bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, pts[i].y));
	}
	Vec3f P;
	for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
		for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
			Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
			P.z = 0;
			for (int i = 0; i < 3; i++) 
			{
				P.z += pts[i][2] * bc_screen[i];//重心坐标插值计算出屏幕每一像素的z值
			} 
			//将模型z值与zbuffer中的值做比较，显示离得近的（看向-z，所以离得近的z值大）
			if (zbuffer[int(P.x+P.y*width)] < P.z)
			{
				zbuffer[int(P.x + P.y * width)] = P.z;
				image.set(P.x,P.y,color);
			}
		}
	}
} //}




int main(int argc, char** argv) {

	//获取模型文件
	if (2 == argc) {
		model = new Model(argv[1]);
	}
	else {
		model = new Model("obj/african_head.obj");
	}
	//用画布大小的一维数组存储zbuffer
	float* zbuffer = new float[width * height];
	for (int i = 0; i < width * height; i++) 
	{
		zbuffer[i] = -std::numeric_limits<float>::max();//看向-z方向，数值越小离得越远
	}
		
	TGAImage image(width, height, TGAImage::RGB);
	for (int i = 0; i < model->nfaces(); i++) {
		std::vector<int> face = model->face(i);
		Vec3f screen_coords[3];
		Vec3f world_coords[3];
		for (int j = 0; j < 3; j++) {
			Vec3f v = model->vert(face[j]);
			screen_coords[j] = Vec3f(int((v.x + 1.) * width / 2.+.5), int((v.y + 1.) * height / 2.+.5),v.z);
			world_coords[j] = v;
		}
		//计算面片法向量
		Vec3f n = cross(world_coords[2] - world_coords[0],world_coords[1] - world_coords[0]);
		n.normalize();
		//可以用 光照负方向*法线方向*cosθ 来表示光照强度
		float intensity = n * light_dir;
		//背面剔除
		if (intensity > 0) {//光照负方向和法线方向的夹角大于90度时，intensity<0,因此这里我们完成了背面剔除
			triangle(screen_coords, zbuffer, image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
		}
	}
		

		
	
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	delete model;

	

	
	return 0;
}