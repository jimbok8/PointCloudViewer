#version 430 core

struct Warper {
    float rotation[9];
    float translation[3];
    float normalsRotation[9];
    float scaling;
    float xP, yP;
    float xC, yC;
    float nearplane, farplane;
    float v[24];
};

struct ZBufferItem {
    vec4 color, transformedNormal;
    float zMin, zMax, w;
};

layout(local_size_x = 1024) in;

uniform int width;
uniform int height;

layout(std430, binding = 1) buffer SSBO1 {
    Warper warper;
};

layout(std430, binding = 3) buffer SSBO3 {
    ZBufferItem zBuffer[];
};

layout(std430, binding = 5) buffer SSBO5 {
    float image[];
};

void main() {
    int i = int(gl_GlobalInvocationID.x);
    if (i >= width * height)
        return;

	float vp_sx, vp_sy;
	float vp_tx, vp_ty;

	int index = i * 3;

	vp_sx = 2 * warper.xP / width;
	vp_sy = 2 * warper.yP / height;
	vp_tx = warper.xC - warper.xP;
	vp_ty = warper.yC - warper.yP;

	int zbf_rows = i / width;
	int zbf_cols = i % width;
	float zbf_x_cur = float(zbf_cols) + 0.5f;
	float zbf_y_cur = float(zbf_rows) + 0.5f;

	if (zBuffer[i].w != 0.0f) {
		float w_ = 1.f / zBuffer[i].w;

		float r = zBuffer[i].color.x * w_;
		float g = zBuffer[i].color.y * w_;
		float b = zBuffer[i].color.z * w_;

		float nx = zBuffer[i].transformedNormal.x;
		float ny = zBuffer[i].transformedNormal.y;
		float nz = zBuffer[i].transformedNormal.z;
		w_ = 1.f / sqrt(nx * nx + ny * ny + nz * nz);
		nx *= w_;
		ny *= w_;
		nz *= w_;

		float vx = -(zbf_x_cur * vp_sx + vp_tx);
		float vy = -(zbf_y_cur * vp_sy + vp_ty);
		float vz = -1.f;
		float vec_len_ = 1.f / sqrt(vx * vx + vy * vy + 1.f);
		vx *= vec_len_;
		vy *= vec_len_;
		vz *= vec_len_;

		float resultR, resultG, resultB;
		float Ir, Ig, Ib;
		float Ar, Ag, Ab;
		float Lx, Ly, Lz;
		float Rx, Ry, Rz;
		float t, power, ndotl, rdotv;
		int j;

		float kA = 0.5f;
		float kD = 0.75f;
		float kS = 0.25f;
		int shininess = 0;
		float specularR = 205;
		float specularG = 205;
		float specularB = 205;

		Ir = Ig = Ib = 1.0f;
		Ar = Ag = Ab = 0.5f;

		// ambient contribution
		t = kA;
		resultR = t * Ar * r;
		resultG = t * Ag * g;
		resultB = t * Ab * b;

		Lx = Ly = 0.0f;
		Lz = -1.0f;

		// calculate the N*L dot product
		ndotl = nx * Lx + ny * Ly + nz * Lz;
		ndotl = (ndotl < 0 ? -ndotl : ndotl);

		// calculate normalized reflection vector
		Rx = 2 * nx * ndotl - Lx;
		Ry = 2 * ny * ndotl - Ly;
		Rz = 2 * nz * ndotl - Lz;

		// calculate R*V dot product
		rdotv = vx * Rx + vy * Ry + vz * Rz;
		rdotv = (rdotv < 0 ? -rdotv : rdotv);

		// calculate the phong shininess power
		power = rdotv;
		j = shininess;
		while (j > 0) {
			power *= rdotv;
			j--;
		}

		// increment intensities
		t = kD * ndotl;
		power *= kS;
		resultR += Ir * (t * r + specularR * power);
		resultG += Ig * (t * g + specularG * power);
		resultB += Ib * (t * b + specularB * power);

		if (resultR > 255.0) resultR = 255.0;
		if (resultG > 255.0) resultG = 255.0;
		if (resultB > 255.0) resultB = 255.0;
		image[index] = resultR;
		image[index + 1] = resultG;
		image[index + 2] = resultB;
	}
	else {
		image[index] = 25;
		image[index + 1] = 25;
		image[index + 2] = 25;
	}
}