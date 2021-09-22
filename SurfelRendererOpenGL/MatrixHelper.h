#ifndef MATRIX_HELPER_H
#define MATRIX_HELPER_H

#define SMALL_NUMBER 1.e-8

#include <cmath>
#include <string>

static float MtrDeterminant2x2f(const float a, const float b, const float c, const float d) {
	float ans = a * d - b * c;
	return ans;
}

static void MtrCopy3x3f(const float* s, float* d) {
	memcpy(d, s, sizeof(float) * 9);
}

static void MtrTranspose3x3f(const float* s, float* d) {
	for (int i = 0; i < 9; i++)
		d[(i % 3) * 3 + (i / 3)] = s[i];
}

static void MtrMultScal3x3f(const float* s, const float c, float* r) {
	for (int i = 0; i < 9; i++)
		r[i] = s[i] * c;
}

static void MtrAdjoint3x3f(const float* m, float* r)
{
	float a1, a2, a3, b1, b2, b3, c1, c2, c3;

	a1 = m[0]; b1 = m[1]; c1 = m[2];
	a2 = m[3]; b2 = m[4]; c2 = m[5];
	a3 = m[6]; b3 = m[7]; c3 = m[8];

	r[0] = MtrDeterminant2x2f(b2, c2, b3, c3);
	r[3] = -MtrDeterminant2x2f(a2, c2, a3, c3);
	r[6] = MtrDeterminant2x2f(a2, b2, a3, b3);

	r[1] = -MtrDeterminant2x2f(b1, c1, b3, c3);
	r[4] = MtrDeterminant2x2f(a1, c1, a3, c3);
	r[7] = -MtrDeterminant2x2f(a1, b1, a3, b3);

	r[2] = MtrDeterminant2x2f(b1, c1, b2, c2);
	r[5] = -MtrDeterminant2x2f(a1, c1, a2, c2);
	r[8] = MtrDeterminant2x2f(a1, b1, a2, b2);
}

static float MtrDeterminant3x3f(const float a1, const float b1, const float c1,
	const float a2, const float b2, const float c2,
	const float a3, const float b3, const float c3) {
	float ans = a1 * MtrDeterminant2x2f(b2, c2, b3, c3)
		- b1 * MtrDeterminant2x2f(a2, c2, a3, c3)
		+ c1 * MtrDeterminant2x2f(a2, b2, a3, b3);
	return ans;
}

static bool MtrInverse3x3f(const float* m, float* r) {
	MtrAdjoint3x3f(m, r);
	float det = MtrDeterminant3x3f(m[0], m[1], m[2],
		m[3], m[4], m[5],
		m[6], m[7], m[8]);

	if (fabs(det) < SMALL_NUMBER)
		return false;

	for (int i = 0; i < 9; i++)
		r[i] = r[i] / det;

	return true;
}

static void MtrUnity4x4f(float* m) {
	for (int i = 0; i < 16; i++)
		m[i] = 0.0f;
	m[0] = 1.0f;
	m[5] = 1.0f;
	m[10] = 1.0f;
	m[15] = 1.0f;
}

static void MtrCopy4x4f(const float* s, float* d) {
	memcpy(d, s, sizeof(float) * 16);
}

static void MtrTranspose4x4f(const float* s, float* d) {
	for (int i = 0; i < 16; i++)
		d[(i % 4) * 4 + (i / 4)] = s[i];
}

static void MtrMult4x4f(const float* a, const float* b, float* r) {
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			float t = 0;
			for (int k = 0; k < 4; k++)
				t += a[k * 4 + i] * b[k + j * 4];
			r[j * 4 + i] = t;
		}
}

static void MtrCreateRotation4x4fc(const float angle, const float x, const float y, const float z, float* r) {
	double norm;
	double xn, yn, zn;
	double xnyn, xnzn, ynzn;
	double cosa, sina;

	norm = sqrt(x * x + y * y + z * z);
	xn = x / norm;
	yn = y / norm;
	zn = z / norm;

	xnyn = xn * yn;
	xnzn = xn * zn;
	ynzn = yn * zn;

	cosa = cos(angle);
	sina = sin(angle);

	*r++ = (float)(xn * xn + cosa * (1.0 - xn * xn));
	*r++ = (float)(xnyn - cosa * xnyn + sina * zn);
	*r++ = (float)(xnzn - cosa * xnzn + sina * (-yn));
	*r++ = 0.0f;

	*r++ = (float)(xnyn - cosa * xnyn + sina * (-zn));
	*r++ = (float)(yn * yn + cosa * (1.0 - yn * yn));
	*r++ = (float)(ynzn - cosa * ynzn + sina * xn);
	*r++ = 0.0f;

	*r++ = (float)(xnzn - cosa * xnzn + sina * yn);
	*r++ = (float)(ynzn - cosa * ynzn + sina * (-xn));
	*r++ = (float)(zn * zn + cosa * (1.0 - zn * zn));
	*r++ = 0.0f;

	*r++ = 0.0;
	*r++ = 0.0;
	*r++ = 0.0;
	*r++ = 1.0;
}

#endif MATRIX_HELPER_H